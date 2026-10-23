import argparse
import asyncio
import contextlib
import functools
import http.server
import pathlib
import shutil
import socket
import socketserver
import ssl
import subprocess
import sys
import threading
import time

try:
    import websockets
except ModuleNotFoundError:
    print("Missing dependency: websockets", file=sys.stderr)
    print("Install it with: pip install websockets", file=sys.stderr)
    raise


class QuietHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        return


class ReusableThreadingTCPServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True


def detect_local_ip() -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        sock.close()


def start_http_server(
    host: str,
    port: int,
    directory: pathlib.Path,
    ssl_context: ssl.SSLContext | None = None,
):
    handler = functools.partial(QuietHTTPRequestHandler, directory=str(directory))
    httpd = ReusableThreadingTCPServer((host, port), handler)
    httpd.daemon_threads = True
    if ssl_context is not None:
        httpd.socket = ssl_context.wrap_socket(httpd.socket, server_side=True)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    return httpd


def ensure_self_signed_cert(cert_file: pathlib.Path, key_file: pathlib.Path, local_ip: str):
    if cert_file.exists() and key_file.exists():
        return

    openssl_path = shutil.which("openssl")
    if not openssl_path:
        raise RuntimeError("openssl not found; cannot generate HTTPS certificate.")

    cert_file.parent.mkdir(parents=True, exist_ok=True)
    san = f"subjectAltName=DNS:localhost,IP:127.0.0.1,IP:{local_ip}"
    command = [
        openssl_path,
        "req",
        "-x509",
        "-nodes",
        "-newkey",
        "rsa:2048",
        "-keyout",
        str(key_file),
        "-out",
        str(cert_file),
        "-days",
        "365",
        "-subj",
        "/CN=localhost",
        "-addext",
        san,
    ]
    subprocess.run(command, check=True, capture_output=True, text=True)


def create_ssl_context(cert_file: pathlib.Path, key_file: pathlib.Path) -> ssl.SSLContext:
    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_context.load_cert_chain(certfile=str(cert_file), keyfile=str(key_file))
    return ssl_context


def configure_adb_reverse(http_port: int, ws_port: int) -> tuple[bool, str]:
    adb_path = shutil.which("adb")
    if not adb_path:
        return False, "adb not found; skipping USB reverse setup."

    try:
        devices = subprocess.run(
            [adb_path, "devices"],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        return False, f"adb devices failed: {exc}"

    connected = []
    for line in devices.stdout.splitlines()[1:]:
        line = line.strip()
        if not line or "\tdevice" not in line:
            continue
        connected.append(line.split("\t", 1)[0])

    if not connected:
        return False, "no adb device detected; skipping USB reverse setup."

    commands = [
        [adb_path, "reverse", "--remove-all"],
        [adb_path, "reverse", f"tcp:{http_port}", f"tcp:{http_port}"],
        [adb_path, "reverse", f"tcp:{ws_port}", f"tcp:{ws_port}"],
    ]
    try:
        for command in commands:
            subprocess.run(command, check=True, capture_output=True, text=True)
    except (OSError, subprocess.CalledProcessError) as exc:
        return False, f"adb reverse setup failed: {exc}"

    return True, f"adb reverse ready for devices: {', '.join(connected)}"


class UDPRelay:
    def __init__(self, udp_ip: str, udp_port: int):
        self.udp_target = (udp_ip, udp_port)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.packet_count = 0
        self.last_stat_time = time.time()

    async def handler(self, websocket):
        print("Quest connected", flush=True)
        try:
            async for message in websocket:
                if isinstance(message, bytes):
                    self.udp_sock.sendto(message, self.udp_target)
                    self.packet_count += 1

                    now = time.time()
                    if now - self.last_stat_time >= 1.0:
                        print(
                            f"forwarding {self.packet_count} packets/sec to "
                            f"{self.udp_target[0]}:{self.udp_target[1]}",
                            flush=True,
                        )
                        self.packet_count = 0
                        self.last_stat_time = now
                else:
                    print(f"text message: {message[:200]}", flush=True)
        except websockets.ConnectionClosed as exc:
            print(f"Quest connection closed: {exc}", flush=True)
        finally:
            print(
                f"Quest disconnected: code={getattr(websocket, 'close_code', None)} "
                f"reason={getattr(websocket, 'close_reason', None)!r}",
                flush=True,
            )

    def close(self):
        self.udp_sock.close()


async def main():
    parser = argparse.ArgumentParser(
        description="Serve index.html and relay Quest WebSocket packets to UDP.",
    )
    parser.add_argument("--http-host", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8080)
    parser.add_argument("--ws-host", default="0.0.0.0")
    parser.add_argument("--ws-port", type=int, default=8765)
    parser.add_argument("--udp-ip", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=5005)
    parser.add_argument(
        "--https",
        action="store_true",
        help="Serve HTTPS/WSS with a self-signed certificate.",
    )
    parser.add_argument("--cert-file", default=".certs/quest-controller.crt")
    parser.add_argument("--key-file", default=".certs/quest-controller.key")
    parser.add_argument(
        "--adb",
        choices=["auto", "always", "never"],
        default="auto",
        help="Configure adb reverse for USB mode. Default: auto.",
    )
    args = parser.parse_args()

    base_dir = pathlib.Path(__file__).resolve().parent
    local_ip = detect_local_ip()
    cert_path = (base_dir / args.cert_file).resolve()
    key_path = (base_dir / args.key_file).resolve()
    adb_ok = False
    adb_message = "adb reverse disabled."
    ssl_context = None
    page_scheme = "http"
    ws_scheme = "ws"
    https_enabled = args.https

    # WiFi mode should default to a secure context for WebXR.
    if args.adb == "never":
        https_enabled = True

    if args.adb != "never":
        adb_ok, adb_message = configure_adb_reverse(args.http_port, args.ws_port)
        if args.adb == "always" and not adb_ok:
            raise RuntimeError(adb_message)

    if https_enabled:
        ensure_self_signed_cert(cert_path, key_path, local_ip)
        ssl_context = create_ssl_context(cert_path, key_path)
        page_scheme = "https"
        ws_scheme = "wss"

    httpd = start_http_server(args.http_host, args.http_port, base_dir, ssl_context=ssl_context)
    relay = UDPRelay(args.udp_ip, args.udp_port)

    usb_page_url = f"{page_scheme}://localhost:{args.http_port}/index.html"
    usb_page_url_with_ws = (
        f"{page_scheme}://localhost:{args.http_port}/index.html"
        f"?ws={ws_scheme}://localhost:{args.ws_port}"
    )
    page_url = f"{page_scheme}://{local_ip}:{args.http_port}/index.html"
    page_url_with_ws = (
        f"{page_scheme}://{local_ip}:{args.http_port}/index.html"
        f"?ws={ws_scheme}://{local_ip}:{args.ws_port}"
    )

    print(f"HTTP serving {base_dir}", flush=True)
    print(f"USB access: {usb_page_url}", flush=True)
    print(f"USB access with ws param: {usb_page_url_with_ws}", flush=True)
    print(f"WiFi access: {page_url}", flush=True)
    print(f"WiFi access with ws param: {page_url_with_ws}", flush=True)
    print(f"WS relay listening on {ws_scheme}://{args.ws_host}:{args.ws_port}", flush=True)
    print(f"UDP forwarding to {args.udp_ip}:{args.udp_port}", flush=True)
    print(adb_message, flush=True)
    if args.adb == "never" and not args.https:
        print("WiFi mode detected: HTTPS/WSS auto-enabled.", flush=True)
    if https_enabled:
        print(f"HTTPS cert: {cert_path}", flush=True)
        print(
            "If Quest warns about the certificate, open the HTTPS page once and accept it manually.",
            flush=True,
        )

    try:
        async with websockets.serve(
            relay.handler,
            args.ws_host,
            args.ws_port,
            max_size=None,
            ssl=ssl_context,
        ):
            await asyncio.Future()
    finally:
        relay.close()
        with contextlib.suppress(Exception):
            httpd.shutdown()
        with contextlib.suppress(Exception):
            httpd.server_close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutting down.", flush=True)
    except OSError as exc:
        if exc.errno == 98:
            print("Port already in use. Stop the previous server or change --http-port / --ws-port.", file=sys.stderr)
        raise
