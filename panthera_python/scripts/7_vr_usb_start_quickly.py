#!/usr/bin/env python3
"""Start USB Quest data relay and a Panthera VR controller together."""

import argparse
import os
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
RELAY_DIR = SCRIPT_DIR / "Quest_controller_stream"
RELAY_SCRIPT = RELAY_DIR / "start_stream_server.py"
CONTROLLER_SCRIPTS = {
    "single": SCRIPT_DIR / "7_vr_cartesian_control.py",
    "dual": SCRIPT_DIR / "7_vr_dual_cartesian_control.py",
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="通过 USB 一键启动 Quest 手柄数据转发和 Panthera VR 遥操",
    )
    parser.add_argument(
        "--controller",
        choices=tuple(CONTROLLER_SCRIPTS),
        default="single",
        help="启动单臂或双臂遥操（默认: single）",
    )
    parser.add_argument("--http-port", type=int, default=8080)
    parser.add_argument("--ws-port", type=int, default=8765)
    parser.add_argument("--udp-ip", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=5005)
    parser.add_argument(
        "--no-open-quest",
        action="store_true",
        help="USB 模式下不自动打开或关闭 Quest Browser",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只显示将执行的命令，不启动进程",
    )
    args = parser.parse_args()

    for name in ("http_port", "ws_port", "udp_port"):
        value = getattr(args, name)
        if not 1 <= value <= 65535:
            parser.error(f"--{name.replace('_', '-')} 必须在 1 到 65535 之间")
    return args


def build_commands(args):
    relay_command = [
        sys.executable,
        "-u",
        str(RELAY_SCRIPT),
        "--http-port",
        str(args.http_port),
        "--ws-port",
        str(args.ws_port),
        "--udp-ip",
        args.udp_ip,
        "--udp-port",
        str(args.udp_port),
        "--adb",
        "always",
    ]
    controller_command = [
        sys.executable,
        "-u",
        str(CONTROLLER_SCRIPTS[args.controller]),
    ]
    return relay_command, controller_command


def _process_script_path(proc_dir, argument):
    """把进程命令行中的脚本参数解析为绝对路径。"""
    path = Path(argument)
    if not path.is_absolute():
        try:
            path = Path(os.readlink(proc_dir / "cwd")) / path
        except OSError:
            return None
    try:
        return path.resolve()
    except OSError:
        return None


def find_stale_relays(http_port, ws_port):
    """查找由本项目启动、且使用相同端口的旧转发进程。"""
    matches = []
    proc_root = Path("/proc")
    for proc_dir in proc_root.iterdir():
        if not proc_dir.name.isdigit():
            continue
        try:
            argv = [
                item.decode(errors="replace")
                for item in (proc_dir / "cmdline").read_bytes().split(b"\0")
                if item
            ]
        except (OSError, PermissionError):
            continue
        if not any(
            _process_script_path(proc_dir, argument) == RELAY_SCRIPT
            for argument in argv[1:]
        ):
            continue
        try:
            old_http_port = int(argv[argv.index("--http-port") + 1])
            old_ws_port = int(argv[argv.index("--ws-port") + 1])
        except (ValueError, IndexError):
            continue
        if old_http_port == http_port or old_ws_port == ws_port:
            matches.append(int(proc_dir.name))
    return matches


def wait_for_process_exit(pid, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return True
        time.sleep(0.05)
    return False


def stop_stale_relays(http_port, ws_port):
    """关闭上次异常退出后遗留的本项目转发服务。"""
    for pid in find_stale_relays(http_port, ws_port):
        try:
            pgid = os.getpgid(pid)
        except ProcessLookupError:
            continue
        print(f"发现旧数据转发服务 PID {pid}，正在清理...", flush=True)
        # 一键启动创建的子进程以自身 PID 为进程组；手动启动的进程只
        # 结束自身，避免向其所在终端的整个进程组发送信号。
        if pgid == pid:
            os.killpg(pgid, signal.SIGINT)
        else:
            os.kill(pid, signal.SIGINT)
        if wait_for_process_exit(pid, 5.0):
            continue
        if pgid == pid:
            os.killpg(pgid, signal.SIGTERM)
        else:
            os.kill(pid, signal.SIGTERM)
        if not wait_for_process_exit(pid, 2.0):
            raise RuntimeError(f"旧数据转发服务 PID {pid} 无法停止")


def ensure_tcp_ports_available(*ports):
    """在启动子进程前给出明确的端口占用错误。"""
    for port in ports:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            # 与 HTTP/WebSocket 服务使用相同的地址复用策略。否则服务
            # 正常退出后残留的 TIME_WAIT 连接会被误判为端口仍被占用。
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("0.0.0.0", port))
        except OSError as exc:
            raise RuntimeError(
                f"TCP 端口 {port} 已被其他程序占用；请停止占用程序，"
                "或通过 --http-port/--ws-port 更换端口"
            ) from exc
        finally:
            sock.close()


def wait_for_relay(process, port, timeout=15.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        return_code = process.poll()
        if return_code is not None:
            raise RuntimeError(f"数据转发服务启动失败，退出码: {return_code}")

        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                time.sleep(0.5)
                if process.poll() is not None:
                    raise RuntimeError("数据转发服务启动后意外退出")
                return
        except OSError:
            time.sleep(0.1)

    raise TimeoutError(f"等待数据转发服务监听 HTTP {port} 端口超时")


def close_quest_browser(announce=True):
    """通过 ADB 结束 Quest Browser，关闭页面和 WebXR 会话。"""
    adb = shutil.which("adb")
    if adb is None:
        if announce:
            print("未找到 adb，无法自动关闭 Quest Browser。", flush=True)
        return False

    try:
        result = subprocess.run(
            [adb, "shell", "am", "force-stop", "com.oculus.browser"],
            check=False,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        if announce:
            print(f"自动关闭 Quest Browser 失败: {exc}", flush=True)
        return False

    if result.returncode != 0:
        if announce:
            message = (result.stderr or result.stdout).strip()
            print(f"自动关闭 Quest Browser 失败: {message}", flush=True)
        return False

    if announce:
        print("已自动关闭 Quest Browser 页面。", flush=True)
    return True


def open_quest_browser(http_port, ws_port):
    adb = shutil.which("adb")
    if adb is None:
        print("未找到 adb，跳过自动打开 Quest Browser。", flush=True)
        return False

    url = (
        f"http://localhost:{http_port}/index.html"
        f"?ws=ws://localhost:{ws_port}"
    )
    # Quest Browser 会把沉浸式 WebXR 页面保留在独立的 WebVRActivity
    # 中。直接再次发送 VIEW Intent 只会在普通面板打开 USB 页面，头显
    # 仍可能显示上一次 WiFi 页面创建的旧 WebVRActivity。启动前结束旧
    # 浏览器会话，确保本次进入的是 localhost + adb reverse 链路。
    if not close_quest_browser(announce=False):
        print("结束旧 Quest Browser 会话失败，继续尝试打开页面。", flush=True)

    result = subprocess.run(
        [
            adb,
            "shell",
            "am",
            "start",
            "-a",
            "android.intent.action.VIEW",
            "-d",
            url,
            "-p",
            "com.oculus.browser",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode == 0:
        print(f"已重置 Quest Browser 并打开 USB 页面: {url}", flush=True)
        return True
    else:
        message = (result.stderr or result.stdout).strip()
        print(f"自动打开 Quest Browser 失败: {message}", flush=True)
        print(f"请在 Quest Browser 中手动打开: {url}", flush=True)
        return False


def signal_process_group(process, sig):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, sig)
    except ProcessLookupError:
        pass


def stop_process(process, name, graceful_timeout, send_interrupt=True):
    if process is None or process.poll() is not None:
        return

    print(f"正在停止{name}...", flush=True)
    if send_interrupt:
        signal_process_group(process, signal.SIGINT)
    try:
        process.wait(timeout=graceful_timeout)
        return
    except subprocess.TimeoutExpired:
        print(f"{name}未在 {graceful_timeout:.0f} 秒内退出，发送 SIGTERM。", flush=True)

    signal_process_group(process, signal.SIGTERM)
    try:
        process.wait(timeout=5.0)
        return
    except subprocess.TimeoutExpired:
        print(f"{name}仍未退出，强制结束。", flush=True)

    signal_process_group(process, signal.SIGKILL)
    process.wait()


def raise_keyboard_interrupt(_signum, _frame):
    raise KeyboardInterrupt


def shutdown_in_progress(_signum, _frame):
    print("\n正在安全退出，请等待机械臂完成回零。", flush=True)


def main():
    args = parse_args()
    relay_command, controller_command = build_commands(args)

    if not RELAY_SCRIPT.is_file():
        raise FileNotFoundError(f"找不到数据转发脚本: {RELAY_SCRIPT}")
    if not CONTROLLER_SCRIPTS[args.controller].is_file():
        raise FileNotFoundError(
            f"找不到遥操脚本: {CONTROLLER_SCRIPTS[args.controller]}"
        )

    print(f"数据转发: {shlex.join(relay_command)}")
    print(f"遥操接收: {shlex.join(controller_command)}")
    if args.dry_run:
        return 0

    signal.signal(signal.SIGTERM, raise_keyboard_interrupt)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, raise_keyboard_interrupt)

    relay_process = None
    controller_process = None
    quest_browser_managed = False
    exit_code = 0
    try:
        stop_stale_relays(args.http_port, args.ws_port)
        ensure_tcp_ports_available(args.http_port, args.ws_port)
        print("\n正在启动 Quest 数据转发服务...", flush=True)
        relay_process = subprocess.Popen(
            relay_command,
            cwd=RELAY_DIR,
            start_new_session=True,
        )
        wait_for_relay(relay_process, args.http_port)
        print("数据转发服务已就绪。", flush=True)

        if not args.no_open_quest:
            quest_browser_managed = open_quest_browser(
                args.http_port,
                args.ws_port,
            )

        print("请在 Quest 页面点击 Start XR Stream。", flush=True)
        print("正在启动遥操接收脚本...\n", flush=True)
        controller_process = subprocess.Popen(
            controller_command,
            cwd=SCRIPT_DIR,
            start_new_session=True,
        )

        while True:
            controller_code = controller_process.poll()
            relay_code = relay_process.poll()
            if controller_code is not None:
                exit_code = controller_code
                break
            if relay_code is not None:
                print(f"\n数据转发服务意外退出，退出码: {relay_code}", flush=True)
                exit_code = relay_code or 1
                break
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n收到退出请求。将先等待机械臂回零，再关闭数据转发。", flush=True)
        exit_code = 130
    except (OSError, RuntimeError, TimeoutError) as exc:
        print(f"\n启动失败: {exc}", file=sys.stderr, flush=True)
        exit_code = 1
    finally:
        # 清理开始后不再让重复 Ctrl+C 打断 process.wait，否则转发服务
        # 会成为残留进程，在启动器退出后继续向终端打印。
        signal.signal(signal.SIGINT, shutdown_in_progress)
        signal.signal(signal.SIGTERM, shutdown_in_progress)
        if hasattr(signal, "SIGHUP"):
            signal.signal(signal.SIGHUP, shutdown_in_progress)

        # 同时通知两个子进程退出。遥操脚本会独立完成保持和回零，退出
        # 数据转发不会影响该流程，并能立即停止 forwarding 输出。
        signal_process_group(controller_process, signal.SIGINT)
        signal_process_group(relay_process, signal.SIGINT)
        if quest_browser_managed:
            close_quest_browser()
        try:
            stop_process(
                controller_process,
                "遥操脚本（等待机械臂回零）",
                30.0,
                send_interrupt=False,
            )
        finally:
            # 即使等待遥操脚本出现异常，也必须回收数据转发进程。
            stop_process(
                relay_process,
                "数据转发服务",
                5.0,
                send_interrupt=False,
            )

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
