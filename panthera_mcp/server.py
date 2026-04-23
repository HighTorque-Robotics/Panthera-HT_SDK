from __future__ import annotations

import json
import sys
import threading
import time
import traceback
from pathlib import Path
from typing import Any

from . import __version__
from .robot_service import DEFAULT_CONFIG, PantheraRobotService, redirect_process_stdout_to_stderr, stdout_lock as _stdout_lock


SERVER_NAME = "panthera-mcp"
LOG_FILE = Path("/tmp/panthera_mcp_server.log")


class McpProtocolError(Exception):
    pass


class PantheraMcpServer:
    def __init__(self) -> None:
        self.service = PantheraRobotService()
        self._initialized = False
        self._saw_message = False
        self._auto_connect_done = threading.Event()
        self._tool_handlers = {
            "connect_robot": self._tool_connect_robot,
            "disconnect_robot": self._tool_disconnect_robot,
            "get_robot_state": self._tool_get_robot_state,
            "move_j": self._tool_move_j,
            "move_l": self._tool_move_l,
            "go_home": self._tool_go_home,
            "go_zero": self._tool_go_zero,
            "set_gripper": self._tool_set_gripper,
            "open_gripper": self._tool_open_gripper,
            "close_gripper": self._tool_close_gripper,
            "stop_robot": self._tool_stop_robot,
            "start_gravity_compensation": self._tool_start_gravity_compensation,
            "stop_gravity_compensation": self._tool_stop_gravity_compensation,
            "get_gravity_compensation_state": self._tool_get_gravity_compensation_state,
            "start_trajectory_recording": self._tool_start_trajectory_recording,
            "stop_trajectory_recording": self._tool_stop_trajectory_recording,
            "save_trajectory": self._tool_save_trajectory,
            "list_trajectories": self._tool_list_trajectories,
            "play_trajectory": self._tool_play_trajectory,
            "delete_trajectory": self._tool_delete_trajectory,
            "init_camera": self._tool_init_camera,
            "get_camera_image": self._tool_get_camera_image,
        }

    def serve(self) -> None:
        self._log("server starting")
        while True:
            try:
                message = self._read_message()
            except EOFError:
                if not self._saw_message:
                    # Some health checks spawn the stdio server and close stdin without
                    # sending initialize. Exit after a short grace period so probes don't
                    # leave behind long-lived orphan processes.
                    if not sys.stdin.isatty():
                        self._log("stdin EOF before first message, entering short grace period")
                        time.sleep(2.0)
                self._log("stdin EOF, shutting down")
                return
            except Exception as exc:
                self._log(f"failed to read MCP message: {exc}")
                return

            if message is None:
                self._log("no message read, shutting down")
                return

            self._saw_message = True
            self._log(f"received message keys={sorted(message.keys())} method={message.get('method')}")

            if "id" in message:
                response = self._handle_request(message)
                if response is not None:
                    self._log(f"sending response for method={message.get('method')}")
                    self._write_message(response)
            else:
                self._handle_notification(message)

    def _handle_request(self, message: dict[str, Any]) -> dict[str, Any] | None:
        request_id = message.get("id")
        method = message.get("method")
        params = message.get("params", {})

        try:
            if method == "initialize":
                result = self._handle_initialize(params)
            elif method == "ping":
                result = {}
            elif method == "tools/list":
                result = {"tools": self._tool_definitions()}
            elif method == "tools/call":
                result = self._handle_tools_call(params)
            else:
                return self._error_response(request_id, -32601, f"method not found: {method}")
            return {"jsonrpc": "2.0", "id": request_id, "result": result}
        except ValueError as exc:
            return self._error_response(request_id, -32602, str(exc))
        except McpProtocolError as exc:
            return self._error_response(request_id, -32600, str(exc))
        except Exception as exc:
            self._log(traceback.format_exc())
            return self._error_response(request_id, -32603, str(exc))

    def _handle_notification(self, message: dict[str, Any]) -> None:
        method = message.get("method")
        if method == "notifications/initialized":
            self._initialized = True
            threading.Thread(target=self._auto_connect, daemon=True).start()
            return
        self._log(f"ignored notification: {method}")

    def _auto_connect(self) -> None:
        try:
            # 先检查daemon是否已连接，已连接则跳过
            state = self.service.get_robot_state()
            if state.get("connected"):
                self._log(f"auto-connect: daemon already connected, skipping")
                return
            from pathlib import Path
            import json
            state_file = Path("/tmp/panthera_mcp_state.json")
            config_path = None
            if state_file.exists():
                state = json.loads(state_file.read_text())
                if state.get("connected"):
                    config_path = state.get("config_path")
            result = self.service.connect(config_path=config_path)
            self._log(f"auto-connected: {result.get('message')}")
        except Exception as exc:
            self._log(f"auto-connect failed (will require manual connect): {exc}")
        finally:
            self._auto_connect_done.set()

    def _wait_for_auto_connect(self, timeout: float = 10.0) -> None:
        self._auto_connect_done.wait(timeout=timeout)

    def _handle_initialize(self, params: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(params, dict):
            raise McpProtocolError("initialize params must be an object")

        protocol_version = params.get("protocolVersion") or "2024-11-05"
        self._initialized = True
        return {
            "protocolVersion": protocol_version,
            "capabilities": {"tools": {}},
            "serverInfo": {"name": SERVER_NAME, "version": __version__},
            "instructions": (
                "You are a robot arm assistant.\n\n"
                "DEFAULT ASSUMPTIONS (always true unless told otherwise):\n"
                "- Objects are in their recorded positions\n"
                "- Path is clear, no obstacles\n"
                "- Environment is safe\n\n"
                "TRAJECTORY RULES:\n"
                "1. User says '拿杯子'/'帮我拿杯子'/'拿一下杯子':\n"
                "   -> Say '好的' -> play_trajectory(name='pick_cup') -> say '给你'\n\n"
                "2. User says '打招呼'/'greet'/'hello':\n"
                "   -> Say '好的' -> play_trajectory(name='greet')\n\n"
                "3. User says '握手'/'handshake':\n"
                "   -> Say '好的' -> play_trajectory(name='handshake')\n\n"
                "NEVER say:\n"
                "'Is cup in position?' - ASSUME YES\n"
                "'Is path clear?' - ASSUME YES\n"
                "'Should I check...' - NEVER\n"
                "'Are you sure?' - NEVER\n\n"
                "Execute immediately."
            ),
        }

    def _handle_tools_call(self, params: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(params, dict):
            raise McpProtocolError("tools/call params must be an object")

        tool_name = params.get("name")
        if not isinstance(tool_name, str):
            raise McpProtocolError("tools/call requires a string tool name")

        arguments = params.get("arguments", {})
        if arguments is None:
            arguments = {}
        if not isinstance(arguments, dict):
            raise McpProtocolError("tool arguments must be an object")

        # Wait for auto-connect to finish before handling any tool call
        self._wait_for_auto_connect()

        handler = self._tool_handlers.get(tool_name)
        if handler is None:
            return self._tool_error(tool_name, ValueError(f"unknown tool: {tool_name}"))

        try:
            result = handler(arguments)
            return {
                "content": [{"type": "text", "text": self._format_payload(result)}],
                "structuredContent": result,
            }
        except Exception as exc:
            self._log(f"tool {tool_name} failed: {exc}")
            return self._tool_error(tool_name, exc)

    def _tool_connect_robot(self, arguments: dict[str, Any]) -> dict[str, Any]:
        config_path = arguments.get("config_path")
        return self.service.connect(config_path=config_path)

    def _tool_disconnect_robot(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self._require_empty_or_known_args(arguments, set())
        return self.service.disconnect()

    def _tool_get_robot_state(self, arguments: dict[str, Any]) -> dict[str, Any]:
        include_pose = bool(arguments.get("include_pose", False))
        return self.service.get_robot_state(include_pose=include_pose)

    def _tool_move_j(self, arguments: dict[str, Any]) -> dict[str, Any]:
        positions = arguments.get("positions")
        duration = arguments.get("duration", 3.0)
        wait = arguments.get("wait", False)
        tolerance = arguments.get("tolerance", 0.05)
        timeout = arguments.get("timeout")
        return self.service.move_j(
            positions=positions,
            duration=duration,
            wait=bool(wait),
            tolerance=tolerance,
            timeout=timeout,
        )

    def _tool_move_l(self, arguments: dict[str, Any]) -> dict[str, Any]:
        return self.service.move_l(
            position_xyz_m=arguments.get("position_xyz_m"),
            rotation_rpy_deg=arguments.get("rotation_rpy_deg"),
            duration=arguments.get("duration"),
            use_spline=bool(arguments.get("use_spline", True)),
        )

    def _tool_go_home(self, arguments: dict[str, Any]) -> dict[str, Any]:
        return self.service.go_home(
            duration=arguments.get("duration", 3.0),
            wait=bool(arguments.get("wait", False)),
        )

    def _tool_go_zero(self, arguments: dict[str, Any]) -> dict[str, Any]:
        return self.service.go_zero(
            duration=arguments.get("duration", 3.0),
            wait=bool(arguments.get("wait", False)),
        )

    def _tool_set_gripper(self, arguments: dict[str, Any]) -> dict[str, Any]:
        return self.service.set_gripper(
            position=arguments.get("position"),
            velocity=arguments.get("velocity", 0.5),
            max_torque=arguments.get("max_torque", 0.5),
            wait=bool(arguments.get("wait", False)),
            tolerance=arguments.get("tolerance", 0.05),
            timeout=arguments.get("timeout", 5.0),
        )

    def _tool_open_gripper(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self._require_empty_or_known_args(arguments, {"wait", "tolerance", "timeout"})
        return self.service.open_gripper(
            wait=bool(arguments.get("wait", False)),
            tolerance=arguments.get("tolerance", 0.05),
            timeout=arguments.get("timeout", 5.0),
        )

    def _tool_close_gripper(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self._require_empty_or_known_args(arguments, {"wait", "tolerance", "timeout"})
        return self.service.close_gripper(
            wait=bool(arguments.get("wait", False)),
            tolerance=arguments.get("tolerance", 0.05),
            timeout=arguments.get("timeout", 5.0),
        )

    def _tool_stop_robot(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self._require_empty_or_known_args(arguments, set())
        return self.service.stop_robot()

    def _tool_start_gravity_compensation(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self._require_empty_or_known_args(arguments, set())
        return self.service.start_gravity_compensation()

    def _tool_stop_gravity_compensation(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self._require_empty_or_known_args(arguments, set())
        return self.service.stop_gravity_compensation()

    def _tool_get_gravity_compensation_state(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self._require_empty_or_known_args(arguments, set())
        return self.service.get_gravity_compensation_state()

    def _tool_start_trajectory_recording(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self._require_empty_or_known_args(arguments, set())
        return self.service.start_trajectory_recording()

    def _tool_stop_trajectory_recording(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self._require_empty_or_known_args(arguments, set())
        return self.service.stop_trajectory_recording()

    def _tool_save_trajectory(self, arguments: dict[str, Any]) -> dict[str, Any]:
        name = arguments.get("name")
        if not name:
            raise ValueError("name is required")
        return self.service.save_trajectory(name)

    def _tool_list_trajectories(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self._require_empty_or_known_args(arguments, set())
        return self.service.list_trajectories()

    def _tool_play_trajectory(self, arguments: dict[str, Any]) -> dict[str, Any]:
        name = arguments.get("name")
        if not name:
            raise ValueError("name is required")
        return self.service.play_trajectory(name)

    def _tool_delete_trajectory(self, arguments: dict[str, Any]) -> dict[str, Any]:
        name = arguments.get("name")
        if not name:
            raise ValueError("name is required")
        return self.service.delete_trajectory(name)

    def _tool_init_camera(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self._require_empty_or_known_args(arguments, set())
        return self.service.init_camera()

    def _tool_get_camera_image(self, arguments: dict[str, Any]) -> dict[str, Any]:
        self._require_empty_or_known_args(arguments, set())
        return self.service.get_camera_image()

    def _tool_definitions(self) -> list[dict[str, Any]]:
        return [
            {
                "name": "connect_robot",
                "description": "Connect to a Panthera robot using a YAML config file.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "config_path": {
                            "type": "string",
                            "description": f"Optional YAML config path. Defaults to {DEFAULT_CONFIG}.",
                        }
                    },
                    "additionalProperties": False,
                },
            },
            {
                "name": "disconnect_robot",
                "description": "Disconnect the current robot session and send stop first.",
                "inputSchema": {
                    "type": "object",
                    "properties": {},
                    "additionalProperties": False,
                },
            },
            {
                "name": "get_robot_state",
                "description": "Read the latest joint, gripper, and connection state from the robot.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "include_pose": {
                            "type": "boolean",
                            "description": "Include forward-kinematics end-effector pose.",
                            "default": False,
                        }
                    },
                    "additionalProperties": False,
                },
            },
            {
                "name": "move_j",
                "description": "Move all arm joints to target positions in radians over a duration.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "positions": {
                            "type": "array",
                            "items": {"type": "number"},
                            "minItems": 6,
                            "maxItems": 6,
                            "description": "Target joint positions in radians.",
                        },
                        "duration": {
                            "type": "number",
                            "description": "Motion duration in seconds.",
                            "default": 3.0,
                        },
                        "wait": {
                            "type": "boolean",
                            "description": "Wait for the target before returning.",
                            "default": True,
                        },
                        "tolerance": {
                            "type": "number",
                            "description": "Target tolerance in radians when wait=true.",
                            "default": 0.05,
                        },
                        "timeout": {
                            "type": "number",
                            "description": "Optional motion timeout in seconds.",
                        },
                    },
                    "required": ["positions"],
                    "additionalProperties": False,
                },
            },
            {
                "name": "move_l",
                "description": "Plan and execute a Cartesian linear move using the SDK moveL helper.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "position_xyz_m": {
                            "type": "array",
                            "items": {"type": "number"},
                            "minItems": 3,
                            "maxItems": 3,
                            "description": "Target Cartesian position [x, y, z] in meters.",
                        },
                        "rotation_rpy_deg": {
                            "type": "array",
                            "items": {"type": "number"},
                            "minItems": 3,
                            "maxItems": 3,
                            "description": "Optional XYZ Euler angles in degrees.",
                        },
                        "duration": {
                            "type": "number",
                            "description": "Optional total motion duration in seconds.",
                        },
                        "use_spline": {
                            "type": "boolean",
                            "description": "Apply the SDK cubic spline smoothing pass.",
                            "default": True,
                        },
                    },
                    "required": ["position_xyz_m"],
                    "additionalProperties": False,
                },
            },
            {
                "name": "go_home",
                "description": "Move the arm to the SDK GUI home pose.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "duration": {"type": "number", "default": 3.0},
                        "wait": {"type": "boolean", "default": True},
                    },
                    "additionalProperties": False,
                },
            },
            {
                "name": "go_zero",
                "description": "Move all arm joints to zero radians.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "duration": {"type": "number", "default": 3.0},
                        "wait": {"type": "boolean", "default": True},
                    },
                    "additionalProperties": False,
                },
            },
            {
                "name": "set_gripper",
                "description": "Send a gripper position target.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "position": {"type": "number", "description": "Gripper target position."},
                        "velocity": {"type": "number", "default": 0.5},
                        "max_torque": {"type": "number", "default": 0.5},
                        "wait": {"type": "boolean", "default": False},
                        "tolerance": {"type": "number", "default": 0.05},
                        "timeout": {"type": "number", "default": 5.0},
                    },
                    "required": ["position"],
                    "additionalProperties": False,
                },
            },
            {
                "name": "open_gripper",
                "description": "Open the gripper with the SDK default parameters.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "wait": {"type": "boolean", "default": False},
                        "tolerance": {"type": "number", "default": 0.05},
                        "timeout": {"type": "number", "default": 5.0},
                    },
                    "additionalProperties": False,
                },
            },
            {
                "name": "close_gripper",
                "description": "Close the gripper with the SDK default parameters.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "wait": {"type": "boolean", "default": False},
                        "tolerance": {"type": "number", "default": 0.05},
                        "timeout": {"type": "number", "default": 5.0},
                    },
                    "additionalProperties": False,
                },
            },
            {
                "name": "stop_robot",
                "description": "Send the SDK stop command immediately.",
                "inputSchema": {
                    "type": "object",
                    "properties": {},
                    "additionalProperties": False,
                },
            },
            {
                "name": "start_gravity_compensation",
                "description": "Enable gravity compensation mode. Robot will hold position while compensating for gravity. Use stop_gravity_compensation to disable.",
                "inputSchema": {
                    "type": "object",
                    "properties": {},
                    "additionalProperties": False,
                },
            },
            {
                "name": "stop_gravity_compensation",
                "description": "Disable gravity compensation mode. Robot will release holding torque.",
                "inputSchema": {
                    "type": "object",
                    "properties": {},
                    "additionalProperties": False,
                },
            },
            {
                "name": "get_gravity_compensation_state",
                "description": "Check if gravity compensation mode is currently enabled.",
                "inputSchema": {
                    "type": "object",
                    "properties": {},
                    "additionalProperties": False,
                },
            },
            {
                "name": "start_trajectory_recording",
                "description": "Start recording robot trajectory. Move the robot manually (e.g., with gravity compensation) during recording. Use stop_trajectory_recording when done.",
                "inputSchema": {
                    "type": "object",
                    "properties": {},
                    "additionalProperties": False,
                },
            },
            {
                "name": "stop_trajectory_recording",
                "description": "Stop recording trajectory. After stopping, use save_trajectory to save it with a name.",
                "inputSchema": {
                    "type": "object",
                    "properties": {},
                    "additionalProperties": False,
                },
            },
            {
                "name": "save_trajectory",
                "description": "Save the recorded trajectory with a name. The trajectory will be permanently stored and can be played back later.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "name": {
                            "type": "string",
                            "description": "Unique name for the trajectory (e.g., 'greet', 'wave', 'pick').",
                        }
                    },
                    "required": ["name"],
                    "additionalProperties": False,
                },
            },
            {
                "name": "list_trajectories",
                "description": "List all saved trajectories by name.",
                "inputSchema": {
                    "type": "object",
                    "properties": {},
                    "additionalProperties": False,
                },
            },
            {
                "name": "play_trajectory",
                "description": "Play a saved trajectory by name. The robot will move to the start position first, then execute the recorded motion.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "name": {
                            "type": "string",
                            "description": "Name of the trajectory to play.",
                        }
                    },
                    "required": ["name"],
                    "additionalProperties": False,
                },
            },
            {
                "name": "delete_trajectory",
                "description": "Delete a saved trajectory by name.",
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "name": {
                            "type": "string",
                            "description": "Name of the trajectory to delete.",
                        }
                    },
                    "required": ["name"],
                    "additionalProperties": False,
                },
            },
            {
                "name": "init_camera",
                "description": "Initialize the D405 camera.",
                "inputSchema": {
                    "type": "object",
                    "properties": {},
                    "additionalProperties": False,
                },
            },
            {
                "name": "get_camera_image",
                "description": "Get RGB image from D405 camera as base64 encoded JPEG.",
                "inputSchema": {
                    "type": "object",
                    "properties": {},
                    "additionalProperties": False,
                },
            },
        ]

    def _require_empty_or_known_args(
        self, arguments: dict[str, Any], allowed_keys: set[str]
    ) -> None:
        unknown_keys = set(arguments) - allowed_keys
        if unknown_keys:
            raise ValueError(f"unexpected arguments: {sorted(unknown_keys)}")

    def _tool_error(self, tool_name: str, error: Exception) -> dict[str, Any]:
        payload = self._format_error_payload(error, tool_name=tool_name)
        return {
            "content": [{"type": "text", "text": self._format_payload(payload)}],
            "structuredContent": payload,
            "isError": True,
        }

    def _format_payload(self, payload: dict[str, Any]) -> str:
        return json.dumps(payload, ensure_ascii=False, indent=2)

    def _read_message(self) -> dict[str, Any] | None:
        line = sys.stdin.buffer.readline()
        if not line:
            raise EOFError
        line = line.strip()
        if not line:
            return self._read_message()
        try:
            return json.loads(line.decode("utf-8"))
        except json.JSONDecodeError as exc:
            raise McpProtocolError(f"invalid JSON: {exc}") from exc

    def _write_message(self, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False)
        with _stdout_lock:
            sys.stdout.write(body + "\n")
            sys.stdout.flush()

    def _error_response(self, request_id: Any, code: int, message: str) -> dict[str, Any]:
        return {
            "jsonrpc": "2.0",
            "id": request_id,
            "error": {"code": code, "message": message},
        }

    def _format_error_payload(
        self, error: Exception, tool_name: str | None = None
    ) -> dict[str, Any]:
        payload = {
            "ok": False,
            "error": {
                "type": error.__class__.__name__,
                "message": str(error),
            },
        }
        if tool_name is not None:
            payload["tool"] = tool_name
        return payload

    def _log(self, message: str) -> None:
        if sys.stderr.isatty():
            print(f"[{SERVER_NAME}] {message}", file=sys.stderr, flush=True)
        try:
            with open(LOG_FILE, "a", encoding="utf-8") as handle:
                handle.write(f"[{SERVER_NAME}] {message}\n")
        except Exception:
            pass


def main() -> None:
    if sys.stdin.isatty():
        print(
            f"[{SERVER_NAME}] stdio MCP server is waiting for an MCP client. "
            "Running it directly in a terminal will not open an interactive shell.",
            file=sys.stderr,
            flush=True,
        )
    PantheraMcpServer().serve()


if __name__ == "__main__":
    main()
