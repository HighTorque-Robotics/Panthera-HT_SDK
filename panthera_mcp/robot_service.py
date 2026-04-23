"""
Thin proxy that forwards all robot operations to the persistent daemon process.
The MCP server imports this module; the daemon owns the actual robot connection.
"""
from __future__ import annotations

import contextlib
import os
import sys
import threading
from pathlib import Path
from typing import Any

from .robot_daemon import call_daemon, start_daemon_if_needed

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = PROJECT_ROOT / "panthera_python" / "scripts"
DEFAULT_CONFIG = PROJECT_ROOT / "panthera_python" / "robot_param" / "Follower.yaml"

# Keep these for any code that imports them from here
stdout_lock = threading.Lock()


@contextlib.contextmanager
def redirect_process_stdout_to_stderr() -> Any:
    """No-op: stdout redirection is handled inside the daemon now."""
    yield


class PantheraRobotService:
    def __init__(self) -> None:
        start_daemon_if_needed()

    def connect(self, config_path: str | None = None) -> dict[str, Any]:
        return call_daemon("connect", {"config_path": config_path})

    def disconnect(self) -> dict[str, Any]:
        return call_daemon("disconnect")

    def get_robot_state(self, include_pose: bool = False) -> dict[str, Any]:
        return call_daemon("get_state", {"include_pose": include_pose})

    def move_j(
        self,
        positions: list[float],
        duration: float = 3.0,
        wait: bool = False,
        tolerance: float = 0.05,
        timeout: float | None = None,
    ) -> dict[str, Any]:
        return call_daemon("move_j", {
            "positions": positions,
            "duration": duration,
            "wait": wait,
            "tolerance": tolerance,
            "timeout": timeout or max(15.0, duration + 5.0),
        })

    def move_l(
        self,
        position_xyz_m: list[float],
        rotation_rpy_deg: list[float] | None = None,
        duration: float | None = None,
        use_spline: bool = True,
    ) -> dict[str, Any]:
        return call_daemon("move_l", {
            "position_xyz_m": position_xyz_m,
            "rotation_rpy_deg": rotation_rpy_deg,
            "duration": duration,
            "use_spline": use_spline,
        })

    def set_gripper(
        self,
        position: float,
        velocity: float = 0.5,
        max_torque: float = 0.5,
        wait: bool = False,
        tolerance: float = 0.05,
        timeout: float = 5.0,
    ) -> dict[str, Any]:
        return call_daemon("set_gripper", {
            "position": position,
            "velocity": velocity,
            "max_torque": max_torque,
            "wait": wait,
            "tolerance": tolerance,
            "timeout": timeout,
        })

    def open_gripper(
        self,
        wait: bool = False,
        tolerance: float = 0.05,
        timeout: float = 5.0,
    ) -> dict[str, Any]:
        return call_daemon("open_gripper", {"wait": wait, "tolerance": tolerance, "timeout": timeout})

    def close_gripper(
        self,
        wait: bool = False,
        tolerance: float = 0.05,
        timeout: float = 5.0,
    ) -> dict[str, Any]:
        return call_daemon("close_gripper", {"wait": wait, "tolerance": tolerance, "timeout": timeout})

    def stop_robot(self) -> dict[str, Any]:
        return call_daemon("stop")

    def start_gravity_compensation(self) -> dict[str, Any]:
        return call_daemon("gravity_compensation_start")

    def stop_gravity_compensation(self) -> dict[str, Any]:
        return call_daemon("gravity_compensation_stop")

    def get_gravity_compensation_state(self) -> dict[str, Any]:
        return call_daemon("gravity_compensation_state")

    def go_home(self, duration: float = 3.0, wait: bool = False) -> dict[str, Any]:
        return call_daemon("go_home", {"duration": duration, "wait": wait})

    def go_zero(self, duration: float = 3.0, wait: bool = False) -> dict[str, Any]:
        return call_daemon("go_zero", {"duration": duration, "wait": wait})

    # 轨迹录制相关
    def start_trajectory_recording(self) -> dict[str, Any]:
        return call_daemon("trajectory_start_recording")

    def stop_trajectory_recording(self) -> dict[str, Any]:
        return call_daemon("trajectory_stop_recording")

    def save_trajectory(self, name: str) -> dict[str, Any]:
        return call_daemon("trajectory_save", {"name": name})

    def list_trajectories(self) -> dict[str, Any]:
        return call_daemon("trajectory_list")

    def play_trajectory(self, name: str) -> dict[str, Any]:
        return call_daemon("trajectory_play", {"name": name})

    def delete_trajectory(self, name: str) -> dict[str, Any]:
        return call_daemon("trajectory_delete", {"name": name})

    def get_trajectory_state(self) -> dict[str, Any]:
        return call_daemon("trajectory_state")

    def init_camera(self) -> dict[str, Any]:
        return call_daemon("camera_init")

    def get_camera_image(self) -> dict[str, Any]:
        return call_daemon("camera_get_image")
