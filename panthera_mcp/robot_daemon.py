"""
Panthera robot daemon process.

Runs as a persistent background process, maintains the robot connection,
and serves requests via a Unix domain socket. The MCP server (which gets
killed and restarted by codex on every turn) connects to this daemon
instead of owning the robot directly.

Usage:
    python -m panthera_mcp.robot_daemon          # start daemon
    python -m panthera_mcp.robot_daemon status   # check if running
    python -m panthera_mcp.robot_daemon stop     # stop daemon
"""
from __future__ import annotations

import json
import os
import signal
import socket
import sys
import threading
import time
from pathlib import Path
from typing import Any, Optional

SOCKET_PATH = Path("/tmp/panthera_robot_daemon.sock")
PID_FILE = Path("/tmp/panthera_robot_daemon.pid")
LOG_FILE = Path("/tmp/panthera_robot_daemon.log")

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = PROJECT_ROOT / "panthera_python" / "scripts"
TRAJECTORY_DIR = PROJECT_ROOT / "trajectories"
DEFAULT_CONFIG = PROJECT_ROOT / "panthera_python" / "robot_param" / "Follower.yaml"
DEFAULT_HOME_POSE = [-0.3, 1.1, 1.1, 0.8, -0.3, 0.3]
DEFAULT_GRIPPER_OPEN_POSITION = 1.6
DEFAULT_GRIPPER_CLOSE_POSITION = 0.0

# 确保轨迹目录存在
TRAJECTORY_DIR.mkdir(exist_ok=True)


def _log(msg: str) -> None:
    line = f"[daemon] {msg}\n"
    try:
        with open(LOG_FILE, "a") as f:
            f.write(line)
    except Exception:
        pass


class RobotDaemon:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._robot = None
        self._config_path = str(DEFAULT_CONFIG)
        self._running = False
        # 重力补偿相关
        self._gravity_mode = False
        self._gravity_thread = None
        self._gravity_stop_event = threading.Event()
        # 轨迹录制相关
        self._recording = False
        self._record_thread: Optional[threading.Thread] = None
        self._record_stop_event = threading.Event()
        self._record_data: list[dict] = []
        self._record_t0: Optional[float] = None
        # 轨迹命名映射
        self._trajectory_names: dict[str, str] = {}  # name -> filepath
        # 位置保持相关
        self._hold_position_mode = False
        self._hold_position_thread = None
        self._hold_position_stop_event = threading.Event()
        # 相机相关
        self._camera = None
        self._camera_pipeline = None
        # 初始化时加载轨迹列表
        self._load_trajectory_names()

    # ------------------------------------------------------------------ robot ops

    def _connect(self, config_path: str | None) -> dict[str, Any]:
        with self._lock:
            self._disconnect_inner()
            resolved = config_path or str(DEFAULT_CONFIG)
            # Remove .local paths to avoid picking up broken pinocchio builds
            sys.path = [p for p in sys.path if ".local" not in p]
            if str(SCRIPTS_DIR) not in sys.path:
                sys.path.insert(0, str(SCRIPTS_DIR))
            # Redirect SDK stdout prints to stderr so they don't pollute anything
            stdout_fd = sys.stdout.fileno()
            stderr_fd = sys.stderr.fileno()
            saved = os.dup(stdout_fd)
            try:
                sys.stdout.flush()
                os.dup2(stderr_fd, stdout_fd)
                from Panthera_lib import Panthera  # noqa: PLC0415
                self._robot = Panthera(resolved)
            finally:
                sys.stdout.flush()
                os.dup2(saved, stdout_fd)
                os.close(saved)
            self._config_path = resolved
            _log(f"connected: {resolved}")
            return {"ok": True, "message": "robot connected",
                    "state": self._get_state(include_pose=False)}

    def _disconnect_inner(self) -> None:
        if self._robot is not None:
            try:
                self._robot.set_stop()
            except Exception:
                pass
            self._robot = None

    def _disconnect(self) -> dict[str, Any]:
        with self._lock:
            self._disconnect_inner()
            _log("disconnected")
            return {"ok": True, "message": "robot disconnected",
                    "connected": False, "config_path": self._config_path}

    def _get_state(self, include_pose: bool = False) -> dict[str, Any]:
        # 在重力补偿模式下，不加锁，直接读取，避免阻塞重力补偿线程
        if self._gravity_mode:
            if self._robot is None:
                return {"connected": False, "status": "disconnected",
                        "config_path": self._config_path}
            r = self._robot
            # 重力补偿模式下，不发送任何命令，不调用forward_kinematics
            state: dict[str, Any] = {
                "connected": True,
                "status": "online",
                "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "config_path": self._config_path,
                "motor_count": int(r.motor_count),
                "gripper_id": int(r.gripper_id),
                "joint_pos_rad": r.get_current_pos().astype(float).tolist(),
                "joint_vel_rad_s": r.get_current_vel().astype(float).tolist(),
                "joint_torque_nm": r.get_current_torque().astype(float).tolist(),
                "gripper_pos": float(r.get_current_pos_gripper()),
                "gripper_vel": float(r.get_current_vel_gripper()),
                "gripper_torque": float(r.get_current_torque_gripper()),
                "joint_limits_lower_rad": r.joint_limits["lower"].astype(float).tolist(),
                "joint_limits_upper_rad": r.joint_limits["upper"].astype(float).tolist(),
                "gripper_limits": {
                    "lower": float(r.gripper_limits["lower"]),
                    "upper": float(r.gripper_limits["upper"]),
                },
                "gravity_compensation_active": True,
            }
            # 重力补偿模式下不计算正运动学，避免干扰
            return state

        # 非重力补偿模式，正常加锁处理
        with self._lock:
            if self._robot is None:
                return {"connected": False, "status": "disconnected",
                        "config_path": self._config_path}
            r = self._robot
            r.send_get_motor_state_cmd()
            state: dict[str, Any] = {
                "connected": True,
                "status": "online",
                "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "config_path": self._config_path,
                "motor_count": int(r.motor_count),
                "gripper_id": int(r.gripper_id),
                "joint_pos_rad": r.get_current_pos().astype(float).tolist(),
                "joint_vel_rad_s": r.get_current_vel().astype(float).tolist(),
                "joint_torque_nm": r.get_current_torque().astype(float).tolist(),
                "gripper_pos": float(r.get_current_pos_gripper()),
                "gripper_vel": float(r.get_current_vel_gripper()),
                "gripper_torque": float(r.get_current_torque_gripper()),
                "joint_limits_lower_rad": r.joint_limits["lower"].astype(float).tolist(),
                "joint_limits_upper_rad": r.joint_limits["upper"].astype(float).tolist(),
                "gripper_limits": {
                    "lower": float(r.gripper_limits["lower"]),
                    "upper": float(r.gripper_limits["upper"]),
                },
                "gravity_compensation_active": False,
            }
            if include_pose:
                fk = r.forward_kinematics()
                if fk is not None:
                    state["tool_pose"] = {
                        "position_xyz_m": [float(x) for x in fk["position"]],
                        "rotation_matrix": [[float(x) for x in row] for row in fk["rotation"]],
                    }
            return state

    def _move_j(self, positions, duration, wait, tolerance, timeout) -> dict[str, Any]:
        self._stop_hold_position()  # 停止位置保持
        with self._lock:
            r = self._require_robot()
            ok = r.moveJ(positions, duration, max_tqu=r.max_torque,
                         iswait=False, tolerance=tolerance, timeout=timeout)
            if ok is False:
                raise RuntimeError("move_j rejected by SDK safety checks")
        if wait:
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                with self._lock:
                    r.send_get_motor_state_cmd()
                    cur = r.get_current_pos().astype(float).tolist()
                errs = [abs(c - t) for c, t in zip(cur, positions)]
                if all(e <= tolerance for e in errs):
                    self._start_hold_position()  # 启动位置保持
                    return {"ok": True, "message": "move_j reached",
                            "reached": True, "final_positions_rad": cur,
                            "position_error_rad": errs}
                time.sleep(0.02)
        self._start_hold_position()  # 启动位置保持
        return {"ok": True, "message": "move_j accepted", "wait": wait}

    def _set_gripper(self, position, velocity, max_torque, wait, tolerance, timeout) -> dict[str, Any]:
        with self._lock:
            r = self._require_robot()
            ok = r.gripper_control(position, velocity, max_torque)
            if ok is False:
                raise RuntimeError("gripper target rejected")
        if wait:
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                with self._lock:
                    r.send_get_motor_state_cmd()
                    cur = float(r.get_current_pos_gripper())
                if abs(cur - position) <= tolerance:
                    return {"ok": True, "message": "gripper reached", "reached": True}
                time.sleep(0.02)
        return {"ok": True, "message": "gripper target sent"}

    def _move_l(self, position_xyz_m, rotation_rpy_deg, duration, use_spline) -> dict[str, Any]:
        with self._lock:
            self._stop_hold_position()  # 停止位置保持
            r = self._require_robot()
            rotation_matrix = None
            if rotation_rpy_deg is not None:
                from scipy.spatial.transform import Rotation
                rotation_matrix = Rotation.from_euler(
                    "xyz", rotation_rpy_deg, degrees=True
                ).as_matrix()
            ok = r.moveL(
                position_xyz_m,
                target_rotation=rotation_matrix,
                duration=duration,
                use_spline=bool(use_spline),
                max_tqu=r.max_torque,
            )
        if ok is False:
            raise RuntimeError("move_l failed")
        self._start_hold_position()  # 启动位置保持
        return {
            "ok": True, "message": "move_l completed",
            "target_position_xyz_m": position_xyz_m,
            "rotation_rpy_deg": rotation_rpy_deg,
        }

    def _stop(self) -> dict[str, Any]:
        with self._lock:
            self._require_robot().set_stop()
        return {"ok": True, "message": "stop sent"}

    def _start_gravity_compensation(self) -> dict[str, Any]:
        """启动重力补偿模式"""
        with self._lock:
            if self._gravity_mode:
                return {"ok": True, "message": "gravity compensation already running"}
            self._stop_hold_position()  # 停止位置保持
            self._gravity_mode = True
            self._gravity_stop_event.clear()
            self._gravity_thread = threading.Thread(target=self._gravity_loop, daemon=True)
            self._gravity_thread.start()
        return {"ok": True, "message": "gravity compensation started"}

    def _stop_gravity_compensation(self) -> dict[str, Any]:
        """停止重力补偿模式"""
        with self._lock:
            if not self._gravity_mode:
                return {"ok": True, "message": "gravity compensation not running"}
            self._gravity_mode = False
            self._gravity_stop_event.set()
            if self._gravity_thread:
                self._gravity_thread.join(timeout=2.0)
        self._start_hold_position()  # 恢复位置保持
        return {"ok": True, "message": "gravity compensation stopped"}

    def _gravity_loop(self) -> None:
        """重力补偿后台循环"""
        r = self._robot
        tau_limit = [15.0, 30.0, 30.0, 15.0, 5.0, 5.0]
        zero_kp = [0.0] * r.motor_count
        zero_kd = [0.0] * r.motor_count

        while not self._gravity_stop_event.is_set():
            try:
                tor = r.get_Gravity()
                tor = [max(-tau_limit[i], min(tau_limit[i], tor[i])) for i in range(len(tor))]
                r.pos_vel_tqe_kp_kd([0.0] * r.motor_count, [0.0] * r.motor_count, tor, zero_kp, zero_kd)
                r.gripper_control_MIT(0, 0, 0, 0, 0)
                self._gravity_stop_event.wait(0.002)  # ~500Hz
            except Exception:
                break

    def _start_hold_position(self) -> None:
        """启动位置保持模式"""
        if self._hold_position_mode:
            return
        self._hold_position_mode = True
        self._hold_position_stop_event.clear()
        self._hold_position_thread = threading.Thread(target=self._hold_position_loop, daemon=True)
        self._hold_position_thread.start()

    def _stop_hold_position(self) -> None:
        """停止位置保持模式"""
        if not self._hold_position_mode:
            return
        self._hold_position_mode = False
        self._hold_position_stop_event.set()
        if self._hold_position_thread:
            self._hold_position_thread.join(timeout=1.0)

    def _hold_position_loop(self) -> None:
        """位置保持循环：用MIT模式保持当前位置"""
        r = self._robot
        n = r.motor_count
        # 记录要保持的位置
        hold_pos = r.get_current_pos().tolist()
        kp = [80.0, 80.0, 80.0, 40.0, 20.0, 20.0][:n]
        kd = [2.0, 2.0, 2.0, 1.0, 0.5, 0.5][:n]
        zero_vel = [0.0] * n
        zero_tqe = [0.0] * n
        while not self._hold_position_stop_event.is_set():
            try:
                r.pos_vel_tqe_kp_kd(hold_pos, zero_vel, zero_tqe, kp, kd)
                self._hold_position_stop_event.wait(0.002)  # 500Hz
            except Exception:
                break

    def _get_gravity_compensation_state(self) -> dict[str, Any]:
        """获取重力补偿模式状态"""
        return {
            "gravity_compensation_enabled": self._gravity_mode
        }

    # ------------------------------------------------------------------ trajectory recording

    def _start_recording(self) -> dict[str, Any]:
        """开始录制轨迹"""
        with self._lock:
            if self._recording:
                return {"ok": False, "error": "already recording"}
            if self._robot is None:
                return {"ok": False, "error": "robot not connected"}

            self._recording = True
            self._record_data = []
            self._record_t0 = None
            self._record_stop_event.clear()
            self._record_thread = threading.Thread(target=self._record_loop, daemon=True)
            self._record_thread.start()
            return {"ok": True, "message": "recording started"}

    def _stop_recording(self) -> dict[str, Any]:
        """停止录制轨迹"""
        with self._lock:
            if not self._recording:
                return {"ok": False, "error": "not recording"}
            self._recording = False
            self._record_stop_event.set()
            if self._record_thread:
                self._record_thread.join(timeout=2.0)

        frame_count = len(self._record_data)
        return {
            "ok": True,
            "message": f"recording stopped, {frame_count} frames captured",
            "frame_count": frame_count,
        }

    def _record_loop(self) -> None:
        """轨迹录制循环"""
        r = self._robot
        while not self._record_stop_event.is_set():
            try:
                t = time.time()
                if self._record_t0 is None:
                    self._record_t0 = t
                frame = {
                    "t": t - self._record_t0,
                    "pos": list(r.get_current_pos()),
                    "vel": list(r.get_current_vel()),
                    "gripper_pos": r.get_current_pos_gripper(),
                    "gripper_vel": r.get_current_vel_gripper(),
                }
                self._record_data.append(frame)
                self._record_stop_event.wait(0.001)  # ~1000Hz
            except Exception:
                break

    def _save_trajectory(self, name: str) -> dict[str, Any]:
        """保存录制的轨迹"""
        with self._lock:
            if not self._record_data:
                return {"ok": False, "error": "no recorded data to save"}
            if not name:
                return {"ok": False, "error": "name cannot be empty"}

            # 清理名称
            safe_name = "".join(c for c in name if c.isalnum() or c in "-_").strip()
            if not safe_name:
                return {"ok": False, "error": "invalid name"}

            frame_count = len(self._record_data)
            filepath = TRAJECTORY_DIR / f"{safe_name}.jsonl"
            with open(filepath, "w", encoding="utf-8") as f:
                for frame in self._record_data:
                    f.write(json.dumps(frame, ensure_ascii=False) + "\n")

            self._trajectory_names[safe_name] = str(filepath)
            self._record_data = []  # 清空已保存的数据

            return {
                "ok": True,
                "message": f"trajectory '{safe_name}' saved",
                "name": safe_name,
                "filepath": str(filepath),
                "frame_count": frame_count,
            }

    def _load_trajectory_names(self) -> dict[str, Any]:
        """加载所有已保存的轨迹名称"""
        self._trajectory_names.clear()
        for f in TRAJECTORY_DIR.glob("*.jsonl"):
            name = f.stem
            self._trajectory_names[name] = str(f)
        return {
            "ok": True,
            "trajectories": list(self._trajectory_names.keys()),
        }

    def _play_trajectory(self, name: str) -> dict[str, Any]:
        """播放指定名称的轨迹"""
        with self._lock:
            r = self._require_robot()

            if name not in self._trajectory_names:
                return {"ok": False, "error": f"trajectory '{name}' not found"}

            filepath = self._trajectory_names[name]
            return self._play_trajectory_file(r, filepath, name)

    def _play_trajectory_file(self, r, filepath: str, name: str = "") -> dict[str, Any]:
        """播放轨迹文件"""
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                frames = [json.loads(line) for line in f if line.strip()]
        except Exception as e:
            return {"ok": False, "error": f"failed to load trajectory: {e}"}

        if not frames:
            return {"ok": False, "error": "trajectory file is empty"}

        # 检查是否有夹爪数据
        has_gripper = "gripper_pos" in frames[0]
        frame_count = len(frames)
        total_duration = frames[-1]["t"] if frames else 0

        # 获取夹爪限位
        gripper_lower = 0.0
        gripper_upper = 2.0
        if has_gripper and r.gripper_limits is not None:
            gripper_lower = r.gripper_limits['lower']
            gripper_upper = r.gripper_limits['upper']

        max_torque = [21.0, 36.0, 36.0, 21.0, 10.0, 10.0]

        # 移动到起始点（和 SDK 完全一样）
        first_frame = frames[0]
        start_pos = first_frame["pos"]

        # 夹爪移动到起点（SDK 有这步）
        if has_gripper:
            gripper_start = first_frame["gripper_pos"]
            # 限位修正
            gripper_start_clamped = max(gripper_lower, min(gripper_upper, gripper_start))
            r.gripper_control(gripper_start_clamped, 0.5, 0.5)
            time.sleep(1.5)

        r.Joint_Pos_Vel(start_pos, [0.5] * len(start_pos), max_torque, iswait=True, tolerance=0.05, timeout=30.0)
        time.sleep(0.5)

        # 开始回放
        t0 = time.time()
        gripper_frames_sent = 0

        for i, frame in enumerate(frames):
            # 检查总超时（最多播放 60 秒）
            elapsed = time.time() - t0
            if elapsed > 60:
                break

            # 等待到这一帧的时间
            target_t = frame["t"]
            while time.time() - t0 < target_t:
                time.sleep(0.001)

            # 发送关节位置控制
            pos = frame["pos"]
            vel = frame.get("vel", [0.0] * 6)
            r.Joint_Pos_Vel(pos, vel, max_torque)

            # 夹爪控制（每帧都发送）
            if has_gripper and "gripper_pos" in frame:
                gpos = frame["gripper_pos"]
                gvel = frame.get("gripper_vel", 0.0)

                # 限位修正后直接发送
                clamped_pos = max(gripper_lower, min(gripper_upper, gpos))
                # 使用 gripper_control 而不是 gripper_control_MIT，避免内部限位检查
                r.gripper_control(clamped_pos, 0.5, 0.5)
                gripper_frames_sent += 1

        print(f"[playback] Completed: {frame_count} frames, gripper commands sent: {gripper_frames_sent}", flush=True)
        return {
            "ok": True,
            "message": f"trajectory '{name}' playback completed",
            "frame_count": frame_count,
            "gripper_commands_sent": gripper_frames_sent,
            "has_gripper": has_gripper,
            "first_gripper_pos": frames[0].get("gripper_pos") if has_gripper else None,
            "last_gripper_pos": frames[-1].get("gripper_pos") if has_gripper else None,
        }

    def _delete_trajectory(self, name: str) -> dict[str, Any]:
        """删除指定名称的轨迹"""
        if name not in self._trajectory_names:
            return {"ok": False, "error": f"trajectory '{name}' not found"}

        filepath = self._trajectory_names.pop(name)
        try:
            os.remove(filepath)
        except Exception:
            pass

        return {"ok": True, "message": f"trajectory '{name}' deleted"}

    def _get_recording_state(self) -> dict[str, Any]:
        """获取录制状态"""
        return {
            "recording": self._recording,
            "frame_count": len(self._record_data),
        }

    def _require_robot(self):
        if self._robot is None:
            raise RuntimeError("robot is not connected")
        return self._robot

    # ------------------------------------------------------------------ camera ops

    def _init_camera(self) -> dict[str, Any]:
        """初始化D405相机"""
        if self._camera_pipeline is not None:
            return {"ok": True, "message": "camera already initialized"}
        try:
            import pyrealsense2 as rs
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            pipeline.start(config)
            self._camera_pipeline = pipeline
            self._camera = rs.align(rs.stream.color)
            return {"ok": True, "message": "camera initialized"}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def _get_camera_image(self) -> dict[str, Any]:
        """获取RGB图像"""
        if self._camera_pipeline is None:
            return {"ok": False, "error": "camera not initialized"}
        try:
            import numpy as np
            import base64
            import cv2
            frames = self._camera_pipeline.wait_for_frames(timeout_ms=1000)
            aligned_frames = self._camera.process(frames)
            color_frame = aligned_frames.get_color_frame()
            if not color_frame:
                return {"ok": False, "error": "no color frame"}
            image = np.asanyarray(color_frame.get_data())
            _, buffer = cv2.imencode('.jpg', image)
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            return {"ok": True, "image": image_base64, "width": 640, "height": 480}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    # ------------------------------------------------------------------ dispatch

    def _dispatch(self, req: dict[str, Any]) -> dict[str, Any]:
        method = req.get("method")
        params = req.get("params", {}) or {}
        try:
            if method == "connect":
                return self._connect(params.get("config_path"))
            elif method == "disconnect":
                return self._disconnect()
            elif method == "get_state":
                return self._get_state(include_pose=bool(params.get("include_pose", False)))
            elif method == "move_j":
                pos = params["positions"]
                dur = float(params.get("duration", 3.0))
                wait = bool(params.get("wait", False))
                tol = float(params.get("tolerance", 0.05))
                to = float(params.get("timeout", max(15.0, dur + 5.0)))
                return self._move_j(pos, dur, wait, tol, to)
            elif method == "go_home":
                dur = float(params.get("duration", 3.0))
                wait = bool(params.get("wait", False))
                return self._move_j(DEFAULT_HOME_POSE, dur, wait, 0.05, max(15.0, dur + 5.0))
            elif method == "go_zero":
                with self._lock:
                    r = self._require_robot()
                    n = int(r.motor_count)
                dur = float(params.get("duration", 3.0))
                wait = bool(params.get("wait", False))
                return self._move_j([0.0] * n, dur, wait, 0.05, max(15.0, dur + 5.0))
            elif method == "move_l":
                return self._move_l(
                    params["position_xyz_m"],
                    params.get("rotation_rpy_deg"),
                    params.get("duration"),
                    bool(params.get("use_spline", True)),
                )
            elif method == "set_gripper":
                return self._set_gripper(
                    float(params["position"]),
                    float(params.get("velocity", 0.5)),
                    float(params.get("max_torque", 0.5)),
                    bool(params.get("wait", False)),
                    float(params.get("tolerance", 0.05)),
                    float(params.get("timeout", 5.0)),
                )
            elif method == "open_gripper":
                return self._set_gripper(
                    DEFAULT_GRIPPER_OPEN_POSITION, 0.5, 0.5,
                    bool(params.get("wait", False)),
                    float(params.get("tolerance", 0.05)),
                    float(params.get("timeout", 5.0)),
                )
            elif method == "close_gripper":
                return self._set_gripper(
                    DEFAULT_GRIPPER_CLOSE_POSITION, 0.5, 0.5,
                    bool(params.get("wait", False)),
                    float(params.get("tolerance", 0.05)),
                    float(params.get("timeout", 5.0)),
                )
            elif method == "stop":
                return self._stop()
            elif method == "gravity_compensation_start":
                return self._start_gravity_compensation()
            elif method == "gravity_compensation_stop":
                return self._stop_gravity_compensation()
            elif method == "gravity_compensation_state":
                return self._get_gravity_compensation_state()
            elif method == "trajectory_start_recording":
                return self._start_recording()
            elif method == "trajectory_stop_recording":
                return self._stop_recording()
            elif method == "trajectory_save":
                return self._save_trajectory(params.get("name", ""))
            elif method == "trajectory_list":
                return self._load_trajectory_names()
            elif method == "trajectory_play":
                return self._play_trajectory(params.get("name", ""))
            elif method == "trajectory_delete":
                return self._delete_trajectory(params.get("name", ""))
            elif method == "trajectory_state":
                return self._get_recording_state()
            elif method == "camera_init":
                return self._init_camera()
            elif method == "camera_get_image":
                return self._get_camera_image()
            elif method == "ping":
                return {"ok": True, "message": "pong"}
            else:
                return {"ok": False, "error": f"unknown method: {method}"}
        except Exception as exc:
            _log(f"dispatch error [{method}]: {exc}")
            return {"ok": False, "error": str(exc)}

    # ------------------------------------------------------------------ server

    def _handle_client(self, conn: socket.socket) -> None:
        try:
            data = b""
            while True:
                chunk = conn.recv(65536)
                if not chunk:
                    break
                data += chunk
                if b"\n" in data:
                    break
            if not data.strip():
                return
            req = json.loads(data.strip())
            resp = self._dispatch(req)
            conn.sendall((json.dumps(resp) + "\n").encode())
        except Exception as exc:
            _log(f"client handler error: {exc}")
        finally:
            conn.close()

    def serve(self) -> None:
        SOCKET_PATH.unlink(missing_ok=True)
        PID_FILE.write_text(str(os.getpid()))
        _log(f"daemon started pid={os.getpid()}")

        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server.bind(str(SOCKET_PATH))
        server.listen(8)
        server.settimeout(1.0)

        self._running = True

        def _shutdown(sig, frame):
            self._running = False
            _log("shutdown signal received")

        signal.signal(signal.SIGTERM, _shutdown)
        signal.signal(signal.SIGINT, _shutdown)

        while self._running:
            try:
                conn, _ = server.accept()
                threading.Thread(target=self._handle_client, args=(conn,), daemon=True).start()
            except socket.timeout:
                continue
            except Exception as exc:
                _log(f"accept error: {exc}")
                break

        self._disconnect_inner()
        server.close()
        SOCKET_PATH.unlink(missing_ok=True)
        PID_FILE.unlink(missing_ok=True)
        _log("daemon stopped")


# ------------------------------------------------------------------ CLI helpers

def _is_running() -> bool:
    if not PID_FILE.exists():
        return False
    try:
        pid = int(PID_FILE.read_text().strip())
        os.kill(pid, 0)
        return True
    except (ProcessLookupError, ValueError):
        return False


def start_daemon_if_needed() -> None:
    """Called by the MCP server to ensure the daemon is running."""
    if _is_running():
        return
    _log("starting daemon subprocess")
    import subprocess
    python = sys.executable
    env = os.environ.copy()
    env.pop("PYTHONPATH", None)
    proc = subprocess.Popen(
        [python, "-m", "panthera_mcp.robot_daemon"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
        env=env,
    )
    # Wait up to 3 seconds for socket to appear
    for _ in range(30):
        if SOCKET_PATH.exists():
            return
        time.sleep(0.1)
    _log(f"daemon pid={proc.pid} socket not ready in time")


def call_daemon(method: str, params: dict | None = None, timeout: float = 15.0) -> dict[str, Any]:
    """Send one request to the daemon and return the response."""
    start_daemon_if_needed()
    req = json.dumps({"method": method, "params": params or {}}) + "\n"
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        sock.connect(str(SOCKET_PATH))
        sock.sendall(req.encode())
        data = b""
        while True:
            chunk = sock.recv(65536)
            if not chunk:
                break
            data += chunk
            if b"\n" in data:
                break
        return json.loads(data.strip())
    finally:
        sock.close()


if __name__ == "__main__":
    if len(sys.argv) > 1:
        cmd = sys.argv[1]
        if cmd == "status":
            if _is_running():
                print("daemon is running")
                resp = call_daemon("ping")
                print(f"ping: {resp}")
            else:
                print("daemon is NOT running")
        elif cmd == "stop":
            if PID_FILE.exists():
                pid = int(PID_FILE.read_text().strip())
                os.kill(pid, signal.SIGTERM)
                print(f"sent SIGTERM to {pid}")
            else:
                print("daemon not running")
        sys.exit(0)

    RobotDaemon().serve()
