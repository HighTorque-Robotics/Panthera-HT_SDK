#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""使用 7 个飞特舵机组成的 GELLO 遥操 Panthera 从臂。

控制关系：
  - GELLO J1~J6：飞特 ID 1, 2, 3, 4, 5, 6
  - GELLO 夹爪：飞特 ID 7
  - 飞特读取：PacketHandler(0) + 地址 56 + read2ByteTxRx()
  - Panthera：使用 robot_param/Follower.yaml

启动时同时捕获 GELLO 当前零位和 Panthera 从臂当前位置，因此开始遥操时
不会产生标定跳变。Panthera 从臂初始化后会先以低速回到六关节零位，
然后才允许 GELLO 标定并进入 MIT 跟随。程序不初始化 Panthera 主臂。

运行示例：
    python 5_gello_follower_teleop.py --gello-port /dev/ttyACM1

先只验证 GELLO 映射、不连接 Panthera：
    python 5_gello_follower_teleop.py --gello-port /dev/ttyACM1 --dry-run
"""

import argparse
import math
import os
import select
import signal
import sys
import threading
import time
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
DEFAULT_FOLLOWER_CONFIG = os.path.join(PROJECT_DIR, "robot_param", "Follower.yaml")

FEETECH_BAUD = 1_000_000
PRESENT_POSITION_ADDR = 56
TICKS_PER_REV = 4096
RAD_PER_TICK = 2.0 * math.pi / TICKS_PER_REV

# 顺序对应 Panthera 从臂 J1~J6，最后一个为夹爪。
# 固定使用正常顺序，不开放命令行修改：J1~J6 -> ID 1~6，夹爪 -> ID 7。
SERVO_IDS = [1, 2, 3, 4, 5, 6, 7]

# GELLO J1~J6 与 Panthera 从臂 J1~J6 默认反向。
DEFAULT_JOINT_SIGNS = [-1.0] * 6
DEFAULT_JOINT_SCALES = [1.0] * 6

# 比原主从示例更均衡的 MIT 跟随增益，仍低于仓库轨迹控制使用的高增益。
DEFAULT_KP = [20.0, 40.0, 50.0, 30.0, 15.0, 10.0]
DEFAULT_KD = [2.0, 2.5, 2.5, 2.0, 1.5, 1.0]
DEFAULT_GRIPPER_KP = 5.0
DEFAULT_GRIPPER_KD = 0.2

FEETECH_TICK_JUMP_THRESHOLD = 800
DEFAULT_CONTROL_RATE = 100.0
DEFAULT_GELLO_TIMEOUT = 0.5

# Panthera 从臂启动复位参数。
FOLLOWER_HOME_POSITION = np.zeros(6, dtype=float)
FOLLOWER_HOME_SPEED = 0.3
FOLLOWER_HOME_TOLERANCE = 0.05
FOLLOWER_HOME_TIMEOUT = 30.0
FOLLOWER_GRIPPER_CLOSED_POSITION = 0.0
FOLLOWER_GRIPPER_HOME_SPEED = 0.3
FOLLOWER_GRIPPER_HOME_MAX_TORQUE = 0.3
FOLLOWER_GRIPPER_HOME_TOLERANCE = 0.10

stop_event = threading.Event()


def handle_termination_signal(signum, _frame):
    print("\n收到信号 {}，正在安全退出...".format(signum), flush=True)
    stop_event.set()


def install_signal_handlers():
    # SIGINT 保留 Python 默认行为，使 input()/select() 也能被 Ctrl+C 中断。
    signal.signal(signal.SIGTERM, handle_termination_signal)


def detect_default_gello_port() -> str:
    """按现有 GELLO 程序使用的优先级选择串口。"""
    candidates = [
        "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14028931-if00",
        "/dev/ttyACM1",
        "/dev/feetech_servos",
        "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
        "/dev/ttyUSB0",
    ]
    for path in candidates:
        if os.path.exists(path):
            return path
    return "/dev/ttyACM1"


def import_feetech_sdk():
    try:
        from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler
    except (ImportError, OSError) as error:
        raise RuntimeError(
            "缺少或装错飞特 SDK，请安装：python -m pip install "
            "feetech-servo-sdk"
        ) from error
    return COMM_SUCCESS, PacketHandler, PortHandler


def import_panthera_class():
    try:
        from Panthera_lib import Panthera
    except (ImportError, OSError) as error:
        raise RuntimeError(
            "无法导入 Panthera_lib，请在 panthera Conda 环境的 scripts 目录运行"
        ) from error
    return Panthera


class ServoController:
    """与现有 GELLO 程序相同的飞特位置读取接口。"""

    def __init__(self, servo_ids: Sequence[int], port: str, baudrate: int, sdk):
        comm_success, packet_handler_factory, port_handler_class = sdk
        self.servo_ids = list(servo_ids)
        self.port = port_handler_class(port)
        self.packet_handler = packet_handler_factory(0)
        self.comm_success = comm_success
        self.baudrate = int(baudrate)

    def connect(self):
        if not self.port.openPort():
            raise RuntimeError(
                "无法打开 GELLO 串口：{}".format(self.port.getPortName())
            )
        if not self.port.setBaudRate(self.baudrate):
            self.port.closePort()
            raise RuntimeError("无法设置 GELLO 波特率：{}".format(self.baudrate))

    def disconnect(self):
        if self.port is not None and getattr(self.port, "is_open", False):
            self.port.closePort()

    def read_positions(self) -> Dict[int, int]:
        ticks: Dict[int, int] = {}
        for sid in self.servo_ids:
            position, result, error = self.packet_handler.read2ByteTxRx(
                self.port,
                int(sid),
                PRESENT_POSITION_ADDR,
            )
            if result != self.comm_success:
                message = self.packet_handler.getTxRxResult(result)
                raise RuntimeError(
                    "读取飞特 ID {} 失败：{}".format(sid, message)
                )
            if error != 0:
                message = self.packet_handler.getRxPacketError(error)
                raise RuntimeError(
                    "飞特 ID {} 返回错误：{}".format(sid, message)
                )
            ticks[int(sid)] = int(position)
        return ticks


def wrap_tick_delta(delta: int) -> int:
    """把相邻帧 tick 差值折叠到 [-2048, 2047]。"""
    return ((int(delta) + TICKS_PER_REV // 2) % TICKS_PER_REV) - (
        TICKS_PER_REV // 2
    )


class FeetechReader:
    """完整帧校验、跳变过滤和多圈展开。"""

    def __init__(self, servo_ids: Sequence[int], jump_threshold: int):
        self.servo_ids = list(servo_ids)
        self.jump_threshold = int(jump_threshold)
        self.raw_previous: Dict[int, int] = {}
        self.unwrapped: Dict[int, int] = {}
        self.last_ok_time: Optional[float] = None
        self.last_error = "尚未收到数据"
        self.last_error_print_time = 0.0

    @property
    def initialized(self) -> bool:
        return len(self.unwrapped) == len(self.servo_ids)

    def reset(self):
        """丢弃旧基准；用于从臂回零后重新接收 GELLO 当前帧。"""
        self.raw_previous.clear()
        self.unwrapped.clear()
        self.last_ok_time = None
        self.last_error = "尚未收到数据"

    def update(self, controller: ServoController) -> bool:
        try:
            ticks = controller.read_positions()
        except Exception as error:
            self.last_error = str(error)
            return False

        missing = [sid for sid in self.servo_ids if sid not in ticks]
        if missing:
            self.last_error = "飞特位置帧不完整，缺少 ID {}".format(missing)
            return False

        if not self.initialized:
            self.raw_previous = dict(ticks)
            self.unwrapped = dict(ticks)
        else:
            deltas: Dict[int, int] = {}
            for sid in self.servo_ids:
                delta = wrap_tick_delta(ticks[sid] - self.raw_previous[sid])
                if abs(delta) > self.jump_threshold:
                    self.last_error = (
                        "飞特 ID {} 位置跳变：{} -> {}，delta={} tick"
                    ).format(sid, self.raw_previous[sid], ticks[sid], delta)
                    return False
                deltas[sid] = delta

            for sid in self.servo_ids:
                self.unwrapped[sid] += deltas[sid]
                self.raw_previous[sid] = ticks[sid]

        self.last_ok_time = time.monotonic()
        self.last_error = ""
        return True

    def age(self) -> float:
        if self.last_ok_time is None:
            return float("inf")
        return time.monotonic() - self.last_ok_time

    def print_error_rate_limited(self, prefix: str = "[GELLO]"):
        now = time.monotonic()
        if now - self.last_error_print_time >= 1.0:
            print("{} {}".format(prefix, self.last_error), flush=True)
            self.last_error_print_time = now


def wait_for_first_frame(
    controller: ServoController,
    reader: FeetechReader,
    timeout: float,
    port: str,
):
    deadline = time.monotonic() + timeout
    while not stop_event.is_set() and time.monotonic() < deadline:
        if reader.update(controller):
            return
        reader.print_error_rate_limited()
        time.sleep(0.02)

    raise RuntimeError(
        "GELLO 预检失败：串口 {} 在 {:.1f}s 内没有收到 7 个舵机的完整位置。"
        "最近错误：{}。请检查端口（参考程序通常使用 /dev/ttyACM1）、"
        "飞特供电、TX/RX 接线和舵机 ID。".format(
            port, timeout, reader.last_error
        )
    )


def format_vector(values: Sequence[float], digits: int = 3) -> str:
    template = "{{: .{}f}}".format(digits)
    return "[" + ", ".join(template.format(float(v)) for v in values) + "]"


def print_gello_ticks(reader: FeetechReader, servo_ids: Sequence[int]):
    values = []
    for index, sid in enumerate(servo_ids):
        name = "J{}".format(index + 1) if index < 6 else "Grip"
        values.append("{}(ID{})={}".format(name, sid, reader.raw_previous[sid]))
    print("\r[GELLO] " + "  ".join(values) + " " * 8, end="", flush=True)


def wait_for_calibration_enter(
    controller: ServoController,
    reader: FeetechReader,
):
    print("\n" + "=" * 72)
    print("标定准备")
    print("1. Panthera 从臂六关节已经回到零位，请不要再手推从臂")
    print("2. 将 GELLO 放在与从臂零位对应的舒适姿态")
    print("3. 确认从臂周围无人、急停可用，然后按 Enter 开始 MIT 跟随")
    print("按 Enter 的瞬间会记录 GELLO 当前位置作为遥操零位。")
    print("=" * 72)

    last_display = 0.0
    while not stop_event.is_set():
        updated = reader.update(controller)
        now = time.monotonic()
        if updated and now - last_display >= 0.2:
            print_gello_ticks(reader, reader.servo_ids)
            last_display = now
        elif not updated:
            reader.print_error_rate_limited()

        readable, _, _ = select.select([sys.stdin], [], [], 0.02)
        if readable:
            sys.stdin.readline()
            if not reader.initialized or reader.age() > 0.2:
                print("\n当前 GELLO 数据无效，请恢复通信后重新按 Enter。")
                continue
            print()
            return

    raise RuntimeError("标定被终止")


def joint_target_from_gello(
    unwrapped: Dict[int, int],
    zero_ticks: Dict[int, int],
    follower_zero: np.ndarray,
    joint_servo_ids: Sequence[int],
    joint_signs: np.ndarray,
    joint_scales: np.ndarray,
) -> np.ndarray:
    delta_ticks = np.array(
        [unwrapped[sid] - zero_ticks[sid] for sid in joint_servo_ids],
        dtype=float,
    )
    return follower_zero + joint_signs * joint_scales * delta_ticks * RAD_PER_TICK


def gripper_target_from_gello(
    unwrapped_tick: int,
    zero_tick: int,
    follower_zero: float,
    lower: float,
    upper: float,
    sign: float,
    tick_span: float,
) -> float:
    range_size = upper - lower
    delta = sign * float(unwrapped_tick - zero_tick) / tick_span * range_size
    return float(np.clip(follower_zero + delta, lower, upper))


def get_gravity_torque(robot, q: np.ndarray) -> np.ndarray:
    gravity = np.asarray(robot.get_Gravity(q), dtype=float)
    max_torque = np.asarray(robot.max_torque, dtype=float)
    return np.clip(gravity, -max_torque, max_torque)


def refresh_follower_state(robot, settle_time: float = 0.0):
    """主动请求一次 Panthera 电机状态，避免诊断时读取到旧缓存。"""
    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    if settle_time > 0:
        time.sleep(settle_time)


def move_follower_to_zero(robot, home_gripper: bool = True):
    """让 Panthera 六关节低速回零，并以低力矩闭合夹爪。"""
    refresh_follower_state(robot, settle_time=0.05)
    q_before = np.asarray(robot.get_current_pos(), dtype=float)
    g_before = float(robot.get_current_pos_gripper())

    print("\n" + "=" * 72)
    print("Panthera 从臂即将执行启动复位")
    print("当前位置：{} rad".format(format_vector(q_before)))
    print("目标位置：{} rad".format(format_vector(FOLLOWER_HOME_POSITION)))
    if home_gripper:
        print(
            "夹爪：{:.3f} -> {:.3f}（闭合，速度 {:.2f}，最大力矩 {:.2f} Nm）".format(
                g_before,
                FOLLOWER_GRIPPER_CLOSED_POSITION,
                FOLLOWER_GRIPPER_HOME_SPEED,
                FOLLOWER_GRIPPER_HOME_MAX_TORQUE,
            )
        )
    else:
        print("夹爪：--no-gripper 已启用，保持当前位置")
    print("关节回零速度：{:.2f} rad/s".format(FOLLOWER_HOME_SPEED))
    print("请确认工作空间无人，必要时按 Ctrl+C 停止。")
    print("=" * 72)

    # 留出短暂反应时间，同时明确告知即将发生运动。
    for remaining in (3, 2, 1):
        if stop_event.is_set():
            raise RuntimeError("从臂回零已被终止")
        print("{} 秒后开始回零...".format(remaining), flush=True)
        time.sleep(1.0)

    if stop_event.is_set():
        raise RuntimeError("从臂回零已被终止")

    velocity = np.full(robot.motor_count, FOLLOWER_HOME_SPEED, dtype=float)
    success = robot.Joint_Pos_Vel(
        FOLLOWER_HOME_POSITION.tolist(),
        velocity.tolist(),
        np.asarray(robot.max_torque, dtype=float).tolist(),
        iswait=False,
    )
    if success is False:
        raise RuntimeError("Panthera 从臂拒绝了回零命令")

    if home_gripper:
        success = robot.gripper_control(
            FOLLOWER_GRIPPER_CLOSED_POSITION,
            FOLLOWER_GRIPPER_HOME_SPEED,
            FOLLOWER_GRIPPER_HOME_MAX_TORQUE,
        )
        if success is False:
            raise RuntimeError("Panthera 从臂拒绝了夹爪闭合命令")

    deadline = time.monotonic() + FOLLOWER_HOME_TIMEOUT
    q_after = q_before.copy()
    g_after = g_before
    joint_error = float("inf")
    gripper_error = float("inf") if home_gripper else 0.0
    while time.monotonic() < deadline:
        if stop_event.is_set():
            raise RuntimeError("从臂回零已被终止")
        refresh_follower_state(robot, settle_time=0.02)
        q_after = np.asarray(robot.get_current_pos(), dtype=float)
        g_after = float(robot.get_current_pos_gripper())
        joint_error = float(np.max(np.abs(q_after - FOLLOWER_HOME_POSITION)))
        if home_gripper:
            gripper_error = abs(g_after - FOLLOWER_GRIPPER_CLOSED_POSITION)
        if (
            joint_error <= FOLLOWER_HOME_TOLERANCE
            and gripper_error <= FOLLOWER_GRIPPER_HOME_TOLERANCE
        ):
            break

    if (
        joint_error > FOLLOWER_HOME_TOLERANCE
        or gripper_error > FOLLOWER_GRIPPER_HOME_TOLERANCE
    ):
        raise RuntimeError(
            "Panthera 从臂启动复位失败：关节位置={}，关节最大误差={:.3f} rad，"
            "夹爪位置={:.3f}，夹爪误差={:.3f}。"
            "请检查从臂是否接在 CAN 口 1、电机状态和急停。".format(
                format_vector(q_after), joint_error, g_after, gripper_error
            )
        )

    print(
        "[OK] Panthera 从臂启动复位完成：关节={} rad，夹爪={:.3f}".format(
            format_vector(q_after), g_after
        )
    )


def command_follower(
    robot,
    q_command: np.ndarray,
    kp: np.ndarray,
    kd: np.ndarray,
    gripper_command: Optional[float],
    gripper_kp: float,
    gripper_kd: float,
):
    # MIT 位置跟随：目标位置来自 GELLO，目标速度固定为 0，
    # 力矩项只使用目标位置处的重力补偿。
    zero_velocity = np.zeros_like(q_command)
    gravity = get_gravity_torque(robot, q_command)
    success = robot.pos_vel_tqe_kp_kd(
        q_command.tolist(),
        zero_velocity.tolist(),
        gravity.tolist(),
        kp.tolist(),
        kd.tolist(),
    )
    if success is False:
        raise RuntimeError("Panthera 从臂拒绝了关节控制命令")

    if gripper_command is not None:
        success = robot.gripper_control_MIT(
            float(gripper_command),
            0.0,
            0.0,
            float(gripper_kp),
            float(gripper_kd),
        )
        if success is False:
            raise RuntimeError("Panthera 从臂拒绝了夹爪控制命令")


def safe_hold_current(
    robot,
    kp: np.ndarray,
    kd: np.ndarray,
    no_gripper: bool,
    gripper_kp: float,
    gripper_kd: float,
):
    """退出前发送数帧当前位置保持命令。"""
    try:
        q_hold = np.asarray(robot.get_current_pos(), dtype=float)
        g_hold = None if no_gripper else float(robot.get_current_pos_gripper())
        for _ in range(3):
            command_follower(
                robot=robot,
                q_command=q_hold,
                kp=kp,
                kd=kd,
                gripper_command=g_hold,
                gripper_kp=gripper_kp,
                gripper_kd=gripper_kd,
            )
            time.sleep(0.02)
        print("[SAFE] Panthera 从臂已发送当前位置保持命令", flush=True)
    except Exception as error:
        print("[WARN] 从臂安全保持失败：{}".format(error), flush=True)


def run_dry_mode(
    args,
    controller: ServoController,
    reader: FeetechReader,
    zero_ticks: Dict[int, int],
):
    print("[DRY-RUN] 不连接 Panthera，只打印 GELLO 相对关节量。")
    start_time = time.monotonic()
    last_print = 0.0
    while not stop_event.is_set():
        updated = reader.update(controller)
        now = time.monotonic()
        if not updated:
            reader.print_error_rate_limited()
            if reader.age() > args.gello_timeout:
                raise RuntimeError("GELLO 连续通信超时：{}".format(reader.last_error))
        elif now - last_print >= args.print_interval:
            q_delta = joint_target_from_gello(
                reader.unwrapped,
                zero_ticks,
                np.zeros(6),
                SERVO_IDS[:6],
                np.asarray(args.joint_signs),
                np.asarray(args.joint_scales),
            )
            gripper_delta = (
                args.gripper_sign
                * (reader.unwrapped[SERVO_IDS[6]] - zero_ticks[SERVO_IDS[6]])
            )
            print(
                "[DRY-RUN] q_delta(rad)={}  gripper_delta={} tick".format(
                    format_vector(q_delta), int(gripper_delta)
                )
            )
            last_print = now

        if args.duration > 0 and now - start_time >= args.duration:
            return
        time.sleep(0.002)


def run_control_loop(
    args,
    controller: ServoController,
    reader: FeetechReader,
    robot,
    zero_ticks: Dict[int, int],
    follower_zero: np.ndarray,
    gripper_zero: float,
):
    kp = np.asarray(args.kp, dtype=float)
    kd = np.asarray(args.kd, dtype=float)
    joint_signs = np.asarray(args.joint_signs, dtype=float)
    joint_scales = np.asarray(args.joint_scales, dtype=float)

    lower = np.asarray(robot.joint_limits["lower"], dtype=float)
    upper = np.asarray(robot.joint_limits["upper"], dtype=float)
    gripper_lower = float(robot.gripper_limits["lower"])
    gripper_upper = float(robot.gripper_limits["upper"])

    q_command = np.clip(follower_zero.copy(), lower, upper)
    q_raw_target = q_command.copy()
    q_target = q_command.copy()
    g_command = float(np.clip(gripper_zero, gripper_lower, gripper_upper))

    start_time = time.monotonic()
    next_cycle = start_time
    last_print = start_time
    rate_window_start = start_time
    loop_count = 0
    valid_gello_frames = 0

    print("\n[OK] GELLO 与 Panthera 从臂已对齐，开始遥操")
    print("     GELLO ID：{}（固定正常顺序）".format(SERVO_IDS))
    print("     关节方向：{}".format(args.joint_signs))
    print("     控制频率：{:.1f} Hz".format(args.control_rate))
    print("     Ctrl+C：保持从臂当前位置并退出\n")

    while not stop_event.is_set():
        loop_count += 1
        now = time.monotonic()

        updated = reader.update(controller)
        if updated:
            valid_gello_frames += 1
            q_raw_target = joint_target_from_gello(
                reader.unwrapped,
                zero_ticks,
                follower_zero,
                SERVO_IDS[:6],
                joint_signs,
                joint_scales,
            )
            q_target = np.clip(q_raw_target, lower, upper)
            # 不做低通或软件速度限制，经过硬关节限位保护后直接作为 MIT 位置命令。
            q_command = q_target.copy()

            if not args.no_gripper:
                g_command = gripper_target_from_gello(
                    reader.unwrapped[SERVO_IDS[6]],
                    zero_ticks[SERVO_IDS[6]],
                    gripper_zero,
                    gripper_lower,
                    gripper_upper,
                    args.gripper_sign,
                    args.gripper_tick_span,
                )
        else:
            reader.print_error_rate_limited()
            if reader.age() > args.gello_timeout:
                raise RuntimeError(
                    "GELLO 连续 {:.3f}s 无有效完整帧，停止遥操。最近错误：{}".format(
                        reader.age(), reader.last_error
                    )
                )

        if args.no_gripper:
            gripper_command: Optional[float] = None
        else:
            gripper_command = g_command

        command_follower(
            robot=robot,
            q_command=q_command,
            kp=kp,
            kd=kd,
            gripper_command=gripper_command,
            gripper_kp=args.gripper_kp,
            gripper_kd=args.gripper_kd,
        )

        if now - last_print >= args.print_interval:
            refresh_follower_state(robot)
            q_actual = np.asarray(robot.get_current_pos(), dtype=float)
            gello_delta = q_raw_target - follower_zero
            rate_now = time.monotonic()
            rate_elapsed = max(rate_now - rate_window_start, 1e-6)
            actual_loop_rate = loop_count / rate_elapsed
            actual_gello_rate = valid_gello_frames / rate_elapsed
            print(
                "[RATE] loop={:.1f} Hz  GELLO完整帧={:.1f} Hz  target={:.1f} Hz".format(
                    actual_loop_rate, actual_gello_rate, args.control_rate
                )
            )
            print("[GELLO] delta={}".format(format_vector(gello_delta)))
            print(
                "[FOLLOWER] target={}  cmd={}  actual={}  err={}".format(
                    format_vector(q_target),
                    format_vector(q_command),
                    format_vector(q_actual),
                    format_vector(q_command - q_actual),
                )
            )
            if np.any(np.abs(q_raw_target - q_target) > 1e-6):
                limited = np.where(np.abs(q_raw_target - q_target) > 1e-6)[0] + 1
                print(
                    "[LIMIT] 关节 {} 的 GELLO 目标已被从臂关节限位截断".format(
                        limited.tolist()
                    )
                )

            elapsed = now - start_time
            gello_motion = float(np.max(np.abs(gello_delta)))
            command_motion = float(np.max(np.abs(q_command - follower_zero)))
            actual_motion = float(np.max(np.abs(q_actual - follower_zero)))
            if elapsed >= 3.0 and gello_motion < 0.02:
                print(
                    "[DIAG] 未检测到明显 GELLO 关节变化；请在按 Enter 标定后再转动 GELLO。"
                )
            elif command_motion >= 0.08 and actual_motion < 0.02:
                print(
                    "[DIAG] GELLO 和目标命令已变化，但从臂几乎未运动；"
                    "请检查从臂是否接在 CAN 口 1、电机是否正常响应以及急停状态。"
                )
            if not args.no_gripper:
                print(
                    "[GRIPPER] cmd={:.3f}  actual={:.3f}".format(
                        g_command, float(robot.get_current_pos_gripper())
                    )
                )
            last_print = now
            rate_window_start = rate_now
            loop_count = 0
            valid_gello_frames = 0

        if args.duration > 0 and now - start_time >= args.duration:
            print("已达到指定运行时长 {:.1f}s".format(args.duration))
            return

        next_cycle += 1.0 / args.control_rate
        remaining = next_cycle - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        else:
            next_cycle = time.monotonic()


def run(args, sdk, panthera_class=None) -> int:
    port = args.gello_port or detect_default_gello_port()
    controller = ServoController(SERVO_IDS, port, args.baudrate, sdk)
    reader = FeetechReader(SERVO_IDS, args.tick_jump_threshold)
    robot = None
    control_started = False
    kp = np.asarray(args.kp, dtype=float)
    kd = np.asarray(args.kd, dtype=float)

    try:
        controller.connect()
        print(
            "[OK] GELLO 串口已打开：{} @ {}，PacketHandler(0)".format(
                port, args.baudrate
            )
        )

        # 必须先通过 GELLO 预检，避免飞特无响应时仍初始化并控制从臂。
        wait_for_first_frame(
            controller,
            reader,
            args.startup_timeout,
            port,
        )
        print("[OK] 7 个飞特舵机位置读取正常")

        if not args.dry_run:
            if not os.path.isfile(args.follower_config):
                raise RuntimeError(
                    "从臂配置文件不存在：{}".format(args.follower_config)
                )
            if panthera_class is None:
                panthera_class = import_panthera_class()
            print("正在初始化 Panthera 从臂：{}".format(args.follower_config))
            robot = panthera_class(args.follower_config)
            if int(robot.motor_count) != 6:
                raise RuntimeError(
                    "期望 6 个从臂关节，实际发现 {} 个".format(robot.motor_count)
                )
            if robot.joint_limits is None or robot.gripper_limits is None:
                raise RuntimeError("Follower.yaml 缺少关节或夹爪限位")

            # 一旦开始回零，就已经向机械臂发送了控制命令；异常退出时需要保持。
            control_started = True
            move_follower_to_zero(robot, home_gripper=not args.no_gripper)

            # 回零期间 GELLO 可能被移动，旧展开基准已无意义，重新从当前帧建基准。
            reader.reset()
            wait_for_first_frame(
                controller,
                reader,
                args.startup_timeout,
                port,
            )
            print("[OK] 回零后 GELLO 位置读取正常")

        wait_for_calibration_enter(controller, reader)
        zero_ticks = dict(reader.unwrapped)

        if args.dry_run:
            run_dry_mode(args, controller, reader, zero_ticks)
            return 0

        follower_zero = np.asarray(robot.get_current_pos(), dtype=float)
        gripper_zero = float(robot.get_current_pos_gripper())
        print("[CALIB] GELLO 零位 tick：{}".format(zero_ticks))
        print("[CALIB] 从臂起始位置：{} rad".format(format_vector(follower_zero)))
        print("[CALIB] 从臂夹爪起始位置：{:.3f}".format(gripper_zero))

        run_control_loop(
            args,
            controller,
            reader,
            robot,
            zero_ticks,
            follower_zero,
            gripper_zero,
        )
        return 0
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，停止遥操。", flush=True)
        return 0
    except Exception as error:
        print("\n错误：{}".format(error), file=sys.stderr, flush=True)
        return 1
    finally:
        # 标定前退出时从未向机械臂发送控制指令，此时也不要突然发送保持命令。
        if robot is not None and control_started:
            safe_hold_current(
                robot,
                kp,
                kd,
                args.no_gripper,
                args.gripper_kp,
                args.gripper_kd,
            )
        controller.disconnect()
        print("[OK] GELLO 串口已关闭", flush=True)


def positive_float(value: str) -> float:
    number = float(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("参数必须大于 0")
    return number


def nonnegative_float(value: str) -> float:
    number = float(value)
    if number < 0:
        raise argparse.ArgumentTypeError("参数不能小于 0")
    return number


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="使用 GELLO 飞特舵机遥操 Panthera 从臂"
    )
    parser.add_argument(
        "--gello-port",
        "--port",
        dest="gello_port",
        default=None,
        help="GELLO 串口；不指定时按现有 GELLO 程序规则检测",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=FEETECH_BAUD,
        help="飞特波特率（默认：1000000）",
    )
    parser.add_argument(
        "--joint-signs",
        nargs=6,
        type=float,
        default=DEFAULT_JOINT_SIGNS,
        metavar="SIGN",
        help="GELLO 六关节方向（默认全部为 -1）",
    )
    parser.add_argument(
        "--joint-scales",
        nargs=6,
        type=positive_float,
        default=DEFAULT_JOINT_SCALES,
        metavar="SCALE",
        help="GELLO 六关节角度倍率（默认均为 1）",
    )
    parser.add_argument(
        "--follower-config",
        default=DEFAULT_FOLLOWER_CONFIG,
        help="Panthera 从臂 Follower.yaml 路径",
    )
    parser.add_argument(
        "--control-rate",
        type=positive_float,
        default=DEFAULT_CONTROL_RATE,
        help="控制频率 Hz（默认：100）",
    )
    parser.add_argument(
        "--kp",
        nargs=6,
        type=nonnegative_float,
        default=DEFAULT_KP,
        metavar="KP",
        help="Panthera 六关节 MIT Kp",
    )
    parser.add_argument(
        "--kd",
        nargs=6,
        type=nonnegative_float,
        default=DEFAULT_KD,
        metavar="KD",
        help="Panthera 六关节 MIT Kd",
    )
    parser.add_argument(
        "--gello-timeout",
        type=positive_float,
        default=DEFAULT_GELLO_TIMEOUT,
        help="运行中 GELLO 连续无有效帧的停止阈值 s（默认：0.5）",
    )
    parser.add_argument(
        "--startup-timeout",
        type=positive_float,
        default=3.0,
        help="启动时等待 GELLO 完整帧的时间 s（默认：3）",
    )
    parser.add_argument(
        "--tick-jump-threshold",
        type=int,
        default=FEETECH_TICK_JUMP_THRESHOLD,
        help="飞特相邻帧最大 tick 跳变（默认：800）",
    )
    parser.add_argument(
        "--no-gripper",
        action="store_true",
        help="不控制 Panthera 从臂夹爪",
    )
    parser.add_argument(
        "--gripper-sign",
        type=float,
        default=-1.0,
        help="GELLO 夹爪方向（默认：-1）",
    )
    parser.add_argument(
        "--gripper-tick-span",
        type=positive_float,
        default=700.0,
        help="GELLO 夹爪全行程 tick（默认：700）",
    )
    parser.add_argument(
        "--gripper-kp",
        type=nonnegative_float,
        default=DEFAULT_GRIPPER_KP,
        help="从臂夹爪 MIT Kp（默认：4.0）",
    )
    parser.add_argument(
        "--gripper-kd",
        type=nonnegative_float,
        default=DEFAULT_GRIPPER_KD,
        help="从臂夹爪 MIT Kd（默认：0.4）",
    )
    parser.add_argument(
        "--print-interval",
        type=positive_float,
        default=0.5,
        help="状态打印周期 s（默认：0.5）",
    )
    parser.add_argument(
        "--duration",
        type=nonnegative_float,
        default=0.0,
        help="运行时长 s；0 表示一直运行（默认：0）",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只测试 GELLO 读取和方向，不连接或控制 Panthera",
    )
    return parser


def validate_arguments(parser: argparse.ArgumentParser, args):
    if args.baudrate <= 0:
        parser.error("波特率必须大于 0")
    if args.tick_jump_threshold <= 0:
        parser.error("tick 跳变阈值必须大于 0")
    if any(abs(sign) < 1e-9 for sign in args.joint_signs):
        parser.error("关节方向不能为 0")
    if abs(args.gripper_sign) < 1e-9:
        parser.error("夹爪方向不能为 0")


def main() -> int:
    install_signal_handlers()
    parser = build_argument_parser()
    args = parser.parse_args()
    validate_arguments(parser, args)

    try:
        sdk = import_feetech_sdk()
    except RuntimeError as error:
        print("错误：{}".format(error), file=sys.stderr)
        return 1

    return run(args, sdk)


if __name__ == "__main__":
    sys.exit(main())
