#!/usr/bin/env python3
"""北通 A2P3A/XInput 手柄控制机械臂笛卡尔速度。"""

import os
import sys
import threading
import time

import numpy as np

from Panthera_lib import Panthera


linear_speed = 0.3       # 最大线速度 m/s
angular_speed = 2.0      # 最大角速度 rad/s
stick_deadzone = 0.08    # 摇杆死区
trigger_deadzone = 0.05  # 扣机死区
gripper_speed = 0.5      # 夹爪速度 rad/s
gripper_limit_margin = 0.02
gripper_open_position = 1.6   # 与 Panthera.gripper_open() 默认位置一致
gripper_close_position = 0.0
default_gamepad_device = (
    "/dev/input/by-id/"
    "usb-BEITONG_BEITONG_A2P3A_XINPUT_DONGLE-event-joystick"
)

# 平滑和定时参数
control_period = 0.01             # 100 Hz 目标周期
display_period = 0.10             # 状态输出 10 Hz，避免终端阻塞控制循环
max_control_dt = 0.03             # 长时延迟时限制单次积分步长
adaptation_time_constant = 0.08   # 可操作度/阻尼参数低通时常数
linear_acceleration_limit = 0.8   # m/s²
linear_jerk_limit = 8.0           # m/s³
angular_acceleration_limit = 5.0  # rad/s²
angular_jerk_limit = 40.0         # rad/s³
acceleration_filter_time = 0.04   # 加速度低通时常数
velocity_tracking_time = 0.16     # 速度误差转为期望加速度的时常数
linear_settle_tolerance = 0.0005  # m/s，低于此误差时允许精确停稳
angular_settle_tolerance = 0.003  # rad/s

# 电机协议的速度分辨率约为 2π/4000 = 0.00157 rad/s。
# 在量化区内采用迟滞启停，避免零速附近反复跳码。
joint_motion_start_threshold = 0.0045  # rad/s，约 3 个量化单位
joint_motion_stop_threshold = 0.003    # rad/s，约 2 个量化单位

running = True
robot_keepalive = None
safe_shutdown_complete = False


class JerkLimitedVelocityFilter:
    """无零速振荡的二阶速度滤波器，同时限制加速度和 jerk。"""

    def __init__(self, max_acceleration, max_jerk,
                 acceleration_time, tracking_time, settle_tolerance):
        self.max_acceleration = np.asarray(max_acceleration, dtype=float)
        self.max_jerk = np.asarray(max_jerk, dtype=float)
        self.acceleration_time = float(acceleration_time)
        self.tracking_time = float(tracking_time)
        self.settle_tolerance = np.asarray(settle_tolerance, dtype=float)
        self.velocity = np.zeros_like(self.max_acceleration)
        self.acceleration = np.zeros_like(self.max_acceleration)

    def update(self, target_velocity, dt):
        target_velocity = np.asarray(target_velocity, dtype=float)
        error = target_velocity - self.velocity

        # 速度误差只决定期望加速度；加速度再经一阶低通。
        # tracking_time = 4 * acceleration_time 时为临界阻尼，
        # 不会像旧实现那样在零速附近交替修正加速度。
        desired_acceleration = error / self.tracking_time
        desired_acceleration = np.clip(
            desired_acceleration,
            -self.max_acceleration,
            self.max_acceleration,
        )

        acceleration_alpha = 1.0 - np.exp(-dt / self.acceleration_time)
        max_acceleration_change = self.max_jerk * dt
        previous_acceleration = self.acceleration.copy()
        self.acceleration += np.clip(
            acceleration_alpha * (
                desired_acceleration - self.acceleration
            ),
            -max_acceleration_change,
            max_acceleration_change,
        )
        self.acceleration = np.clip(
            self.acceleration,
            -self.max_acceleration,
            self.max_acceleration,
        )

        # 梯形积分比显式欧拉在低速段更均匀。
        self.velocity += 0.5 * (
            previous_acceleration + self.acceleration
        ) * dt
        remaining_error = target_velocity - self.velocity
        crossed_target = error * remaining_error < 0.0
        can_zero_acceleration = (
            np.abs(self.acceleration) <= self.max_jerk * dt
        )
        settled = (
            (np.abs(remaining_error) <= self.settle_tolerance) &
            can_zero_acceleration
        )
        finished = crossed_target | settled
        self.velocity[finished] = target_velocity[finished]
        self.acceleration[finished] = 0.0
        return self.velocity.copy()

    def reset(self):
        self.velocity[:] = 0.0
        self.acceleration[:] = 0.0


def limit_vector(target, limits, origin=None):
    """从 origin 向 target 等比例逼近，不改变向量方向。"""
    target = np.asarray(target, dtype=float)
    origin = np.zeros_like(target) if origin is None else np.asarray(origin)
    delta = target - origin
    moving = np.abs(delta) > 1e-12
    if not np.any(moving):
        return target.copy(), 1.0
    scale = min(
        1.0,
        float(np.min(np.asarray(limits)[moving] / np.abs(delta[moving]))),
    )
    return origin + delta * scale, scale


class GamepadController:
    """北通 A2P3A（Xbox 360/XInput）手柄监听器。"""

    def __init__(self, on_exit, on_failure, device_path=None):
        try:
            import evdev
        except ImportError as exc:
            raise RuntimeError("缺少 evdev，请执行: pip install evdev") from exc

        self.evdev = evdev
        self.on_exit = on_exit
        self.on_failure = on_failure
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None
        self.velocity = np.zeros(6)
        self.axis_values = {
            'lx': 0.0, 'ly': 0.0,
            'rx': 0.0, 'ry': 0.0,
            'lt': 0.0, 'rt': 0.0,
        }
        self.button_values = {
            'lb': 0.0, 'rb': 0.0,
            'a': 0.0, 'b': 0.0,
        }
        self.device = self._open_gamepad(device_path or default_gamepad_device)
        ecodes = evdev.ecodes
        self.axis_mapping = {
            ecodes.ABS_X: ('lx', self._normalize_stick),
            ecodes.ABS_Y: ('ly', self._normalize_stick),
            ecodes.ABS_RX: ('rx', self._normalize_stick),
            ecodes.ABS_RY: ('ry', self._normalize_stick),
            ecodes.ABS_Z: ('lt', self._normalize_trigger),
            ecodes.ABS_RZ: ('rt', self._normalize_trigger),
        }
        self.button_mapping = {
            ecodes.BTN_TL: 'lb',
            ecodes.BTN_TR: 'rb',
            ecodes.BTN_SOUTH: 'a',
            ecodes.BTN_EAST: 'b',
        }
        self.exit_buttons = {ecodes.BTN_START, ecodes.BTN_MODE}

    def _open_gamepad(self, device_path):
        ecodes = self.evdev.ecodes
        try:
            device = self.evdev.InputDevice(device_path)
        except PermissionError as exc:
            raise RuntimeError(
                f"无权读取手柄设备: {device_path}\n"
                "请将当前用户加入 input 组后注销并重新登录：\n"
                "  sudo usermod -aG input $USER"
            ) from exc
        except OSError as exc:
            raise RuntimeError(f"无法打开北通手柄: {device_path} ({exc})") from exc

        capabilities = device.capabilities(absinfo=False)
        abs_codes = set(capabilities.get(ecodes.EV_ABS, []))
        key_codes = set(capabilities.get(ecodes.EV_KEY, []))
        required_abs = {
            ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_RX,
            ecodes.ABS_RY, ecodes.ABS_Z, ecodes.ABS_RZ,
        }
        required_keys = {
            ecodes.BTN_TL, ecodes.BTN_TR,
            ecodes.BTN_SOUTH, ecodes.BTN_EAST,
        }
        if not required_abs.issubset(abs_codes) or not required_keys.issubset(key_codes):
            device.close()
            raise RuntimeError(f"设备不是预期的北通 XInput 手柄: {device_path}")

        abs_capabilities = device.capabilities(absinfo=True)
        self.abs_info = dict(abs_capabilities.get(ecodes.EV_ABS, []))
        return device

    @property
    def description(self):
        return f"{self.device.path}: {self.device.name} (北通 XInput 固定映射)"

    @staticmethod
    def _clamp(value, minimum=-1.0, maximum=1.0):
        return max(minimum, min(maximum, value))

    def _normalize_stick(self, code, raw_value):
        info = self.abs_info[code]
        center = (info.min + info.max) / 2.0
        span = max(center - info.min, info.max - center)
        if span <= 0:
            return 0.0
        value = self._clamp((raw_value - center) / span)
        hardware_deadzone = max(0.0, float(info.flat) / span)
        deadzone = max(stick_deadzone, hardware_deadzone)
        if abs(value) <= deadzone:
            return 0.0
        return np.sign(value) * (abs(value) - deadzone) / (1.0 - deadzone)

    def _normalize_trigger(self, code, raw_value):
        info = self.abs_info[code]
        span = info.max - info.min
        if span <= 0:
            return 0.0
        value = self._clamp((raw_value - info.min) / span, 0.0, 1.0)
        if value <= trigger_deadzone:
            return 0.0
        return (value - trigger_deadzone) / (1.0 - trigger_deadzone)

    def _update_velocity_locked(self):
        # 左摇杆向上/向左的 Linux 原始值为负，分别映射为 W/A。
        self.velocity[0] = -self.axis_values['ly'] * linear_speed
        self.velocity[1] = -self.axis_values['lx'] * linear_speed
        # RT=Q（+Z），LT=E（-Z）。
        self.velocity[2] = (
            self.axis_values['rt'] - self.axis_values['lt']
        ) * linear_speed
        # LB=1，RB=2。
        self.velocity[3] = (
            self.button_values['lb'] - self.button_values['rb']
        ) * angular_speed
        # 右摇杆上/下=3/4，左/右=5/6。
        self.velocity[4] = -self.axis_values['ry'] * angular_speed
        self.velocity[5] = -self.axis_values['rx'] * angular_speed

    def get_commands(self):
        with self.lock:
            gripper_direction = (
                self.button_values['a'] - self.button_values['b']
            )
            return self.velocity.copy(), gripper_direction

    def clear_velocity(self):
        with self.lock:
            self.velocity[:] = 0.0
            for name in self.button_values:
                self.button_values[name] = 0.0

    def start(self):
        self.thread = threading.Thread(
            target=self._run,
            name="panthera-gamepad",
            daemon=True,
        )
        self.thread.start()

    def _run(self):
        ecodes = self.evdev.ecodes
        try:
            for event in self.device.read_loop():
                if self.stop_event.is_set():
                    break
                if event.type == ecodes.EV_ABS and event.code in self.axis_mapping:
                    name, normalize = self.axis_mapping[event.code]
                    value = normalize(event.code, event.value)
                    with self.lock:
                        self.axis_values[name] = value
                        self._update_velocity_locked()
                elif (event.type == ecodes.EV_KEY and
                      event.code in self.button_mapping and
                      event.value in (0, 1)):
                    with self.lock:
                        self.button_values[self.button_mapping[event.code]] = float(event.value)
                        self._update_velocity_locked()
                elif (event.type == ecodes.EV_KEY and event.value == 1 and
                      event.code in self.exit_buttons):
                    self.clear_velocity()
                    self.on_exit()
                    break
        except OSError as exc:
            if not self.stop_event.is_set():
                self.clear_velocity()
                self.on_failure(exc)
        except Exception as exc:
            self.clear_velocity()
            self.on_failure(exc)

    def is_alive(self):
        return self.thread is not None and self.thread.is_alive()

    def stop(self):
        self.stop_event.set()
        self.clear_velocity()
        self.device.close()
        if self.thread is not None and self.thread is not threading.current_thread():
            self.thread.join(timeout=0.5)


def request_exit():
    global running
    running = False
    print("\n检测到 START/Menu 键，准备退出...")


def gamepad_failed(exc):
    global running
    running = False
    print(f"\n手柄监听中断: {exc}")


def move_to_safe_position(robot):
    """启动时缓慢移动到安全位置。"""
    print("\n" + "=" * 60)
    print("正在移动到安全位置...")
    print("=" * 60)
    safe_joint_pos = [0.0, 0.5, 0.6, 0.0, 0.0, 0.0]
    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    time.sleep(0.3)
    print("移动中...")
    success = robot.Joint_Pos_Vel(
        safe_joint_pos, [0.5] * robot.motor_count, iswait=True
    )
    if not success:
        print("✗ 移动到安全位置失败")
        return False
    print("✓ 已到达安全位置")
    time.sleep(0.5)
    return True


def brake_robot(robot):
    """显式发送刹车命令，兼容 hightorque_robot 1.2.0 退出问题。"""
    for motor in robot.Motors:
        motor.brake()
    for _ in range(3):
        robot.motor_send_cmd()
        time.sleep(0.01)


def set_gripper_velocity(robot, direction):
    """
    设置夹爪速度（不单独发送）。

    随后的 robot.Joint_Vel() 会在同一个速度数据包中填入六个关节
    速度并统一发送，避免夹爪命令覆盖机械臂命令。
    """
    velocity = float(direction) * gripper_speed
    current_position = robot.get_current_pos_gripper()
    lower = gripper_close_position
    upper = gripper_open_position
    if robot.gripper_limits is not None:
        lower = max(lower, robot.gripper_limits['lower'])
        upper = min(upper, robot.gripper_limits['upper'])
    if current_position >= upper - gripper_limit_margin and velocity > 0.0:
        velocity = 0.0
    elif current_position <= lower + gripper_limit_margin and velocity < 0.0:
        velocity = 0.0

    robot.Motors[robot.gripper_id - 1].velocity(velocity)
    return velocity


def main():
    global robot_keepalive, safe_shutdown_complete

    print("=" * 60)
    print("机械臂笛卡尔空间手柄速度控制程序")
    print("=" * 60)

    # 在机械臂运动前先检查手柄和读取权限。
    gamepad = GamepadController(
        request_exit,
        gamepad_failed,
        os.environ.get('PANTHERA_GAMEPAD_DEVICE'),
    )
    print(f"\n手柄设备: {gamepad.description}")

    print("\n初始化机械臂...")
    robot = Panthera()
    if __name__ == "__main__":
        robot_keepalive = robot

    if robot.model is None:
        print("错误：未找到 Pinocchio 模型，无法计算雅可比矩阵")
        return
    if not move_to_safe_position(robot):
        print("初始化失败，退出程序")
        return

    current_fk = robot.forward_kinematics()
    current_pos = current_fk['position']
    print(
        f"\n初始位置: [{current_pos[0]:.3f}, "
        f"{current_pos[1]:.3f}, {current_pos[2]:.3f}] m"
    )
    print("\n" + "=" * 60)
    print("手柄控制说明（摇杆/扣机按比例调速）：")
    print("  左摇杆: W/S + A/D（X/Y 线速度）")
    print("  RT/LT: Q/E（Z 线速度 +/-）")
    print("  LB/RB: 1/2（绕 X 轴 +/-）")
    print("  右摇杆上/下: 3/4（绕 Y 轴 +/-）")
    print("  右摇杆左/右: 5/6（绕 Z 轴 +/-）")
    print("  A/B: 按住打开/关闭夹爪（松开即停）")
    print("  START/Menu: 退出")
    print("=" * 60)
    print("\n开始控制，请小心操作！\n")

    gamepad.start()

    damping_base = 0.01
    kp = [0.0] * robot.motor_count
    kd = [10.0, 15.0, 15.0, 10.0, 5.0, 5.0]
    max_joint_vel = np.asarray(robot.velocity_limits, dtype=float)
    joint_acceleration_limits = np.asarray(robot.acceleration_limits, dtype=float)
    cartesian_filter = JerkLimitedVelocityFilter(
        max_acceleration=np.array(
            [linear_acceleration_limit] * 3 +
            [angular_acceleration_limit] * 3
        ),
        max_jerk=np.array(
            [linear_jerk_limit] * 3 + [angular_jerk_limit] * 3
        ),
        acceleration_time=acceleration_filter_time,
        tracking_time=velocity_tracking_time,
        settle_tolerance=np.array(
            [linear_settle_tolerance] * 3 +
            [angular_settle_tolerance] * 3
        ),
    )
    previous_joint_velocity = np.zeros(robot.motor_count)
    joint_motion_enabled = False
    filtered_damping = damping_base
    filtered_speed_scale = 1.0
    last_cycle_time = time.monotonic() - control_period
    next_cycle_time = last_cycle_time + control_period
    last_display_time = last_cycle_time - display_period

    try:
        while running:
            # 以绝对截止时间调度，避免“运算耗时 + 固定 sleep”造成周期漂移。
            now = time.monotonic()
            remaining = next_cycle_time - now
            if remaining > 0.0:
                time.sleep(remaining)
                now = time.monotonic()

            measured_dt = max(now - last_cycle_time, 1e-4)
            dt = min(measured_dt, max_control_dt)
            last_cycle_time = now
            next_cycle_time += control_period
            if next_cycle_time <= now:
                # 本周期严重超时就重新同步，不连续补发积压的速度命令。
                next_cycle_time = now + control_period

            if not gamepad.is_alive():
                gamepad_failed("监听线程已退出")
                break

            commanded_velocity, gripper_direction = gamepad.get_commands()
            q = robot.get_current_pos()
            try:
                J = robot.get_jacobian(q)
            except Exception as exc:
                print(f"\r雅可比计算失败: {exc}                    ", end='')
                # 雅可比异常时平滑减速，避免偶发计算错误形成硬停顿。
                cartesian_filter.update(np.zeros(6), dt)
                previous_joint_velocity, _ = limit_vector(
                    np.zeros(robot.motor_count),
                    joint_acceleration_limits * dt,
                    previous_joint_velocity,
                )
                set_gripper_velocity(robot, gripper_direction)
                robot.Joint_Vel(previous_joint_velocity)
                continue

            # 复用本周期的雅可比，避免 get_manipulability 再算一次雅可比。
            singular_values = np.linalg.svd(J, compute_uv=False)
            manipulability = float(np.prod(singular_values))
            manip_threshold = 0.01
            if manipulability < manip_threshold:
                target_damping = damping_base * (
                    1.0 + (manip_threshold - manipulability) * 100
                )
                target_speed_scale = max(0.0, manipulability / manip_threshold)
            else:
                target_damping = damping_base
                target_speed_scale = 1.0

            # 奇异区附近的可操作度会因编码器噪声微变；低通后再调速/调阻尼。
            adaptation_alpha = 1.0 - np.exp(
                -dt / adaptation_time_constant
            )
            filtered_damping += adaptation_alpha * (
                target_damping - filtered_damping
            )
            filtered_speed_scale += adaptation_alpha * (
                target_speed_scale - filtered_speed_scale
            )
            end_velocity_scaled = commanded_velocity * filtered_speed_scale

            J_damp = Panthera.compute_damped_pseudoinverse(
                J, filtered_damping
            )
            actual_velocity = cartesian_filter.update(
                end_velocity_scaled, dt
            )

            # 与雅可比使用同一份关节快照，避免额外串口读取和姿态时差。
            current_fk = robot.forward_kinematics(q)
            current_pos = current_fk['position']
            R_tool = current_fk['rotation']
            end_velocity_world = np.zeros(6)
            end_velocity_world[0:3] = R_tool @ actual_velocity[0:3]
            end_velocity_world[3:6] = R_tool @ actual_velocity[3:6]
            raw_joint_velocity = J_damp @ end_velocity_world
            target_joint_velocity, joint_velocity_scale = limit_vector(
                raw_joint_velocity, max_joint_vel
            )
            q_dot, _ = limit_vector(
                target_joint_velocity,
                joint_acceleration_limits * dt,
                previous_joint_velocity,
            )

            # 速度模式在 1~2 个协议量化单位附近容易产生启停抖动。
            # 用不同的启/停阈值形成迟滞；只在手柄已回中时提前停稳，
            # 不修改正常运动阶段的目标速度。
            motion_requested = np.any(
                np.abs(commanded_velocity) > 1e-9
            )
            joint_speed_peak = float(np.max(np.abs(q_dot)))
            if joint_motion_enabled:
                if (not motion_requested and
                        joint_speed_peak <= joint_motion_stop_threshold):
                    joint_motion_enabled = False
                    cartesian_filter.reset()
                    actual_velocity = cartesian_filter.velocity.copy()
                    q_dot = np.zeros(robot.motor_count)
            elif (motion_requested and
                  joint_speed_peak >= joint_motion_start_threshold):
                joint_motion_enabled = True
            else:
                q_dot = np.zeros(robot.motor_count)

            previous_joint_velocity = q_dot
            commanded_gripper_velocity = set_gripper_velocity(
                robot, gripper_direction
            )
            robot.Joint_Vel(q_dot)

            if now - last_display_time >= display_period:
                last_display_time = now
                linear_norm = np.linalg.norm(actual_velocity[:3])
                angular_norm = np.linalg.norm(actual_velocity[3:])
                print(
                    f"\r位置: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, "
                    f"{current_pos[2]:.3f}] | 线速度: {linear_norm:.3f} m/s | "
                    f"角速度: {angular_norm:.3f} rad/s | "
                    f"夹爪速度: {commanded_gripper_velocity:+.2f} rad/s | "
                    f"可操作度: {manipulability:.4f} | "
                    f"关节缩放: {joint_velocity_scale:.2f} | "
                    f"周期: {measured_dt * 1000.0:.1f} ms",
                    end='',
                    flush=True,
                )

    except KeyboardInterrupt:
        print("\n\n程序被中断")
    finally:
        gamepad.stop()
        print("\n\n停止运动...")
        robot.Motors[robot.gripper_id - 1].velocity(0.0)
        robot.motor_send_cmd()
        q = robot.get_current_pos()
        gra = robot.get_Gravity()
        robot.pos_vel_tqe_kp_kd(
            q, [0.0] * robot.motor_count, gra, kp, kd
        )
        time.sleep(0.5)

        print("返回零位...")
        zero_pos = [0.0] * robot.motor_count
        robot.Joint_Pos_Vel(
            zero_pos, [0.5] * robot.motor_count, iswait=True
        )
        print("所有电机已停止")
        brake_robot(robot)
        print("所有电机已刹车")
        safe_shutdown_complete = True


if __name__ == "__main__":
    exit_code = 0
    try:
        main()
    except Exception as exc:
        exit_code = 1
        print(f"\n错误: {exc}")
        import traceback
        traceback.print_exc()
    finally:
        if safe_shutdown_complete:
            sys.stdout.flush()
            sys.stderr.flush()
            os._exit(exit_code)
