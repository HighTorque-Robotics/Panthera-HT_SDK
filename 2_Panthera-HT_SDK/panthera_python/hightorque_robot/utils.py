"""
辅助工具函数

提供常用的电机控制辅助函数和工具
"""

import math
import time
from typing import List, Tuple, Callable
import numpy as np


def rad_to_deg(rad: float) -> float:
    """
    弧度转角度

    参数:
        rad: 弧度值

    返回:
        角度值
    """
    return rad * 180.0 / math.pi


def deg_to_rad(deg: float) -> float:
    """
    角度转弧度

    参数:
        deg: 角度值

    返回:
        弧度值
    """
    return deg * math.pi / 180.0


def print_motor_states(motors, detailed=False):
    """
    打印电机状态信息

    参数:
        motors: Motor对象列表
        detailed: 是否显示详细信息
    """
    print("=" * 80)
    print(f"电机状态 - 共 {len(motors)} 个电机")
    print("-" * 80)

    if detailed:
        print(f"{'ID':>4} {'名称':<15} {'模式':>4} {'故障':>4} "
              f"{'位置(rad)':>12} {'速度(rad/s)':>12} {'力矩(Nm)':>12} "
              f"{'温度(°C)':>10} {'电压(V)':>10}")
        print("-" * 80)
    else:
        print(f"{'ID':>4} {'名称':<15} {'位置(rad)':>12} "
              f"{'速度(rad/s)':>12} {'力矩(Nm)':>12}")
        print("-" * 80)

    for motor in motors:
        state = motor.get_current_motor_state()
        if detailed:
            print(f"{state.ID:>4} {motor.get_motor_name():<15} "
                  f"{state.mode:>4} {state.fault:>4X} "
                  f"{state.position:>12.4f} {state.velocity:>12.4f} "
                  f"{state.torque:>12.4f} {state.temperature:>10.1f} "
                  f"{state.voltage:>10.2f}")
        else:
            print(f"{state.ID:>4} {motor.get_motor_name():<15} "
                  f"{state.position:>12.4f} {state.velocity:>12.4f} "
                  f"{state.torque:>12.4f}")

    print("=" * 80)


def setup_motor_control(robot, motor_indices=None,
                       pos_limits=None, tor_limits=None):
    """
    批量设置电机控制参数

    参数:
        robot: Robot对象
        motor_indices: 要设置的电机索引列表,None表示所有电机
        pos_limits: 位置限制 [(upper1, lower1), (upper2, lower2), ...]
        tor_limits: 力矩限制 [(upper1, lower1), (upper2, lower2), ...]

    返回:
        设置的电机列表
    """
    motors = robot.get_motors()

    if motor_indices is None:
        motor_indices = range(len(motors))

    selected_motors = [motors[i] for i in motor_indices]

    # 设置位置限制
    if pos_limits is not None:
        for i, motor in enumerate(selected_motors):
            if i < len(pos_limits):
                upper, lower = pos_limits[i]
                motor.set_pos_limit(upper, lower)
                print(f"电机 {motor.get_motor_id()} 位置限制: "
                      f"[{lower:.2f}, {upper:.2f}]")

    # 设置力矩限制
    if tor_limits is not None:
        for i, motor in enumerate(selected_motors):
            if i < len(tor_limits):
                upper, lower = tor_limits[i]
                motor.set_tor_limit(upper, lower)
                print(f"电机 {motor.get_motor_id()} 力矩限制: "
                      f"[{lower:.2f}, {upper:.2f}]")

    return selected_motors


def create_sinusoidal_trajectory(amplitude: float,
                                 frequency: float,
                                 phase: float = 0.0,
                                 offset: float = 0.0) -> Callable[[float], float]:
    """
    创建正弦轨迹生成器

    参数:
        amplitude: 振幅
        frequency: 频率 (Hz)
        phase: 相位 (弧度)
        offset: 偏移量

    返回:
        轨迹函数 f(t) = amplitude * sin(2π * frequency * t + phase) + offset

    示例:
        >>> traj = create_sinusoidal_trajectory(1.0, 0.5)
        >>> position = traj(time.time())
    """
    omega = 2 * math.pi * frequency

    def trajectory(t: float) -> float:
        return amplitude * math.sin(omega * t + phase) + offset

    return trajectory


def create_trapezoidal_profile(start: float, end: float,
                               max_vel: float, max_acc: float) -> Tuple[List[float], List[float], List[float]]:
    """
    创建梯形速度轮廓

    参数:
        start: 起始位置
        end: 结束位置
        max_vel: 最大速度
        max_acc: 最大加速度

    返回:
        (时间列表, 位置列表, 速度列表)
    """
    distance = abs(end - start)
    direction = 1 if end > start else -1

    # 计算加速时间和距离
    t_acc = max_vel / max_acc
    s_acc = 0.5 * max_acc * t_acc ** 2

    # 判断是否能达到最大速度
    if 2 * s_acc <= distance:
        # 梯形轮廓
        s_cruise = distance - 2 * s_acc
        t_cruise = s_cruise / max_vel
        t_total = 2 * t_acc + t_cruise
    else:
        # 三角形轮廓
        t_acc = math.sqrt(distance / max_acc)
        t_cruise = 0
        t_total = 2 * t_acc

    # 生成轨迹
    dt = 0.001  # 1ms
    times = np.arange(0, t_total, dt)
    positions = []
    velocities = []

    for t in times:
        if t < t_acc:
            # 加速阶段
            s = 0.5 * max_acc * t ** 2
            v = max_acc * t
        elif t < t_acc + t_cruise:
            # 匀速阶段
            s = s_acc + max_vel * (t - t_acc)
            v = max_vel
        else:
            # 减速阶段
            t_dec = t - t_acc - t_cruise
            s = distance - 0.5 * max_acc * (t_total - t) ** 2
            v = max_vel - max_acc * t_dec

        positions.append(start + direction * s)
        velocities.append(direction * v)

    return times.tolist(), positions, velocities


class MotorController:
    """
    电机控制器辅助类

    提供高层次的电机控制功能
    """

    def __init__(self, robot):
        """
        初始化控制器

        参数:
            robot: Robot对象
        """
        self.robot = robot
        self.motors = robot.get_motors()
        self.start_time = time.time()

    def get_elapsed_time(self) -> float:
        """获取从初始化以来经过的时间"""
        return time.time() - self.start_time

    def reset_time(self):
        """重置时间计数器"""
        self.start_time = time.time()

    def set_all_positions(self, positions: List[float]):
        """
        设置所有电机位置

        参数:
            positions: 位置列表
        """
        for motor, pos in zip(self.motors, positions):
            motor.position(pos)
        self.robot.motor_send_cmd()

    def set_all_velocities(self, velocities: List[float]):
        """
        设置所有电机速度

        参数:
            velocities: 速度列表
        """
        for motor, vel in zip(self.motors, velocities):
            motor.velocity(vel)
        self.robot.motor_send_cmd()

    def set_all_torques(self, torques: List[float]):
        """
        设置所有电机力矩

        参数:
            torques: 力矩列表
        """
        for motor, tqe in zip(self.motors, torques):
            motor.torque(tqe)
        self.robot.motor_send_cmd()

    def get_all_positions(self) -> List[float]:
        """
        获取所有电机位置

        返回:
            位置列表
        """
        return [m.get_current_motor_state().position for m in self.motors]

    def get_all_velocities(self) -> List[float]:
        """
        获取所有电机速度

        返回:
            速度列表
        """
        return [m.get_current_motor_state().velocity for m in self.motors]

    def get_all_torques(self) -> List[float]:
        """
        获取所有电机力矩

        返回:
            力矩列表
        """
        return [m.get_current_motor_state().torque for m in self.motors]

    def stop_all(self):
        """停止所有电机"""
        self.robot.set_stop()

    def zero_all(self):
        """所有电机回零"""
        self.robot.set_reset_zero()


def safe_control_loop(robot, control_function: Callable,
                     frequency: float = 1000.0,
                     max_iterations: int = None):
    """
    安全的控制循环

    参数:
        robot: Robot对象
        control_function: 控制函数,接收(robot, iteration, elapsed_time)参数
        frequency: 控制频率 (Hz)
        max_iterations: 最大迭代次数,None表示无限循环

    示例:
        >>> def my_control(robot, iter, t):
        ...     motors = robot.get_motors()
        ...     motors[0].position(math.sin(t))
        ...     robot.motor_send_cmd()
        ...     return iter < 1000  # 运行1000次后停止
        >>> safe_control_loop(robot, my_control, frequency=1000)
    """
    dt = 1.0 / frequency
    start_time = time.time()
    iteration = 0

    try:
        while True:
            loop_start = time.time()

            # 调用控制函数
            elapsed_time = time.time() - start_time
            should_continue = control_function(robot, iteration, elapsed_time)

            # 检查是否应该停止
            if should_continue is False:
                break

            if max_iterations is not None and iteration >= max_iterations:
                break

            # 等待到下一个控制周期
            loop_duration = time.time() - loop_start
            sleep_time = dt - loop_duration

            if sleep_time > 0:
                time.sleep(sleep_time)
            elif loop_duration > dt * 1.5:
                print(f"警告: 控制循环超时 ({loop_duration*1000:.2f}ms > {dt*1000:.2f}ms)")

            iteration += 1

    except KeyboardInterrupt:
        print("\n检测到键盘中断,正在停止电机...")
        robot.set_stop()
        print("电机已安全停止")
    except Exception as e:
        print(f"\n错误: {e}")
        print("正在紧急停止电机...")
        robot.set_stop()
        raise


def monitor_motor_health(motors, check_interval: float = 1.0,
                        max_temperature: float = 80.0,
                        max_voltage: float = 30.0,
                        min_voltage: float = 20.0):
    """
    监控电机健康状态

    参数:
        motors: Motor对象列表
        check_interval: 检查间隔 (秒)
        max_temperature: 最高温度阈值 (°C)
        max_voltage: 最高电压阈值 (V)
        min_voltage: 最低电压阈值 (V)

    返回:
        (健康状态, 警告列表)
    """
    warnings = []

    for motor in motors:
        state = motor.get_current_motor_state()
        motor_id = motor.get_motor_id()
        motor_name = motor.get_motor_name()

        # 检查温度
        if state.temperature > max_temperature:
            warnings.append(
                f"电机 {motor_id} ({motor_name}): "
                f"温度过高 {state.temperature:.1f}°C > {max_temperature}°C"
            )

        # 检查电压
        if state.voltage > max_voltage:
            warnings.append(
                f"电机 {motor_id} ({motor_name}): "
                f"电压过高 {state.voltage:.1f}V > {max_voltage}V"
            )
        elif state.voltage < min_voltage:
            warnings.append(
                f"电机 {motor_id} ({motor_name}): "
                f"电压过低 {state.voltage:.1f}V < {min_voltage}V"
            )

        # 检查故障码
        if state.fault != 0:
            warnings.append(
                f"电机 {motor_id} ({motor_name}): "
                f"检测到故障码 0x{state.fault:02X}"
            )

    is_healthy = len(warnings) == 0
    return is_healthy, warnings
