#!/usr/bin/env python3
"""
正弦轨迹跟踪控制程序（带期望/实际轨迹对比图）
机器人关节和夹爪沿着正弦函数轨迹运动，夹爪范围取自配置限位
"""
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from Panthera_lib import Panthera


def move_gripper_to(position, velocity, max_torque, timeout=15.0):
    """缓慢移动夹爪并确认到位，避免从当前位置直接跳到正弦轨迹中点。"""
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        if robot.gripper_control(position, velocity, max_torque) is False:
            raise RuntimeError("夹爪位置命令被拒绝")
        actual = float(robot.get_current_pos_gripper())
        if not np.isfinite(actual) or abs(actual) >= 999:
            raise RuntimeError("夹爪位置反馈无效，请检查通信")
        if abs(actual - position) <= 0.03:
            return
        time.sleep(0.02)
    raise RuntimeError("夹爪移动到目标位置超时")


def plot_tracking_result(times, desired_positions, actual_positions, save_path):
    """绘制六关节及可选夹爪的期望/实际轨迹对比图"""
    if not times:
        print("没有采集到轨迹数据，跳过绘图")
        return
    desired_positions = np.asarray(desired_positions)
    actual_positions = np.asarray(actual_positions)

    channel_count = desired_positions.shape[1]
    rows = (channel_count + 1) // 2
    fig, axes = plt.subplots(rows, 2, figsize=(14, rows * 3.3), sharex=True)
    axes = axes.flatten()

    for joint_idx, ax in enumerate(axes):
        if joint_idx >= channel_count:
            ax.set_visible(False)
            continue
        ax.plot(times, desired_positions[:, joint_idx], label="desired", linewidth=2.0)
        ax.plot(times, actual_positions[:, joint_idx], label="actual", linewidth=1.5, alpha=0.85)
        ax.set_title(f"Joint {joint_idx + 1}" if joint_idx < 6 else "Gripper")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position (rad)")
        ax.grid(True, alpha=0.3)
        ax.legend()

    fig.suptitle("Sin Trajectory Tracking: Desired vs Actual", fontsize=14)
    fig.tight_layout()
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    print(f"\n轨迹对比图已保存到: {save_path}")

    try:
        plt.show()
    except Exception as exc:
        print(f"图形窗口未显示，仅保存图片: {exc}")
    finally:
        plt.close(fig)


def main():
    # 控制参数
    frequency = 0.2  # Hz，正弦波频率（可调节：0.1-2.0 Hz）(频率越高速度越快)
    duration = 10.0  # 运动持续时间（秒）
    control_rate = 500  # 控制频率 Hz
    dt = 1.0 / control_rate
    gripper_frequency = frequency  # 与前六个关节共用频率
    gripper_range_ratio = 0.8  # 使用限位总行程的 80%，两端各留 10%
    gripper_move_velocity = 0.5  # rad/s，移动到中点的速度
    gripper_max_torque = 0.5  # Nm

    if robot.has_gripper:
        if robot.gripper_limits is None:
            raise ValueError("夹爪正弦运动需要配置 gripper_limits")
        gripper_lower = float(robot.gripper_limits["lower"])
        gripper_upper = float(robot.gripper_limits["upper"])
        if not (np.isfinite(gripper_lower) and np.isfinite(gripper_upper)
                and gripper_lower < gripper_upper):
            raise ValueError("夹爪限位必须是有限数值，且下限小于上限")
        if not (0 < gripper_range_ratio < 1 and np.isfinite(gripper_frequency)
                and gripper_frequency > 0):
            raise ValueError("夹爪行程比例必须在 (0, 1) 内，频率必须为正的有限数值")
        gripper_center = (gripper_lower + gripper_upper) / 2
        gripper_amplitude = (gripper_upper - gripper_lower) / 2 * gripper_range_ratio
        gripper_omega = 2 * np.pi * gripper_frequency
        print(f"夹爪正弦范围: [{gripper_center - gripper_amplitude:.3f}, "
              f"{gripper_center + gripper_amplitude:.3f}] rad，频率: {gripper_frequency} Hz")
        print("夹爪缓慢移动到中点...")
        move_gripper_to(gripper_center, gripper_move_velocity, gripper_max_torque)
    else:
        print("未检测到夹爪，仅运行关节正弦轨迹")

    # 使用 Panthera 从 YAML 的 robot.joint_limits 加载的限位（弧度）
    lower_limits = robot.joint_limits["lower"]
    upper_limits = robot.joint_limits["upper"]

    # 获取初始位置作为中心位置（返回np.ndarray）
    print("获取初始位置...")
    center_pos = robot.get_current_pos()
    print(f"中心位置: {center_pos}")

    # 超限时停止，避免生成负的安全振幅
    for i, pos in enumerate(center_pos):
        if pos < lower_limits[i] or pos > upper_limits[i]:
            raise ValueError(f"关节{i+1}初始位置 {pos:.3f} 超出限制范围 [{lower_limits[i]:.3f}, {upper_limits[i]:.3f}]")

    # 设置各关节的振幅（弧度）- 自动调整以避免超限（向量化运算）
    dist_to_upper = upper_limits - center_pos
    dist_to_lower = center_pos - lower_limits
    safe_amplitudes = np.minimum(dist_to_upper, dist_to_lower) * 0.8
    preset_amplitudes = np.array([0.4, 0.6, 0.6, 1.0, 0.8, 0.5])
    amplitudes = np.minimum(safe_amplitudes, preset_amplitudes)

    print(f"调整后的振幅: {amplitudes} rad")

    # 计算并显示最大速度（向量化运算）
    max_velocities = amplitudes * 2 * np.pi * frequency
    print(f"各关节最大速度: {max_velocities} rad/s")

    # 设置各关节的相位偏移（可以让各关节运动不同步）
    # phase_offsets = np.array([0, np.pi/4, np.pi/2, 3*np.pi/4, np.pi, 0])  # 相位偏移
    phase_offsets = np.zeros(robot.motor_count)  # 零相位偏移

    print("\n开始正弦轨迹运动...")
    print(f"频率: {frequency} Hz, 持续时间: {duration} 秒")
    print(f"振幅: {amplitudes}")

    times, desired_positions, actual_positions = [], [], []
    start_time = time.perf_counter()
    step = 0

    try:
        while (time.perf_counter() - start_time) < duration:
            loop_start = time.perf_counter()
            current_time = loop_start - start_time

            # 计算正弦轨迹（向量化运算）
            omega = 2 * np.pi * frequency

            # 位置：x = x0 + A * sin(ωt + φ)（向量化）
            pos = center_pos + amplitudes * np.sin(omega * current_time + phase_offsets)

            # 速度（位置的导数）：v = A * ω * cos(ωt + φ)（向量化）
            vel = amplitudes * omega * np.cos(omega * current_time + phase_offsets)

            # 角度限幅（向量化）
            # 限制位置在关节限制范围内
            below_limit = pos < lower_limits
            above_limit = pos > upper_limits
            pos = np.clip(pos, lower_limits, upper_limits)
            # 到达限位时速度置零
            vel[below_limit | above_limit] = 0

            robot.Joint_Pos_Vel(pos, vel, robot.max_torque, iswait=False)

            if robot.has_gripper:
                gripper_pos = float(np.clip(
                    gripper_center + gripper_amplitude * np.sin(gripper_omega * current_time),
                    gripper_lower, gripper_upper))
                gripper_vel = float(gripper_amplitude * gripper_omega
                                    * np.cos(gripper_omega * current_time))
                if robot.gripper_control(gripper_pos, gripper_vel, gripper_max_torque) is False:
                    raise RuntimeError("夹爪正弦轨迹命令被拒绝")

            # 仅采集数据，运动结束后绘图，避免绘图阻塞控制循环。
            desired = list(pos)
            actual = list(robot.get_current_pos())
            if robot.has_gripper:
                desired.append(gripper_pos)
                actual.append(float(robot.get_current_pos_gripper()))
            times.append(current_time)
            desired_positions.append(desired)
            actual_positions.append(actual)

            # 定期打印状态
            if step % 50 == 0:  # 每0.5秒打印一次
                print(f"\r时间: {current_time:.2f}s | "
                      f"关节1位置: {pos[0]:.3f} | "
                      f"关节2位置: {pos[1]:.3f} | "
                      f"关节3位置: {pos[2]:.3f}"
                      + (f" | 夹爪位置: {gripper_pos:.3f} rad" if robot.has_gripper else ""),
                      end="")

            step += 1

            # 控制循环频率
            loop_time = time.perf_counter() - loop_start
            if loop_time < dt:
                time.sleep(dt - loop_time)

    except KeyboardInterrupt:
        print("\n\n轨迹被中断")

    # 返回中心位置
    print("\n\n返回中心位置...")
    if robot.has_gripper:
        move_gripper_to(gripper_center, gripper_move_velocity, gripper_max_torque)
    robot.Joint_Pos_Vel(center_pos, [0.5] * robot.motor_count, [10.0] * robot.motor_count, iswait=True)

    print("运动完成")
    time.sleep(1)
    output_path = Path(__file__).resolve().with_name("sin_trajectory_tracking.png")
    plot_tracking_result(times, desired_positions, actual_positions, output_path)

if __name__ == "__main__":
    robot = Panthera()

    # 先移动到安全的初始位置
    print("移动到初始位置...")
    zero_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # init_pos = [0.0, -1.0, -1.0, 0.0, 0.0, 0.0]
    init_pos = [-0.3, 1.1, 1.1, 0.2, -0.3, 0.0]
    vel = [0.5] * robot.motor_count

    success = robot.Joint_Pos_Vel(zero_pos, vel, robot.max_torque, iswait=True)
    time.sleep(3)

    success = robot.Joint_Pos_Vel(init_pos, vel, robot.max_torque, iswait=True)
    if success:
        print("到达初始位置")
        time.sleep(1)

    try:
        main()
        success = robot.Joint_Pos_Vel(zero_pos, vel, robot.max_torque, iswait=True)
        time.sleep(2)
    except KeyboardInterrupt:
        # robot.set_stop()
        print("\n\n程序被中断")
        print("\n\n所有电机已停止")
    except Exception as e:
        print(f"\n错误: {e}")
        # robot.set_stop()
