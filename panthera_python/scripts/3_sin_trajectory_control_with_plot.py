#!/usr/bin/env python3
"""
正弦轨迹跟踪控制程序（带期望/实际轨迹对比图）
机器人关节沿着正弦函数轨迹运动，结束后输出可视化曲线图。
"""
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from Panthera_lib import Panthera


def plot_tracking_result(times, desired_positions, actual_positions, save_path):
    """绘制各关节期望/实际轨迹对比图"""
    desired_positions = np.asarray(desired_positions)
    actual_positions = np.asarray(actual_positions)

    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
    axes = axes.flatten()

    for joint_idx, ax in enumerate(axes):
        ax.plot(times, desired_positions[:, joint_idx], label="desired", linewidth=2.0)
        ax.plot(times, actual_positions[:, joint_idx], label="actual", linewidth=1.5, alpha=0.85)
        ax.set_title(f"Joint {joint_idx + 1}")
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
    frequency = 0.5
    duration = 20.0
    control_rate = 500
    dt = 1.0 / control_rate
    max_torque = [21.0, 36.0, 36.0, 21.0, 10.0, 10.0]
    kp = [30.0, 50.0, 60.0, 25.0, 15.0, 10.0]
    kd = [3.0, 5.0, 6.0, 2.5, 1.5, 1.0]

    # 定义各关节角度限制（弧度）
    joint_limits = [
        [-np.pi, np.pi],
        [0, np.pi],
        [0, np.pi],
        [0, np.pi / 2],
        [-np.pi / 2, np.pi / 2],
        [-np.pi, np.pi],
    ]

    print("获取初始位置...")
    center_pos = robot.get_current_pos()
    print(f"中心位置: {center_pos}")

    joint_limits_array = np.array(joint_limits)
    lower_limits = joint_limits_array[:, 0]
    upper_limits = joint_limits_array[:, 1]

    for i, pos in enumerate(center_pos):
        if pos < lower_limits[i] or pos > upper_limits[i]:
            print(f"警告: 关节{i+1}初始位置 {pos:.3f} 超出限制范围 [{lower_limits[i]:.3f}, {upper_limits[i]:.3f}]")

    dist_to_upper = upper_limits - center_pos
    dist_to_lower = center_pos - lower_limits
    safe_amplitudes = np.minimum(dist_to_upper, dist_to_lower) * 0.8
    preset_amplitudes = np.array([0.4, 0.4, 0.4, 0.5, 0.4, 0.0])
    amplitudes = np.minimum(safe_amplitudes, preset_amplitudes)

    print(f"调整后的振幅: {amplitudes} rad")
    max_velocities = amplitudes * 2 * np.pi * frequency
    print(f"各关节最大速度: {max_velocities} rad/s")

    phase_offsets = np.zeros(robot.motor_count)

    print("\n开始正弦轨迹运动...")
    print(f"频率: {frequency} Hz, 持续时间: {duration} 秒")
    print(f"振幅: {amplitudes}")

    times = []
    desired_positions = []
    actual_positions = []

    start_time = time.time()
    step = 0

    try:
        while (time.time() - start_time) < duration:
            loop_start = time.time()
            current_time = time.time() - start_time
            omega = 2 * np.pi * frequency

            pos = center_pos + amplitudes * np.sin(omega * current_time + phase_offsets)
            vel = amplitudes * omega * np.cos(omega * current_time + phase_offsets)

            below_limit = pos < lower_limits
            above_limit = pos > upper_limits
            pos = np.clip(pos, lower_limits, upper_limits)
            vel[below_limit | above_limit] = 0.0

            # 使用 Pos_Vel 模式
            # robot.Joint_Pos_Vel(pos, vel, max_torque, iswait=False)

            # 使用 MIT 模式
            current_q = robot.get_current_pos()
            torque_ff = np.asarray(robot.get_Gravity(current_q))
            torque_ff = np.clip(torque_ff, -np.asarray(max_torque), np.asarray(max_torque))
            robot.pos_vel_tqe_kp_kd(pos, vel, torque_ff, kp, kd)

            actual_pos = robot.get_current_pos()
            times.append(current_time)
            desired_positions.append(np.asarray(pos).copy())
            actual_positions.append(np.asarray(actual_pos).copy())

            if step % 50 == 0:
                tracking_error = np.linalg.norm(actual_pos - pos)
                print(
                    f"\r时间: {current_time:.2f}s | "
                    f"关节1期望: {pos[0]:.3f} | "
                    f"关节1实际: {actual_pos[0]:.3f} | "
                    f"误差范数: {tracking_error:.4f}",
                    end=""
                )

            step += 1

            loop_time = time.time() - loop_start
            if loop_time < dt:
                time.sleep(dt - loop_time)

    except KeyboardInterrupt:
        print("\n\n轨迹被中断")

    print("\n\n返回中心位置...")
    robot.Joint_Pos_Vel(center_pos, [0.5] * robot.motor_count, [10.0] * robot.motor_count, iswait=True)
    print("运动完成")

    output_path = Path(__file__).resolve().with_name("sin_trajectory_tracking.png")
    plot_tracking_result(times, desired_positions, actual_positions, output_path)


if __name__ == "__main__":
    robot = Panthera()

    print("移动到初始位置...")
    zero_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    init_pos = [-0.3, 1.1, 1.1, 0.2, -0.3, 0.0]
    vel = [0.5] * robot.motor_count
    max_torque = [10.0] * robot.motor_count

    success = robot.Joint_Pos_Vel(zero_pos, vel, max_torque, iswait=True)
    time.sleep(3)

    success = robot.Joint_Pos_Vel(init_pos, vel, max_torque, iswait=True)
    if success:
        print("到达初始位置")
        time.sleep(1)

    try:
        main()
        success = robot.Joint_Pos_Vel(zero_pos, vel, max_torque, iswait=True)
        time.sleep(2)
    except KeyboardInterrupt:
        print("\n\n程序被中断")
        print("\n\n所有电机已停止")
    except Exception as e:
        print(f"\n错误: {e}")
