#!/usr/bin/env python3
"""
七轴正弦轨迹跟踪控制程序
前 6 个关节和第 7 个电机同时沿正弦轨迹运动。
"""
import time
import numpy as np
from Panthera_lib import Panthera


def main():
    frequency = 0.5  # Hz，正弦波频率（可调节：0.1-2.0 Hz）(频率越高速度越快)
    duration = 600.0 # 运动持续时间（秒）
    control_rate = 500 # 控制频率 Hz
    dt = 1.0 / control_rate

    joint_max_torque = np.array([21.0, 36.0, 36.0, 21.0, 10.0, 10.0], dtype=float)
    gripper_max_torque = 0.5

    joint_limits = np.array([
        [-np.pi, np.pi],
        [0.0, np.pi],
        [0.0, np.pi],
        [0.0, np.pi / 2.0],
        [-np.pi / 2.0, np.pi / 2.0],
        [-np.pi, np.pi],
    ], dtype=float)

    print("获取初始位置...")
    center_pos = robot.get_current_pos()
    gripper_current = robot.get_current_pos_gripper()
    print(f"六关节中心位置: {center_pos}")
    print(f"第7电机当前位置: {gripper_current:.3f}")

    lower_limits = joint_limits[:, 0]
    upper_limits = joint_limits[:, 1]

    dist_to_upper = upper_limits - center_pos
    dist_to_lower = center_pos - lower_limits
    safe_amplitudes = np.minimum(dist_to_upper, dist_to_lower) * 0.8
    preset_amplitudes = np.array([0.4, 0.6, 0.6, 0.8, 0.4, 0.6], dtype=float)
    amplitudes = np.minimum(safe_amplitudes, preset_amplitudes)

    gripper_lower = float(robot.gripper_limits["lower"])
    gripper_upper = float(robot.gripper_limits["upper"])
    # 第7电机按夹爪开合处理：以限位中点为中心做周期性开合。
    gripper_center = 0.5 * (gripper_lower + gripper_upper)
    gripper_amplitude = 0.4 * (gripper_upper - gripper_lower)

    print(f"六关节振幅: {amplitudes} rad")
    print(f"第7电机开合中心: {gripper_center:.3f}")
    print(f"第7电机开合振幅: {gripper_amplitude:.3f}")
    print(f"六关节最大速度: {amplitudes * 2 * np.pi * frequency} rad/s")
    print(f"第7电机最大速度: {gripper_amplitude * 2 * np.pi * frequency:.3f}")

    phase_offsets = np.zeros(robot.motor_count, dtype=float)
    gripper_phase = -np.pi / 2.0

    print("\n开始七轴正弦轨迹运动...")
    print(f"频率: {frequency} Hz, 持续时间: {duration} 秒")

    start_time = time.time()
    step = 0

    try:
        while (time.time() - start_time) < duration:
            loop_start = time.time()
            current_time = time.time() - start_time
            omega = 2.0 * np.pi * frequency

            pos = center_pos + amplitudes * np.sin(omega * current_time + phase_offsets)
            vel = amplitudes * omega * np.cos(omega * current_time + phase_offsets)

            below_limit = pos < lower_limits
            above_limit = pos > upper_limits
            pos = np.clip(pos, lower_limits, upper_limits)
            vel[below_limit | above_limit] = 0.0

            gripper_pos = gripper_center + gripper_amplitude * np.sin(omega * current_time + gripper_phase)
            gripper_vel = gripper_amplitude * omega * np.cos(omega * current_time + gripper_phase)
            if gripper_pos < gripper_lower:
                gripper_pos = gripper_lower
                gripper_vel = 0.0
            elif gripper_pos > gripper_upper:
                gripper_pos = gripper_upper
                gripper_vel = 0.0

            for i in range(robot.motor_count):
                robot.Motors[i].pos_vel_MAXtqe(pos[i], vel[i], joint_max_torque[i])
            robot.Motors[robot.gripper_id - 1].pos_vel_MAXtqe(gripper_pos, gripper_vel, gripper_max_torque)
            robot.motor_send_cmd()

            if step % 50 == 0:
                print(
                    f"\r时间: {current_time:.2f}s | "
                    f"关节6位置: {pos[5]:.3f} | "
                    f"第7电机位置: {gripper_pos:.3f}",
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
    robot.gripper_control(gripper_center, 0.5, gripper_max_torque)
    print("运动完成")
    time.sleep(1)


if __name__ == "__main__":
    robot = Panthera()

    print("移动到初始位置...")
    zero_pos = [0.0] * robot.motor_count
    init_pos = [-0.3, 1.1, 1.1, 0.8, -0.3, 0.3]
    vel = [0.5] * robot.motor_count
    max_torque = [10.0] * robot.motor_count

    robot.Joint_Pos_Vel(zero_pos, vel, max_torque, iswait=True)
    time.sleep(3)

    success = robot.Joint_Pos_Vel(init_pos, vel, max_torque, iswait=True)
    if success:
        print("到达初始位置")
        time.sleep(1)

    robot.gripper_open()
    time.sleep(1)

    try:
        main()
        robot.Joint_Pos_Vel(zero_pos, vel, max_torque, iswait=True)
        robot.gripper_close()
        time.sleep(2)
    except KeyboardInterrupt:
        print("\n\n程序被中断")
        print("\n\n所有电机已停止")
    except Exception as e:
        print(f"\n错误: {e}")
