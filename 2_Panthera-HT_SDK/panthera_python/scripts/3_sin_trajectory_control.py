#!/usr/bin/env python3
"""
正弦轨迹跟踪控制程序
机器人关节沿着正弦函数轨迹运动
"""
import time
import sys
import os
import numpy as np

# 添加上一级目录(python目录)到sys.path
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_dir)
from scripts.Panthera_lib.Panthera import Panthera 

def main():
    # 控制参数
    frequency = 0.45  # Hz，正弦波频率（可调节：0.1-2.0 Hz）(频率越高速度越快)
    duration = 600.0  # 运动持续时间（秒）
    control_rate = 500  # 控制频率 Hz
    dt = 1.0 / control_rate
    max_torque = [10.0] * robot.motor_count

    # 定义各关节角度限制（弧度）
    # 1关节：±180° = ±π, 2-3关节：0到-180° = 0到-π, 4-5关节：±90° = ±π/2, 6关节：±180° = ±π
    joint_limits = [
        [-np.pi, np.pi],        # 关节1：±180度
        [0, np.pi],             # 关节2：-180到0度
        [0, np.pi],             # 关节3：-180到0度
        [0, np.pi/2],           # 关节4：±90度
        [-np.pi/2, np.pi/2],    # 关节5：±90度
        [-np.pi, np.pi]         # 关节6：±180度
    ]
    
    # 获取初始位置作为中心位置（返回np.ndarray）
    print("获取初始位置...")
    center_pos = robot.get_current_pos()
    print(f"中心位置: {center_pos}")

    # 将关节限制转换为numpy数组以便向量化运算
    joint_limits_array = np.array(joint_limits)
    lower_limits = joint_limits_array[:, 0]
    upper_limits = joint_limits_array[:, 1]

    # 检查初始位置是否在限制范围内
    for i, pos in enumerate(center_pos):
        if pos < lower_limits[i] or pos > upper_limits[i]:
            print(f"警告: 关节{i+1}初始位置 {pos:.3f} 超出限制范围 [{lower_limits[i]:.3f}, {upper_limits[i]:.3f}]")

    # 设置各关节的振幅（弧度）- 自动调整以避免超限（向量化运算）
    dist_to_upper = upper_limits - center_pos
    dist_to_lower = center_pos - lower_limits
    safe_amplitudes = np.minimum(dist_to_upper, dist_to_lower) * 0.8
    preset_amplitudes = np.array([0.4, 0.6, 0.6, 0.5, 0.4, 0.0])
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
    
    start_time = time.time()
    step = 0
    
    try:
        while (time.time() - start_time) < duration:
            loop_start = time.time()
            current_time = time.time() - start_time
            
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
            
            robot.pos_vel_MAXtqe(pos, vel, max_torque, iswait=False)
            
            # 定期打印状态
            if step % 50 == 0:  # 每0.5秒打印一次
                print(f"\r时间: {current_time:.2f}s | "
                      f"关节1位置: {pos[0]:.3f} | "
                      f"关节2位置: {pos[1]:.3f} | "
                      f"关节3位置: {pos[2]:.3f}", end="")
            
            step += 1
            
            # 控制循环频率
            loop_time = time.time() - loop_start
            if loop_time < dt:
                time.sleep(dt - loop_time)
                
    except KeyboardInterrupt:
        print("\n\n轨迹被中断")
    
    # 返回中心位置
    print("\n\n返回中心位置...")
    robot.pos_vel_MAXtqe(center_pos, [0.5] * robot.motor_count, [10.0] * robot.motor_count, iswait=True)
    
    print("运动完成")
    time.sleep(1)

if __name__ == "__main__":
    robot = Panthera()
    
    # 先移动到安全的初始位置
    print("移动到初始位置...")
    zero_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # init_pos = [0.0, -1.0, -1.0, 0.0, 0.0, 0.0]
    init_pos = [-0.3, 1.1, 1.1, 0.2, -0.3, 0.0]
    vel = [0.5] * robot.motor_count
    max_torque = [10.0] * robot.motor_count
    
    success = robot.pos_vel_MAXtqe(zero_pos, vel, max_torque, iswait=True)
    time.sleep(3)

    success = robot.pos_vel_MAXtqe(init_pos, vel, max_torque, iswait=True)
    if success:
        print("到达初始位置")
        time.sleep(1)
    
    try:
        main()
        success = robot.pos_vel_MAXtqe(zero_pos, vel, max_torque, iswait=True)
        time.sleep(2)
    except KeyboardInterrupt:
        # robot.set_stop()
        print("\n\n程序被中断")
        print("\n\n所有电机已停止")
    except Exception as e:
        print(f"\n错误: {e}")
        # robot.set_stop()