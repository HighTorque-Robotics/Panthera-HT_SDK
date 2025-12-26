#!/usr/bin/env python3
"""
重力补偿并输出正运动学结果程序
功能：
1. 实现重力补偿控制
2. 实时输出机器人末端位置和旋转矩阵
"""
import time
import sys
import os
import numpy as np
# 添加上一级目录(python目录)到sys.path
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_dir)
from scripts.Panthera_lib.Panthera import Panthera

def rotation_matrix_to_euler(R):
    """将旋转矩阵转换为欧拉角（ZYX顺序，单位：度）"""
    sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    
    singular = sy < 1e-6
    
    if not singular:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
    
    return np.degrees([x, y, z])

def print_matrix(matrix, title, precision=3):
    """格式化打印矩阵"""
    print(f"\n{title}:")
    for row in matrix:
        print("  [" + "  ".join([f"{val:8.{precision}f}" for val in row]) + "]")

def main():
    # 获取当前关节角度
    current_angles = robot.get_current_pos()
    
    # 计算重力补偿力矩
    gravity_torque = robot.get_Gravity(current_angles)
    
    # 力矩限幅（基于电机规格）
    tau_limit = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0])
    gravity_torque = np.clip(gravity_torque, -tau_limit, tau_limit)
    
    # 应用重力补偿控制
    zero_pos = [0.0] * robot.motor_count
    zero_vel = [0.0] * robot.motor_count
    zero_kp = [0.0] * robot.motor_count
    zero_kd = [0.0] * robot.motor_count
    
    robot.pos_vel_tqe_kp_kd(zero_pos, zero_vel, gravity_torque, zero_kp, zero_kd)
    
    # 计算正运动学
    fk = robot.forward_kinematics(current_angles)
    
    # 输出结果
    print("\n" + "="*80)
    print("重力补偿控制 + 正运动学结果")
    print("="*80)
    
    # 显示关节角度
    joint_angles_deg = [np.degrees(angle) for angle in current_angles]
    print(f"关节角度 (度): {[f'{a:7.2f}' for a in joint_angles_deg]}")
    
    # 显示重力补偿力矩
    print(f"重力补偿力矩 (Nm): {[f'{t:7.2f}' for t in gravity_torque]}")
    
    if fk:
        # 显示末端位置
        pos = fk['position']
        print(f"\n末端位置 (m): x={pos[0]:8.4f}, y={pos[1]:8.4f}, z={pos[2]:8.4f}")
        
        # 显示旋转矩阵
        rotation_matrix = fk['rotation']
        print_matrix(rotation_matrix, "旋转矩阵 (R)")
        
        # 显示欧拉角
        euler_angles = rotation_matrix_to_euler(rotation_matrix)
        print(f"\n欧拉角 (度): Roll={euler_angles[0]:7.2f}, Pitch={euler_angles[1]:7.2f}, Yaw={euler_angles[2]:7.2f}")
        
        # 显示4x4变换矩阵
        print_matrix(fk['transform'], "4x4变换矩阵 (T)", precision=4)
    
    time.sleep(0.002) 

if __name__ == "__main__":
    robot = Panthera()
    
    try:
        print("开始重力补偿控制，同时输出正运动学结果...")
        print("按 Ctrl+C 停止程序")
        
        while True:
            main()
            
    except KeyboardInterrupt:
        # 不加这行电机在程序停止后也会掉电
        # robot.set_stop()
        print("\n\n程序被中断")
        print("所有电机已停止")
    except Exception as e:
        # robot.set_stop()
        print(f"\n错误: {e}")
        print("所有电机已停止")