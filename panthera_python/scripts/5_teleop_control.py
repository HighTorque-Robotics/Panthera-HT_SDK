#!/usr/bin/env python3
"""
主从臂遥操作程序
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
    # ******臂控制******* 
    # 获取主臂位置速度作为从臂同步信息
    Leader_positions = Leader.get_current_pos()
    Leader_velocity = Leader.get_current_vel()
    Follower_velocity = Follower.get_current_vel()
    # 获取从臂力矩计算反馈力矩
    Follower_torque = Follower.get_current_torque()
    # 计算重力力矩
    Leader_gra = Leader.get_Gravity()
    Follower_gra = Follower.get_Gravity()  
    # 计算从臂受到的外力
    tor_diff = np.array(Follower_torque) - np.array(Follower_gra)
    # 对每个元素单独判断，小于阈值的设为 0
    tor_diff[np.abs(tor_diff) < tor_threshold] = 0
    # 主臂力矩
     # 力反馈模式：
    # Leader_tor = np.array(Leader_gra) - tor_diff*0.8 + Leader.get_friction_compensation(Leader_velocity, Fc, Fv, vel_threshold) 
     # 无力反馈模式（更丝滑）：
    Leader_tor = np.array(Leader_gra) + Leader.get_friction_compensation(Leader_velocity, Fc, Fv, vel_threshold) 
    # 从臂力矩
    Follower_tor = np.array(Follower_gra) + Follower.get_friction_compensation(Follower_velocity, Fc, Fv, vel_threshold) 
    # 力矩限幅（基于电机规格）
    tau_limit = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0])
    Leader_tor = np.clip(Leader_tor, -tau_limit, tau_limit)
    Follower_tor = np.clip(Follower_tor, -tau_limit, tau_limit)
    # 运行控制
    Leader.pos_vel_tqe_kp_kd(zero_pos, zero_vel, Leader_tor, zero_kp, zero_kd)
    Follower.pos_vel_tqe_kp_kd(Leader_positions, Leader_velocity, Follower_tor, kp, kd)

    # *******夹爪控制*******
    Leader_gripper_positions = Leader.get_current_pos_gripper()
    Leader_gripper_velocity = Leader.get_current_vel_gripper()
    Follower_gripper = Follower.get_current_state_gripper()
    gripper_torque = Follower.get_friction_compensation(Leader_gripper_velocity, 0.06, 0.0, 0.15) - Follower_gripper.torque*0.5
    tor_diff[np.abs(gripper_torque) < 0.2] = 0
    Leader.gripper_control_MIT(1.5, 0, gripper_torque, 0.2, 0.02)
    Follower.gripper_control_MIT(Leader_gripper_positions, Leader_gripper_velocity, 0, gripper_kp, gripper_kd)

    # 打印6个关节信息
    for i in range(Leader.motor_count):
        print(f"关节{i+1}: 位置={Leader_positions[i]:7.3f} rad, 速度={Leader_velocity[i]:7.3f} rad/s")
    print(f"反馈力矩：",tor_diff)
    print(f"夹爪力矩: {gripper_torque:7.3f} Nm")
    print('-' * 40)

    time.sleep(0.001)
    #结束后电机会自动掉电，请注意安全！！

if __name__ == "__main__":
    # 创建机器人实例
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, "../robot_param/Leader.yaml")
    Leader = Panthera(config_path)
    config_path = os.path.join(script_dir, "../robot_param/Follower.yaml")
    Follower = Panthera(config_path)
    # 创建零位置和零速度数组
    zero_pos = [0.0] * Leader.motor_count
    zero_vel = [0.0] * Leader.motor_count
    zero_kp = [0.0] * Leader.motor_count
    zero_kd = [0.0] * Leader.motor_count
    kp = [10.0, 21.0, 21.0, 16.0, 13.0, 1.0]
    kd = [1.0, 2.0, 2.0, 0.9, 0.8, 0.1]
    gripper_kp = 4.0
    gripper_kd = 0.4
    # 库伦摩擦系数 Fc (Nm) - 恒定摩擦力，与速度大小无关
    Fc = np.array([0.15, 0.12, 0.12, 0.12, 0.04, 0.04])
    # 粘性摩擦系数 Fv (Nm·s/rad) - 线性速度相关摩擦系数
    Fv = np.array([
        0.05,  # 关节1
        0.05,   # 关节2
        0.05,   # 关节3
        0.03,  # 关节4
        0.02,  # 关节5
        0.02   # 关节6
    ])
    # 速度阈值 (rad/s) - 低于此速度时不使用库伦摩擦项
    # 建议值：0.01-0.05 rad/s
    vel_threshold = 0.02
    tor_threshold = np.array([0.5, 1.0, 1.0, 0.5, 0.3, 0.3])

    try:
        while(1):
            main()
    except KeyboardInterrupt:
        print("\n\n程序被中断")
        print("\n\n所有电机已停止")
    except Exception as e:
        print(f"\n错误: {e}")