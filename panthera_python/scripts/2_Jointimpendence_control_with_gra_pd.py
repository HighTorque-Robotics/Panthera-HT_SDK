#!/usr/bin/env python3
"""
关节阻抗控制（PD刚度阻尼项力矩+重力力矩前馈）
其实相当于六个电机PD控制加上前馈力矩
（可以对比1_PD_control.py的效果）
"""
import time
import numpy as np
from Panthera_lib import Panthera

def main():
    # 计算阻抗控制输出力矩
    q_current = robot.get_current_pos()
    vel_current = robot.get_current_vel()
    tor_impedance = K * (q_des - q_current) + B * (v_des - vel_current)
    # 力矩计算（加上重力补偿前馈力矩）
    G = robot.get_Gravity() 
    tor = tor_impedance + G
    # 力矩限幅（基于电机规格）
    tau_limit = np.array([10.0, 20.0, 20.0, 10.0, 5.0, 5.0])
    tor = np.clip(tor, -tau_limit, tau_limit)
    robot.pos_vel_tqe_kp_kd(zero_pos, zero_vel, tor, zero_kp, zero_kd)
    print(f"阻抗力矩：{[f'{t:.3f}' for t in tor_impedance]}, \n重力补偿力矩：{[f'{t:.3f}' for t in G]}, \n总力矩：{[f'{t:.3f}' for t in tor]}")
    time.sleep(0.005)
    #结束后电机会自动掉电，请注意安全！！

if __name__ == "__main__":
    robot = Panthera()
    # 刚度系数和阻尼系数
    K = np.array([4.0, 10.0, 10.0, 2.0, 2.0, 1.0])
    B = np.array([0.5, 0.8, 0.8, 0.2, 0.2, 0.1])
    # 都为零则为重力补偿模式
    # K = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # B = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    q_des = np.array([0.0, 0.7, 0.7, -0.1, 0.0, 0.0]  )  # 期望目标位置
    # q_des = np.zeros(6)  # 期望目标位置
    v_des = np.zeros(6) #期望目标速度为0
    # 创建零位置和零速度数组
    zero_kp = [0.0] * robot.motor_count
    zero_kd = [0.0] * robot.motor_count
    zero_pos = [0.0]*6
    zero_vel = [0.0]*6
    q = np.array([])
    vel = np.array([])
    try:
        while(1):
            main()
    except KeyboardInterrupt:
        # 不加这行电机在程序停止后也会掉电
        # robot.set_stop()
        print("\n\n程序被中断")
        print("\n\n所有电机已停止")
    except Exception as e:
        print(f"\n错误: {e}")