#!/usr/bin/env python3
"""
关节阻抗控制+摩擦补偿
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
    f = robot.get_friction_compensation(vel_current, Fc, Fv, vel_threshold)
    tor = tor_impedance + G
    # 力矩限幅（基于电机规格）
    tau_limit = np.array([10.0, 20.0, 20.0, 10.0, 5.0, 5.0])
    tor = np.clip(tor, -tau_limit, tau_limit)
    robot.pos_vel_tqe_kp_kd(zero_pos, zero_vel, tor, zero_kp, zero_kd)
    print(f"阻抗力矩：{[f'{t:.3f}' for t in tor_impedance]}, \n重力补偿力矩：{[f'{t:.3f}' for t in G]}, \n总力矩：{[f'{t:.3f}' for t in tor]}")
    time.sleep(0.002)
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

    # ==================== 摩擦参数配置 ====================
    # 注意：这些参数需要根据实际机器人进行辨识和调整

    # 库伦摩擦系数 Fc (Nm) - 恒定摩擦力，与速度大小无关
    # 建议初始值：较小的关节用较小值，较大的关节用较大值
    # 参数辨识方法：让关节以极低速度匀速运动，测量所需的最小恒定力矩
    Fc = np.array([
        0.20,  # 关节1
        0.15,  # 关节2
        0.15,  # 关节3
        0.15,  # 关节4
        0.04, # 关节5
        0.04  # 关节6
    ])

    # 粘性摩擦系数 Fv (Nm·s/rad) - 线性速度相关摩擦系数
    # 建议初始值：通常比库伦摩擦小一个数量级
    # 参数辨识方法：让关节以不同速度匀速运动，测量力矩-速度曲线的斜率
    Fv = np.array([
        0.06,  # 关节1
        0.06,   # 关节2
        0.06,   # 关节3
        0.03,  # 关节4
        0.02,  # 关节5
        0.02   # 关节6
    ])

    # 速度阈值 (rad/s) - 低于此速度时不使用库伦摩擦项
    # 建议值：0.01-0.05 rad/s
    vel_threshold = 0.02

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