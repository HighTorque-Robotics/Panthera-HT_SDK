#!/usr/bin/env python3
"""
重力补偿程序
仅补偿重力项
"""
import time
import numpy as np
from Panthera_lib import Panthera

def main():
    tor = robot.get_Gravity()  # 调用函数获取重力补偿力矩
    # 力矩限幅（基于电机规格）
    tau_limit = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0])
    tor = np.clip(tor, -tau_limit, tau_limit)
    robot.pos_vel_tqe_kp_kd(zero_pos, zero_vel, tor, zero_kp, zero_kd)
    robot.gripper_control_MIT(0,0,0,0,0)
    print(f"重力补偿力矩：",tor)
    time.sleep(0.002)
    #结束后电机会自动掉电，请注意安全！！

if __name__ == "__main__":
    robot = Panthera()
    # 创建零位置和零速度数组
    zero_pos = [0.0] * robot.motor_count
    zero_vel = [0.0] * robot.motor_count
    zero_kp = [0.0] * robot.motor_count
    zero_kd = [0.0] * robot.motor_count
    zero_tor = [0.0] * robot.motor_count #调试使用
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