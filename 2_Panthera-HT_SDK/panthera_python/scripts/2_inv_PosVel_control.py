#!/usr/bin/env python3
"""
简单的逆解运动程序
设定末端位姿后解算并发送关节执行
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
    # 发送位置控制命令
    print("\n发送控制命令...")

    pos1 = robot.inverse_kinematics(ik_pos1, ik_rot, robot.get_current_pos())
    # 在逆解执行前尽量检查逆解是否收敛,否则未收敛时程序会直接退出掉电
    if(pos1!=None):
        issuccess1 = robot.pos_vel_MAXtqe(pos1, vel, max_torque, iswait=True)
        print(f"执行状态1：{issuccess1}")
        time.sleep(3)

    pos2 = robot.inverse_kinematics(ik_pos2, ik_rot, robot.get_current_pos())
    if(pos2!=None):
        issuccess2 = robot.pos_vel_MAXtqe(pos2, vel, max_torque, iswait=True)
        print(f"执行状态1：{issuccess2}")
        time.sleep(3)

    pos3 = robot.inverse_kinematics(ik_pos3, ik_rot, robot.get_current_pos())
    if(pos3!=None):
        issuccess3 = robot.pos_vel_MAXtqe(pos3, vel, max_torque, iswait=True)
        print(f"执行状态1：{issuccess3}")
        time.sleep(3)

    issuccess3 = robot.pos_vel_MAXtqe(zero_pos, vel, max_torque, iswait=True)
    print(f"执行状态3：{issuccess3}")
    # 保持位置2秒
    print("\n保持位置2秒...")
    time.sleep(2)
    #结束后电机会自动掉电，请注意安全！！

if __name__ == "__main__":
    robot = Panthera()
    zero_pos = [0.0] * robot.motor_count
    vel = [0.5] * robot.motor_count      
    max_torque = [10.0] * robot.motor_count   
    ik_pos1 = [0.3+0.138, 0.0, 0.3]
    ik_pos2 = [0.5+0.138, 0.0, 0.3]
    # 提供一个超限的位置做例子
    ik_pos3 = [0.74, 0.0, 0.2] 
    
    # 机械臂零位时，所有坐标系都为同一个方向
    # 此时设定目标末端姿态与底座一致
    ik_rot = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    try:
        main()
    except KeyboardInterrupt:
        # 不加这行电机在程序停止后也会掉电
        # robot.set_stop()
        print("\n\n程序被中断")
        print("\n\n所有电机已停止")
    except Exception as e:
        print(f"\n错误: {e}")