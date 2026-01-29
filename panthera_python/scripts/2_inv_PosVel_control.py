#!/usr/bin/env python3
"""
简单的逆解运动程序
设定末端位姿后解算并发送关节执行
"""
import time
import numpy as np
from Panthera_lib import Panthera

def main():
    # 发送位置控制命令
    print("\n发送控制命令...")

    pos1 = robot.inverse_kinematics(ik_pos1, ik_rot2, robot.get_current_pos())
    # 在逆解执行前尽量检查逆解是否收敛,否则未收敛时程序会直接退出掉电
    if(pos1!=None):
        issuccess1 = robot.Joints_Sync_Arrival(pos1, duration=3.0, max_tqu = max_torque, iswait=True)
        print(f"执行状态1：{issuccess1}")
        time.sleep(3)

    pos2 = robot.inverse_kinematics(ik_pos2, ik_rot2, robot.get_current_pos())
    if(pos2!=None):
        issuccess2 = robot.Joints_Sync_Arrival(pos2, duration=3.0, max_tqu = max_torque, iswait=True)
        print(f"执行状态1：{issuccess2}")
        time.sleep(3)

    pos3 = robot.inverse_kinematics(ik_pos3, ik_rot2, robot.get_current_pos())
    if(pos3!=None):
        issuccess3 = robot.Joints_Sync_Arrival(pos3, duration=3.0, max_tqu = max_torque, iswait=True)
        print(f"执行状态1：{issuccess3}")
        time.sleep(3)

    issuccess3 = robot.Joints_Sync_Arrival(zero_pos, duration=3.0, max_tqu = max_torque, iswait=True)
    print(f"执行状态3：{issuccess3}")
    # 保持位置2秒
    print("\n保持位置2秒...")
    time.sleep(2)
    #结束后电机会自动掉电，请注意安全！！

if __name__ == "__main__":
    robot = Panthera()
    zero_pos = [0.0] * robot.motor_count
    vel = [0.5] * robot.motor_count      
    max_torque = [21.0, 36.0, 36.0, 21.0, 10.0, 10.0]  
    ik_pos1 = [0.2, 0.1, 0.18]
    ik_pos2 = [-0.12, 0.15, 0.13]
    # 提供一个超限的位置做例子
    ik_pos3 = [0.74, 0.0, 0.2] 
    
    # 机械臂零位时，所有坐标系都为同一个方向
    # 此时设定目标末端姿态与底座一致
    ik_rot1 = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    ik_rot2 = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])

    try:
        robot.Joint_Pos_Vel(zero_pos, vel, max_torque, iswait=True)
        main()
    except KeyboardInterrupt:
        print("\n\n程序被中断")
        print("\n\n所有电机已停止")
    except Exception as e:
        print(f"\n错误: {e}")