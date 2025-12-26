#!/usr/bin/env python3
"""
获取并打印机械臂关节角度信息
实时显示6个关节和夹爪的当前状态
"""

import time
import sys
import os

# 添加上一级目录(python目录)到sys.path
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_dir)
from scripts.Panthera_lib.Panthera import Panthera

def print_robot_state(robot):
    """打印机器人状态信息"""
    # 获取关节角度
    # 利用运控模式发送控制帧以读取电机反馈状态
    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    positions = robot.get_current_pos()
    velocities = robot.get_current_vel()

    # 获取夹爪状态
    gripper_state = robot.Motors[6].get_current_motor_state()
    
    print("\n" + "="*50)
    print("机械臂状态信息")
    print("="*50)
    
    # 打印6个关节信息
    for i in range(robot.motor_count):
        print(f"关节{i+1}: 位置={positions[i]:7.3f} rad, 速度={velocities[i]:7.3f} rad/s")
    
    # 打印夹爪信息
    print(f"夹爪:   位置={gripper_state.position:7.3f} rad, 速度={gripper_state.velocity:7.3f} rad/s")

def main():
    robot = Panthera()
    
    try:
        robot.set_reset_zero()
        robot.motor_send_cmd()
        time.sleep(1)  # 每0.5秒更新一次
        while True:
            print_robot_state(robot)
            time.sleep(0.5)  # 每0.5秒更新一次
            
    except KeyboardInterrupt:
        print("\n\n程序被中断")

if __name__ == "__main__":
    main()