#!/usr/bin/env python3
"""
简单的六关节机器人PD控制程序
直接在代码中修改目标位置数组来控制机器人
"""
import time
import sys
import os
# 添加上一级目录(python目录)到sys.path
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_dir)
from scripts.Panthera_lib.Panthera import Panthera

if __name__ == "__main__":
    robot = Panthera()
    zero_pos = [0.0] * robot.motor_count
    zero_vel = [0.0] * robot.motor_count
    zero_tqe = [0.0] * robot.motor_count
    pos1 = [0.0, 0.7, 0.7, -0.1, 0.0, 0.0]
    kp = [4.0, 10.0, 10.0, 2.0, 2.0, 1.0]
    kd = [0.5, 0.8, 0.8, 0.2, 0.2, 0.1]
    vel = [0.3] * robot.motor_count      
    max_torque = [21.0, 36.0, 36.0, 21.0, 10.0, 10.0] 
    try:
        while(1):
            robot.pos_vel_tqe_kp_kd(pos1, zero_vel, zero_tqe, kp, kd)

            positions = robot.get_current_pos()
            velocities = robot.get_current_vel()
            torque = robot.get_current_torque()
            # 打印6个关节信息
            for i in range(robot.motor_count):
                print(f"关节{i+1}: 位置={positions[i]:7.3f} rad, 速度={velocities[i]:7.3f} rad/s, 力矩={torque[i]:7.3f}")
            print("-" * 60)

            time.sleep(1)
            
    except KeyboardInterrupt:
        # 不加这行电机在程序停止后也会掉电
        # robot.set_stop()
        print("\n\n程序被中断")
        print("\n\n所有电机已停止")
    except Exception as e:
        print(f"\n错误: {e}")