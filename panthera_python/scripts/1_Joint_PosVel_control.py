#!/usr/bin/env python3
"""
简单的六关节机器人位置速度控制程序
直接在代码中修改目标位置数组来控制机器人
"""
import time
from Panthera_lib import Panthera

def main():
    # 发送位置控制命令（使用阻塞模式）
    print("\n发送控制命令...")
    zero_success = robot.Joint_Pos_Vel(zero_pos, vel, max_torque, iswait=True)
    print(f"执行状态0：{zero_success}")
    time.sleep(1)

    robot.Joint_Pos_Vel(pos2, vel, max_torque, iswait=True)
    robot.gripper_close()
    time.sleep(2)

    robot.Joint_Pos_Vel(pos1, vel, max_torque, iswait=True)
    robot.gripper_open()
    time.sleep(2)

    robot.Joint_Pos_Vel(pos2, vel, max_torque, iswait=True)
    robot.gripper_close()
    time.sleep(2)

    zero_success = robot.Joint_Pos_Vel(zero_pos, vel, max_torque, iswait=True)
    print(f"执行状态0：{zero_success}")
    time.sleep(2)

    # 保持位置2秒
    print("\n保持位置2秒...")
    time.sleep(2)
    # 结束后电机会自动掉电，请注意安全！！

if __name__ == "__main__":
    robot = Panthera()
    zero_pos = [0.0] * robot.motor_count
    pos1 = [0.0, 0.8, 0.8, 0.3, 0.0, 0.0] 
    pos2 = [0.0, 1.2, 1.2, 0.4, 0.0, 0.0] 
    pos3 = [0.0, 0.0, 0.0, 0.0, 0.0, 2.0] 
    vel = [0.5] * robot.motor_count      
    max_torque = [21.0, 36.0, 36.0, 21.0, 10.0, 10.0] 
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n程序被中断")
    except Exception as e:
        print(f"\n错误: {e}")
    finally:
        print("\n\n所有电机已停止")