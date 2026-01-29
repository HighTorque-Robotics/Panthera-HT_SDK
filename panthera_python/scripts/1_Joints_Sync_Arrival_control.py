#!/usr/bin/env python3
"""
简单的��关节机器人位置时间控制程序
通过指定运动时间，自动计算速度实现所有关节同时到达目标位置
直接在代码中修改目标位置数组和运动时间来控制机器人
"""
import time
from Panthera_lib import Panthera

def main():
    # 发送位置时间控制命令（使用阻塞模式）
    print("\n发送控制命令...")
    zero_success = robot.Joints_Sync_Arrival(zero_pos, duration=2.0, max_tqu=max_torque, iswait=True)
    print(f"执行状态0：{zero_success}")
    time.sleep(1)

    # 运动到位置1，使用3秒到达
    robot.Joints_Sync_Arrival(pos1, duration=3.0, max_tqu=max_torque, iswait=True)
    robot.gripper_close()
    time.sleep(2)

    # 运动到位置2，使用2.5秒到达
    robot.Joints_Sync_Arrival(pos2, duration=2.5, max_tqu=max_torque, iswait=True)
    robot.gripper_open()
    time.sleep(2)

    # 运动到位置1，使用3秒到达
    robot.Joints_Sync_Arrival(pos1, duration=3.0, max_tqu=max_torque, iswait=True)
    robot.gripper_close()
    time.sleep(2)

    # 回到零位，使用2秒到达
    zero_success = robot.Joints_Sync_Arrival(zero_pos, duration=2.0, max_tqu=max_torque, iswait=True)
    print(f"执行状态0：{zero_success}")
    time.sleep(2)

    # 保持位置2秒
    print("\n保持位置2秒...")
    time.sleep(2)
    # 结束后电机会自动掉电，请注意安全！！

if __name__ == "__main__":
    robot = Panthera()
    # 定义目标位置（弧度）
    zero_pos = [0.0] * robot.motor_count
    pos1 = [0.0, 0.8, 0.8, 0.3, 0.0, 0.0]
    pos2 = [0.0, 1.2, 1.2, 0.4, 0.0, 0.0]
    pos3 = [0.0, 0.0, 0.0, 0.0, 0.0, 2.0]

    # 定义最大力矩（Nm）
    max_torque = [21.0, 36.0, 36.0, 21.0, 10.0, 10.0]

    try:
        main()
    except KeyboardInterrupt:
        print("\n\n程序被中断")
    except Exception as e:
        print(f"\n错误: {e}")
    finally:
        print("\n\n所有电机已停止")
