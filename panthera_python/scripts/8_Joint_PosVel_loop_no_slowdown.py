#!/usr/bin/env python3
"""
六关节机器人位置速度循环控制程序（固定速度版）
启动后在零位和 pos_A 之间往返循环。
按下 Ctrl+C 时，机械臂先移动到 pos_B，再返回零位，然后结束程序。
"""
import time
from Panthera_lib import Panthera


def move_and_report(target_pos, target_vel, label, pause=1.0):
    """使用固定速度发送 Joint_Pos_Vel 控制命令。"""
    print(f"\n发送控制命令: {label}")
    success = robot.Joint_Pos_Vel(target_pos, target_vel, max_torque, iswait=True)
    print(f"{label} 执行状态: {success}")
    if not success:
        raise RuntimeError(f"{label} 执行失败")
    time.sleep(pause)


def main():
    print("开始循环运动，请注意安全...")
    while True:
        move_and_report(pos_A, loop_vel, "移动到 pos_A")
        move_and_report(zero_pos, loop_vel, "返回零位")
        move_and_report(pos_A, loop_vel, "再次移动到 pos_A")


def safe_exit():
    """Ctrl+C 后的安全退出动作。"""
    print("\n检测到 Ctrl+C，开始执行安全退出流程...")
    move_and_report(pos_B, exit_vel, "移动到 pos_B", pause=1.0)
    move_and_report(zero_pos, exit_vel, "返回零位", pause=0.5)


if __name__ == "__main__":
    robot = Panthera()

    zero_pos = [0.0] * robot.motor_count
    pos_A = [0.0, 1.2, 0.6, -0.9, 0.0, 0.0]
    pos_B = [0.0, 0.2, 0.2, 0.0, 0.0, 0.0]

    loop_vel = [3.2] * robot.motor_count
    exit_vel = [1.2] * robot.motor_count
    max_torque = [21.0, 36.0, 36.0, 21.0, 10.0, 10.0]

    try:
        main()
    except KeyboardInterrupt:
        try:
            safe_exit()
        except Exception as exit_error:
            print(f"\n安全退出过程出错: {exit_error}")
        print("\n程序已停止")
    except Exception as e:
        print(f"\n错误: {e}")
    finally:
        print("\n所有电机已停止")
