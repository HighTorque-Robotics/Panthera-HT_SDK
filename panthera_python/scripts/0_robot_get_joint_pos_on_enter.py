#!/usr/bin/env python3
"""
按下 Enter 时读取并显示当前6个关节的位置。
按 Ctrl+C 结束程序。
"""
from Panthera_lib import Panthera


def get_joint_positions(robot):
    """刷新一次状态并获取当前关节位置。"""
    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    return robot.get_current_pos()


def print_joint_positions(positions):
    """打印当前关节位置。"""
    print("\n当前关节位置:")
    for i, pos in enumerate(positions, start=1):
        print(f"关节{i}: {pos:7.3f} rad")


def main():
    robot = Panthera()
    print("按下 Enter 显示当前关节位置，按 Ctrl+C 退出。")

    try:
        while True:
            input()
            positions = get_joint_positions(robot)
            print_joint_positions(positions)
    except KeyboardInterrupt:
        print("\n程序已结束")


if __name__ == "__main__":
    main()
