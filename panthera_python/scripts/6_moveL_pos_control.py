#!/usr/bin/env python3
"""
moveL 位置控制示例

演示如何使用 moveL 实现丝滑的末端运动
"""

import sys
import os
import numpy as np
import time
from Panthera_lib import Panthera


def main():
    print("="*60)
    print("MoveIt 风格笛卡尔控制示例")
    print("="*60)

    # 1. 初始化机械臂
    print("\n初始化机械臂...")
    robot = Panthera()

    # 2. 移动到初始位置
    print("\n移动到初始位置...")
    ik_pos1 = [0.24, 0.0, 0.1]
    ik_rot1 = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
    zero_pos = [0.0] * robot.motor_count
    vel = [0.5] * robot.motor_count
    robot.Joint_Pos_Vel(zero_pos, vel, iswait=True)

    pos1 = robot.inverse_kinematics(ik_pos1, ik_rot1, robot.get_current_pos())
    if pos1 is not None:
        robot.moveJ(pos1, duration=3.0, iswait=True)

    # 3. 获取当前位姿
    current_fk = robot.forward_kinematics()
    current_pos = np.array(current_fk['position'])
    current_rot = current_fk['rotation']

    print(f"\n当前位置: [{current_pos[0]:.4f}, {current_pos[1]:.4f}, {current_pos[2]:.4f}] m")

    # ========== 示例1: 直线运动（指定时间） ==========
    print("\n" + "="*60)
    print("示例1: 沿X轴移动10cm（指定时间2秒）")
    print("="*60)

    target_pos_1 = current_pos + np.array([0.1, 0.0, 0.0])

    success = robot.moveL(
        target_position=target_pos_1,
        target_rotation=current_rot,
        duration=2.0,  # 指定2秒
        use_spline=True  # 使用样条平滑
    )

    if not success:
        print("示例1失败")
        return

    time.sleep(1)

    # ========== 示例2: 直线运动（指定时间） ==========
    print("\n" + "="*60)
    print("示例2: 沿Y轴移动8cm（指定时间1.5秒）")
    print("="*60)

    current_fk = robot.forward_kinematics()
    current_pos = np.array(current_fk['position'])
    current_rot = current_fk['rotation']

    target_pos_2 = current_pos + np.array([0.0, 0.08, 0.0])

    success = robot.moveL(
        target_position=target_pos_2,
        target_rotation=current_rot,
        duration=1.5,  # 指定1.5秒
        use_spline=True
    )

    if not success:
        print("示例2失败")
        return

    time.sleep(0.5)

    # ========== 示例3: 对角线运动 ==========
    print("\n" + "="*60)
    print("示例3: 对角线运动 (X:-4cm, Y:-4cm, Z:+4cm)")
    print("="*60)

    current_fk = robot.forward_kinematics()
    current_pos = np.array(current_fk['position'])
    current_rot = current_fk['rotation']

    target_pos_3 = current_pos + np.array([-0.04, -0.04, 0.04])

    success = robot.moveL(
        target_position=target_pos_3,
        target_rotation=current_rot,
        duration=2.0,  # 指定2秒
        use_spline=True
    )

    if not success:
        print("示例3失败")
        return

    time.sleep(0.5)

    # ========== 示例4: 多段路径（类似 MoveIt 的 waypoints） ==========
    print("\n" + "="*60)
    print("示例4: 多段路径（正方形）")
    print("="*60)

    current_fk = robot.forward_kinematics()
    start_pos = np.array(current_fk['position'])
    start_rot = current_fk['rotation']

    # 定义正方形的4个角点
    side_length = 0.07  # 7cm
    waypoints = [
        {'position': start_pos, 'rotation': start_rot},
        {'position': start_pos - np.array([side_length, 0, 0]), 'rotation': start_rot},
        {'position': start_pos - np.array([side_length, side_length, 0]), 'rotation': start_rot},
        {'position': start_pos - np.array([0, side_length, 0]), 'rotation': start_rot},
        {'position': start_pos, 'rotation': start_rot},  # 回到起点
    ]

    # 计算完整路径
    print("  计算正方形路径...")
    joint_trajectory, fraction = robot.compute_cartesian_path(waypoints)

    if joint_trajectory is None or fraction < 0.99:
        print(f"  路径规划失败或不完整 (fraction={fraction*100:.1f}%)")
        return

    print(f"  ✓ 路径规划完成: {len(joint_trajectory)} 个点")

    # 时间参数化
    timestamps = robot.compute_time_parameterization(joint_trajectory, duration=3.0)
    print(f"  ✓ 总时间: {timestamps[-1]:.2f}s")

    # 样条平滑
    joint_trajectory, timestamps, velocities = robot.smooth_trajectory_spline(
        joint_trajectory, timestamps
    )
    print(f"  ✓ 样条平滑完成: {len(joint_trajectory)} 个点")

    # 执行
    print("  开始执行...")
    success = robot._execute_trajectory(
        joint_trajectory, timestamps, velocities, max_tqu=None
    )

    if success:
        print("  ✓ 正方形轨迹执行成功")
    else:
        print("  ✗ 正方形轨迹执行失败")

    robot.moveJ(zero_pos, 3.0, iswait=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"\n发生错误: {e}")
        import traceback
        traceback.print_exc()
