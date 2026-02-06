#!/usr/bin/env python3
"""
moveL 姿态变换示例

演示如何使用 moveL 保持末端位置不变，只改变末端姿态
展示 SLERP（球面线性插值）实现平滑的姿态过渡
"""

import sys
import os
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
from Panthera_lib import Panthera

def print_orientation_info(rotation_matrix, label="当前姿态"):
    """打印姿态信息（欧拉角）"""
    rot = R.from_matrix(rotation_matrix)
    euler = rot.as_euler('xyz', degrees=True)
    print(f"{label}:")
    print(f"  Roll:  {euler[0]:7.2f}°")
    print(f"  Pitch: {euler[1]:7.2f}°")
    print(f"  Yaw:   {euler[2]:7.2f}°")


def main():
    print("="*60)
    print("moveL 姿态变换示例")
    print("保持末端位置不变，只改变姿态")
    print("="*60)

    # 1. 初始化机械臂
    print("\n初始化机械臂...")
    robot = Panthera()

    # 2. 移动到初始位置
    print("\n移动到初始位置...")
    zero_pos = [0.0] * robot.motor_count
    vel = [0.5] * robot.motor_count
    robot.Joint_Pos_Vel(zero_pos, vel, iswait=True)
    time.sleep(1)

    # 3. 移动到一个合适的工作位置
    print("\n移动到工作位置...")
    # 定义一个初始位姿
    target_pos = [0.32, 0.0, 0.25]  # 末端位置 (m)
    target_rot = robot.rotation_matrix_from_euler(0, 0, 0)  # 初始姿态：无旋转

    # 使用逆运动学求解
    joint_pos = robot.inverse_kinematics(target_pos, target_rot, robot.get_current_pos())
    if joint_pos is not None:
        robot.moveJ(joint_pos, duration=3.0, iswait=True)
        time.sleep(1)
    else:
        print("初始位置逆解失败！")
        return

    # 4. 获取当前位姿
    current_fk = robot.forward_kinematics()
    current_pos = np.array(current_fk['position'])
    current_rot = current_fk['rotation']

    print(f"\n当前末端位置: [{current_pos[0]:.4f}, {current_pos[1]:.4f}, {current_pos[2]:.4f}] m")
    print_orientation_info(current_rot, "当前姿态")


    # ========== 示例1: 绕 Z 轴旋转 45° ==========
    print("\n" + "="*60)
    print("示例2: 保持位置不变，绕 Z 轴旋转 45°")
    print("="*60)

    # 保持位置不变，只改变姿态
    target_rot_1 = robot.rotation_matrix_from_euler(0, 0, np.pi/4)  # 绕 Z 轴旋转 45°
    print_orientation_info(target_rot_1, "目标姿态")

    success = robot.moveL(
        target_position=current_pos,  # 位置不变
        target_rotation=target_rot_1,  # 姿态改变
        duration=3.0,
        use_spline=True
    )

    if not success:
        print("示例1失败")
        return

    time.sleep(1)

    # ========== 示例2: 绕 Y 轴旋转 45° ==========
    print("\n" + "="*60)
    print("示例2: 保持位置不变，绕 Y 轴旋转 45°")
    print("="*60)

    # 获取当前姿态
    current_fk = robot.forward_kinematics()
    current_rot = current_fk['rotation']

    # 在当前姿态基础上，绕 Y 轴旋转 45°
    additional_rot = robot.rotation_matrix_from_euler(0, np.pi/4, 0)
    # 右乘，使用的是末端坐标系旋转，观察机械臂末端可以看出明显效果（绕着末端坐标系的y轴旋转了45度）
    target_rot_2 = current_rot @ additional_rot
    print_orientation_info(target_rot_2, "目标姿态")

    success = robot.moveL(
        target_position=current_pos,  # 位置不变
        target_rotation=target_rot_2,  # 姿态改变
        duration=3.0,
        use_spline=True
    )

    if not success:
        print("示例2失败")
        return

    time.sleep(1)


    # ========== 示例3: 绕 X 轴旋转 -45° ==========
    print("\n" + "="*60)
    print("示例3: 保持位置不变，绕 X 轴旋转 -45°")
    print("="*60)

    # 获取当前姿态
    current_fk = robot.forward_kinematics()
    current_rot = current_fk['rotation']

    # 在当前姿态基础上，绕 X 轴旋转 -45°
    additional_rot = robot.rotation_matrix_from_euler(-np.pi/4, 0, 0)
    # 右乘，使用的是末端坐标系旋转，观察机械臂末端可以看出明显效果（绕着末端坐标系的x轴旋转了-45度）
    target_rot_3 = current_rot @ additional_rot
    print_orientation_info(target_rot_3, "目标姿态")

    success = robot.moveL(
        target_position=current_pos,  # 位置不变
        target_rotation=target_rot_3,  # 姿态改变
        duration=3.0,
        use_spline=True
    )

    if not success:
        print("示例3失败")
        return

    time.sleep(1)

    # ========== 示例4: 回到初始姿态 ==========
    print("\n" + "="*60)
    print("示例4: 回到初始姿态（无旋转）")
    print("="*60)

    target_rot_4 = robot.rotation_matrix_from_euler(0, 0, 0)
    print_orientation_info(target_rot_4, "目标姿态")

    success = robot.moveL(
        target_position=current_pos,  # 位置不变
        target_rotation=target_rot_4,  # 回到初始姿态
        duration=3.0,
        use_spline=True
    )

    if not success:
        print("示例4失败")
        return

    time.sleep(1)

    # ========== 示例5: 连续姿态变换（圆锥运动）==========
    print("\n" + "="*60)
    print("示例5: 连续姿态变换（圆锥运动）")
    print("末端绕固定点做圆锥运动")
    print("="*60)

    # 定义圆锥运动的参数
    cone_angle = np.pi / 6  # 圆锥半角 30°
    num_steps = 8  # 8 个姿态

    for i in range(num_steps):
        angle = 2 * np.pi * i / num_steps  # 绕 Z 轴的角度

        # 计算目标姿态：先绕 Y 轴倾斜，再绕 Z 轴旋转
        rot_y = robot.rotation_matrix_from_euler(0, cone_angle, 0)
        rot_z = robot.rotation_matrix_from_euler(0, 0, angle)
        # 左乘（z轴角度是不断变化的），使用的是底座坐标系旋转，观察机械臂末端可以看出明显效果（绕着底座坐标系的z轴旋转呈现一个锥形）
        target_rot = rot_z @ rot_y

        print(f"\n步骤 {i+1}/{num_steps}: 角度 = {np.rad2deg(angle):.1f}°")

        success = robot.moveL(
            target_position=current_pos,
            target_rotation=target_rot,
            duration=1.5,
            use_spline=True
        )

        if not success:
            print(f"步骤 {i+1} 失败")
            break

        time.sleep(0.5)

    print("\n圆锥运动完成！")
    time.sleep(1)

    # 回到零位
    print("\n回到零位...")
    robot.moveJ(zero_pos, duration=3.0, iswait=True)

    print("\n" + "="*60)
    print("示例程序执行完成！")
    print("="*60)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"\n发生错误: {e}")
        import traceback
        traceback.print_exc()
