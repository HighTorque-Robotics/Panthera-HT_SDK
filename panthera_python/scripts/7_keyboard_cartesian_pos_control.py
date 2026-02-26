#!/usr/bin/env python3
"""
机械臂笛卡尔空间键盘控制程序

功能：
    1. 启动时机械臂缓慢移动到安全位置
    2. 使用键盘控制末端效应器在笛卡尔空间移动
    3. 实时显示当前末端位置

键盘控制：
    W/S: X轴前后移动
    A/D: Y轴左右移动
    Q: Z轴向上移动
    E: Z轴向下移动
    1/2: 绕X轴旋转 (+/-)
    3/4: 绕Y轴旋转 (+/-)
    5/6: 绕Z轴旋转 (+/-)
    ESC: 退出程序

注意事项：
    - 由于逆运动学的多解和无解特征，在某些姿态下关节可能会突变
    - 建议在机械臂工作空间中心区域操作
    - 使用者应远离机械臂的工作空间
"""

import time
import numpy as np
from pynput import keyboard
from Panthera_lib import Panthera

# 全局变量
target_position = np.array([0.24, 0.0, 0.15])  # 初始目标位置 (m)
target_rotation = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])  # 初始姿态
position_delta = 0.005  # 位置增量 3mm
rotation_delta = 0.03  # 旋转增量 约1.7度
running = True
position_changed = False  # 标记位置是否改变

def rotation_matrix_x(angle):
    """绕X轴旋转矩阵"""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def rotation_matrix_y(angle):
    """绕Y轴旋转矩阵"""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def rotation_matrix_z(angle):
    """绕Z轴旋转矩阵"""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def on_press(key):
    """键盘按下事件处理"""
    global target_position, target_rotation, running, position_changed

    # 字母键控制位置
    if hasattr(key, 'char') and key.char:
        if key.char == 'w' or key.char == 'W':
            target_position[0] += position_delta  # X轴向前
            position_changed = True
        elif key.char == 's' or key.char == 'S':
            target_position[0] -= position_delta  # X轴向后
            position_changed = True
        elif key.char == 'a' or key.char == 'A':
            target_position[1] += position_delta  # Y轴向左
            position_changed = True
        elif key.char == 'd' or key.char == 'D':
            target_position[1] -= position_delta  # Y轴向右
            position_changed = True
        elif key.char == 'q' or key.char == 'Q':
            target_position[2] += position_delta  # Z轴向上
            position_changed = True
        elif key.char == 'e' or key.char == 'E':
            target_position[2] -= position_delta  # Z轴向下
            position_changed = True
        # 数字键控制旋转
        # （增量右乘，欧拉角计算方法，每次旋转都基于当前已旋转后的坐标系）
        elif key.char == '1':
            target_rotation = target_rotation @ rotation_matrix_x(rotation_delta)
            position_changed = True
        elif key.char == '2':
            target_rotation = target_rotation @ rotation_matrix_x(-rotation_delta)
            position_changed = True
        elif key.char == '3':
            target_rotation = target_rotation @ rotation_matrix_y(rotation_delta)
            position_changed = True
        elif key.char == '4':
            target_rotation = target_rotation @ rotation_matrix_y(-rotation_delta)
            position_changed = True
        elif key.char == '5':
            target_rotation = target_rotation @ rotation_matrix_z(rotation_delta)
            position_changed = True
        elif key.char == '6':
            target_rotation = target_rotation @ rotation_matrix_z(-rotation_delta)
            position_changed = True

        # （增量左乘，固定角计算方法，每次旋转都基于固定不动的世界坐标系）
        # elif key.char == '1':
        #     target_rotation = rotation_matrix_x(rotation_delta) @ target_rotation 
        #     position_changed = True
        # elif key.char == '2':
        #     target_rotation = rotation_matrix_x(-rotation_delta) @ target_rotation 
        #     position_changed = True
        # elif key.char == '3':
        #     target_rotation = rotation_matrix_y(rotation_delta) @ target_rotation
        #     position_changed = True
        # elif key.char == '4':
        #     target_rotation = rotation_matrix_y(-rotation_delta) @ target_rotation
        #     position_changed = True
        # elif key.char == '5':
        #     target_rotation = rotation_matrix_z(rotation_delta) @ target_rotation 
        #     position_changed = True
        # elif key.char == '6':
        #     target_rotation = rotation_matrix_z(-rotation_delta) @ target_rotation 
        #     position_changed = True

def on_release(key):
    """键盘释放事件处理"""
    global running
    if key == keyboard.Key.esc:
        print("\n检测到ESC键，准备退出...")
        running = False
        return False  # 停止监听

def move_to_safe_position(robot):
    """启动时缓慢移动到安全位置"""
    print("\n" + "="*60)
    print("正在移动到安全位置...")
    print("="*60)
    # 定义安全位置（关节空间）
    safe_joint_pos = [0.0, 0.5, 0.6, 0.0, 0.0, 0.0]

    # 先刷新状态
    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    time.sleep(0.3)

    # 缓慢移动到安全位置（3秒）
    print("移动中...")
    success = robot.Joint_Pos_Vel(safe_joint_pos, [0.5]*robot.motor_count, iswait=True)

    if success:
        print("✓ 已到达安全位置")
    else:
        print("✗ 移动到安全位置失败")
        return False

    time.sleep(0.5)
    return True

def main():
    global target_position, target_rotation, running, position_changed

    print("="*60)
    print("机械臂笛卡尔空间键盘控制程序")
    print("="*60)

    # 初始化机械臂
    print("\n初始化机械臂...")
    robot = Panthera()

    # 移动到安全位置
    if not move_to_safe_position(robot):
        print("初始化失败，退出程序")
        return

    # 刷新状态，确保获取最新的关节角度
    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    time.sleep(0.1)

    # 获取当前位姿作为初始目标
    current_fk = robot.forward_kinematics()
    target_position = np.array(current_fk['position'])
    target_rotation = np.array(current_fk['rotation'], dtype=float)  # 转换为标准 numpy 数组

    print(f"\n初始位置: [{target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f}] m")
    print("\n" + "="*60)
    print("键盘控制说明：")
    print("  方向键上/下: X轴前后移动")
    print("  方向键左/右: Y轴左右移动")
    print("  空格键: Z轴向上")
    print("  Shift键: Z轴向下")
    print("  1/2: 绕X轴旋转")
    print("  3/4: 绕Y轴旋转")
    print("  5/6: 绕Z轴旋转")
    print("  ESC: 退出程序")
    print("="*60)
    print("\n开始控制，请小心操作！\n")

    # 启动键盘监听
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    # 控制循环
    last_valid_joint_pos = robot.get_current_pos()
    control_rate = 0.01  # 20Hz控制频率
    kp = [30.0, 50.0, 60.0, 25.0, 15.0, 10.0]        # 回放时刚度（可微调）
    kd = [3.0, 5.0, 6.0, 2.5, 1.5, 1.0]           # 回放时阻尼

    # 先发送一次当前位置命令，稳定机械臂
    robot_gra = robot.get_Gravity()
    robot_torque = np.array(robot_gra)
    robot.pos_vel_tqe_kp_kd(last_valid_joint_pos, [0.0]*robot.motor_count, robot_torque, kp, kd)
    time.sleep(0.2)

    try:
        while running:
            # 只在位置改变时才重新计算逆运动学
            if position_changed:
                # 计算逆运动学
                joint_pos = robot.inverse_kinematics(
                    target_position.tolist(),
                    target_rotation,
                    last_valid_joint_pos,
                    multi_init=False
                )

                if joint_pos is not None:
                    # mit+重力前馈下更丝滑
                    robot_gra = robot.get_Gravity()
                    robot_torque = np.array(robot_gra)
                    robot.pos_vel_tqe_kp_kd(joint_pos, [0.0]*robot.motor_count, robot_torque, kp, kd)

                    last_valid_joint_pos = joint_pos
                    position_changed = False  # 重置标志
                else:
                    # 逆运动学无解，保持当前位置
                    print("\r逆运动学无解，保持当前位置", end='')
                    position_changed = False  # 重置标志

            # 获取当前实际位置并显示
            current_fk = robot.forward_kinematics()
            current_pos = current_fk['position']

            # 打印当前位置
            print(f"\r目标位置: [{target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f}] | "
                  f"当前位置: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]", end='')

            time.sleep(control_rate)

    except KeyboardInterrupt:
        print("\n\n程序被中断")
    finally:
        listener.stop()
        print("\n\n返回零位...")
        zero_pos = [0.0] * robot.motor_count
        robot.Joint_Pos_Vel(zero_pos, [0.5]*robot.motor_count, iswait=True)
        print("所有电机已停止")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
