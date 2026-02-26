#!/usr/bin/env python3
"""
机械臂笛卡尔空间速度控制程序（基于雅可比矩阵）

功能：
    1. 启动时机械臂缓慢移动到安全位置
    2. 使用键盘控制末端效应器速度（按住时持续运动）
    3. 使用阻尼伪逆避免雅可比奇异性
    4. 实时显示当前末端位置和速度

键盘控制：
    W/S: 工具坐标系X轴线速度 (+/-)
    A/D: 工具坐标系Y轴线速度 (+/-)
    Q/E: 工具坐标系Z轴线速度 (+/-)
    1/2: 绕工具坐标系X轴角速度 (+/-)
    3/4: 绕工具坐标系Y轴角速度 (+/-)
    5/6: 绕工具坐标系Z轴角速度 (+/-)
    ESC: 退出程序

注意事项：
    - 按住按键时末端持续运动，松开后停止
    - 使用阻尼伪逆避免奇异性，运动更平滑
    - 接近工作空间边界时会自动减速
"""

import time
import numpy as np
from pynput import keyboard
from Panthera_lib import Panthera

# 全局变量
end_velocity = np.zeros(6)  # 目标速度 [vx, vy, vz, ωx, ωy, ωz]
actual_velocity = np.zeros(6)  # 实际速度（平滑后）
linear_speed = 0.3  # 线速度 m/s
angular_speed = 2.0  # 角速度 rad/s
running = True
keys_pressed = set()  # 记录当前按下的键

# 加减速参数
acceleration_factor = 0.02  # 加速因子（0-1，值越大加速越快）
                            # 0.15 约需 0.2-0.3 秒达到目标速度

def on_press(key):
    """键盘按下事件处理"""
    global end_velocity, running, keys_pressed

    # 字母键控制线速度
    if hasattr(key, 'char') and key.char:
        keys_pressed.add(key.char.lower())

        if key.char.lower() == 'w':
            end_velocity[0] = linear_speed  # X轴向前
        elif key.char.lower() == 's':
            end_velocity[0] = -linear_speed  # X轴向后
        elif key.char.lower() == 'a':
            end_velocity[1] = linear_speed  # Y轴向左
        elif key.char.lower() == 'd':
            end_velocity[1] = -linear_speed  # Y轴向右
        elif key.char.lower() == 'q':
            end_velocity[2] = linear_speed  # Z轴向上
        elif key.char.lower() == 'e':
            end_velocity[2] = -linear_speed  # Z轴向下
        # 数字键控制角速度
        elif key.char == '1':
            end_velocity[3] = angular_speed  # 绕X轴
        elif key.char == '2':
            end_velocity[3] = -angular_speed
        elif key.char == '3':
            end_velocity[4] = angular_speed  # 绕Y轴
        elif key.char == '4':
            end_velocity[4] = -angular_speed
        elif key.char == '5':
            end_velocity[5] = angular_speed  # 绕Z轴
        elif key.char == '6':
            end_velocity[5] = -angular_speed

def on_release(key):
    """键盘释放事件处理"""
    global end_velocity, running, keys_pressed

    if key == keyboard.Key.esc:
        print("\n检测到ESC键，准备退出...")
        running = False
        return False  # 停止监听

    # 字母键释放时清零对应速度
    if hasattr(key, 'char') and key.char:
        char = key.char.lower()
        if char in keys_pressed:
            keys_pressed.remove(char)

        if char in ['w', 's']:
            end_velocity[0] = 0.0
        elif char in ['a', 'd']:
            end_velocity[1] = 0.0
        elif char in ['q', 'e']:
            end_velocity[2] = 0.0
        elif char in ['1', '2']:
            end_velocity[3] = 0.0
        elif char in ['3', '4']:
            end_velocity[4] = 0.0
        elif char in ['5', '6']:
            end_velocity[5] = 0.0

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
    global end_velocity, running

    print("="*60)
    print("机械臂笛卡尔空间速度控制程序（基于雅可比矩阵）")
    print("="*60)

    # 初始化机械臂
    print("\n初始化机械臂...")
    robot = Panthera()

    # 检查是否有 Pinocchio 模型
    if robot.model is None:
        print("错误：未找到 Pinocchio 模型，无法计算雅可比矩阵")
        return

    # 移动到安全位置
    if not move_to_safe_position(robot):
        print("初始化失败，退出程序")
        return

    # 获取当前位姿
    current_fk = robot.forward_kinematics()
    current_pos = current_fk['position']

    print(f"\n初始位置: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}] m")
    print("\n" + "="*60)
    print("键盘控制说明（按住时持续运动）：")
    print("  W/S: 工具坐标系X轴线速度 (+/-)")
    print("  A/D: 工具坐标系Y轴线速度 (+/-)")
    print("  Q/E: 工具坐标系Z轴线速度 (+/-)")
    print("  1/2: 绕工具坐标系X轴角速度 (+/-)")
    print("  3/4: 绕工具坐标系Y轴角速度 (+/-)")
    print("  5/6: 绕工具坐标系Z轴角速度 (+/-)")
    print("  ESC: 退出程序")
    print("="*60)
    print("\n开始控制，请小心操作！\n")

    # 启动键盘监听
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    # 控制参数
    control_rate = 0.01  # 100Hz 控制频率
    damping_base = 0.01  # 基础阻尼系数
    damping_adaptive = True  # 自适应阻尼

    # MIT 控制参数
    kp = [0.0] * robot.motor_count  # 位置增益为0（纯速度控制）
    kd = [10.0, 15.0, 15.0, 10.0, 5.0, 5.0]  # 阻尼增益

    # 关节速度限制
    max_joint_vel = np.array([2.0, 2.0, 2.0, 2.0, 3.0, 3.0])  # rad/s

    try:
        while running:
            # 获取当前关节角度
            q = robot.get_current_pos()

            # 计算雅可比矩阵
            try:
                J = robot.get_jacobian(q)
            except Exception as e:
                print(f"\r雅可比计算失败: {e}                    ", end='')
                time.sleep(control_rate)
                continue

            # 计算可操作度
            manipulability = robot.get_manipulability(q)

            # 自适应阻尼：接近奇异时增大阻尼
            if damping_adaptive:
                # 可操作度阈值
                manip_threshold = 0.01
                if manipulability < manip_threshold:
                    # 接近奇异，增大阻尼并减速
                    damping = damping_base * (1.0 + (manip_threshold - manipulability) * 100)
                    speed_scale = manipulability / manip_threshold
                    end_velocity_scaled = end_velocity * speed_scale
                else:
                    damping = damping_base
                    end_velocity_scaled = end_velocity
            else:
                damping = damping_base
                end_velocity_scaled = end_velocity

            # 计算阻尼伪逆
            J_damp = Panthera.compute_damped_pseudoinverse(J, damping)

            # === 速度平滑（加减速） ===
            # 使用指数平滑让实际速度逐渐接近目标速度
            global actual_velocity
            actual_velocity = actual_velocity + (end_velocity_scaled - actual_velocity) * acceleration_factor

            # 使用平滑后的速度
            end_velocity_to_use = actual_velocity

            # 获取当前末端位置和姿态
            current_fk = robot.forward_kinematics()
            current_pos = current_fk['position']
            R_tool = current_fk['rotation']  # 工具坐标系的旋转矩阵

            # 将线速度和角速度从工具坐标系转换到世界坐标系
            # v_world = R_tool @ v_tool
            # ω_world = R_tool @ ω_tool
            end_velocity_world = np.zeros(6)
            end_velocity_world[0:3] = R_tool @ end_velocity_to_use[0:3]  # 线速度转换
            end_velocity_world[3:6] = R_tool @ end_velocity_to_use[3:6]  # 角速度转换

            # 计算关节速度
            q_dot = J_damp @ end_velocity_world

            # 限制关节速度
            q_dot = np.clip(q_dot, -max_joint_vel, max_joint_vel)

            robot.Joint_Vel(q_dot)

            # 显示信息
            vel_norm = np.linalg.norm(actual_velocity[:3])  # 使用实际速度
            print(f"\r位置: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}] | "
                  f"速度: {vel_norm:.3f} m/s | 可操作度: {manipulability:.4f} | 阻尼: {damping:.4f}", end='')

            time.sleep(control_rate)

    except KeyboardInterrupt:
        print("\n\n程序被中断")
    finally:
        listener.stop()
        # 停止运动
        print("\n\n停止运动...")
        q = robot.get_current_pos()
        gra = robot.get_Gravity()
        robot.pos_vel_tqe_kp_kd(q, [0.0]*robot.motor_count, gra, kp, kd)
        time.sleep(0.5)

        print("返回零位...")
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
