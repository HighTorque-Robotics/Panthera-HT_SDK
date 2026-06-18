#!/usr/bin/env python3
"""
六关节机器人位置速度循环控制程序
启动后在零位和 pos_A 之间往返循环。
按下 Ctrl+C 时，机械臂先慢速移动到 pos_B，再返回零位，然后结束程序。
"""
import time
import numpy as np
from Panthera_lib import Panthera


def build_slowdown_velocity(target_pos, max_vel, min_vel, slow_down_distance):
    """根据当前位置到目标位置的剩余距离，生成逐渐减小的关节速度。"""
    current_pos = robot.get_current_pos()
    delta = np.asarray(target_pos) - current_pos
    distance = np.abs(delta)

    normalized_distance = np.clip(distance / slow_down_distance, 0.0, 1.0)
    smooth_scale = 1.0 - np.power(1.0 - normalized_distance, 5.0)

    speed = np.asarray(min_vel) + (np.asarray(max_vel) - np.asarray(min_vel)) * smooth_scale
    speed = np.minimum(speed, np.sqrt(distance) * near_target_gain + np.asarray(min_vel))
    speed = np.where(distance < stop_distance, 0.0, speed)
    return (np.sign(delta) * speed).tolist()


def move_and_report(target_pos, max_vel, label, pause=1.0, timeout=15.0):
    """使用 Joint_Pos_Vel 控制，并在接近目标位置时自动减速。"""
    print(f"\n发送控制命令: {label}")
    start_time = time.time()
    success = False

    while (time.time() - start_time) < timeout:
        cmd_vel = build_slowdown_velocity(target_pos, max_vel, min_vel, slow_down_distance)
        robot.Joint_Pos_Vel(target_pos, cmd_vel, max_torque, iswait=False)

        success, errors = robot.check_position_reached(target_pos, tolerance=reach_tolerance)
        if success:
            break

        time.sleep(control_dt)

    print(f"{label} 执行状态: {success}")
    if not success:
        raise RuntimeError(f"{label} 执行失败，当前位置误差: {[f'{e:.3f}' for e in errors]}")
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
    move_and_report(pos_B, exit_vel, "慢速移动到 pos_B", pause=1.0)
    move_and_report(zero_pos, exit_vel, "慢速返回零位", pause=0.5)


if __name__ == "__main__":
    robot = Panthera()

    zero_pos = [0.0] * robot.motor_count
    pos_A = [0.0, 1.2, 0.6, -0.9, 0.0, 0.0]
    pos_B = [0.0, 0.2, 0.2, 0.0, 0.0, 0.0]

    loop_vel = [3.2] * robot.motor_count
    exit_vel = [1.2] * robot.motor_count
    min_vel = [0.06] * robot.motor_count
    max_torque = [21.0, 36.0, 36.0, 21.0, 10.0, 10.0]
    slow_down_distance = 0.01
    stop_distance = 0.002
    near_target_gain = 0.5
    reach_tolerance = 0.005
    control_dt = 0.02

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
