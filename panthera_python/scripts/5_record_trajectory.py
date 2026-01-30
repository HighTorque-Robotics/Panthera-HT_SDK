#!/usr/bin/env python3
"""
单主臂重力补偿程序 + 实时轨迹记录（位置+速度+夹爪）
"""
import os
import time
import numpy as np
from Panthera_lib import Panthera, TrajectoryRecorder

# ---------------- 参数区 ----------------
DO_RECORD = True          # True=记录  False=不记录
REC_FILE  = None          # None=自动生成文件名
# ---------------------------------------

def main():
    # 获取主臂当前状态
    Leader_positions = Leader.get_current_pos()
    Leader_velocity = Leader.get_current_vel()

    # 获取夹爪当前状态
    gripper_pos = Leader.get_current_pos_gripper()
    gripper_vel = Leader.get_current_vel_gripper()

    # 计算重力补偿力矩
    Leader_gra = Leader.get_Gravity()

    # 添加摩擦补偿
    Leader_tor = np.array(Leader_gra) + Leader.get_friction_compensation(Leader_velocity, Fc, Fv, vel_threshold)

    # 力矩限幅（基于电机规格）
    tau_limit = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0])
    Leader_tor = np.clip(Leader_tor, -tau_limit, tau_limit)

    # 零刚度零阻尼控制（纯重力补偿模式，可自由拖动）
    Leader.pos_vel_tqe_kp_kd(zero_pos, zero_vel, Leader_tor, zero_kp, zero_kd)

    # 夹爪零刚度零阻尼控制（可自由拖动）
    Leader.gripper_control_MIT(0.0, 0.0, 0.0, 0.0, 0.0)

    # 打印6个关节信息 + 夹爪
    print("\r", end="")
    for i in range(Leader.motor_count):
        print(f"J{i+1}: {Leader_positions[i]:6.3f}rad {Leader_velocity[i]:6.3f}rad/s | ", end="")
    print(f"夹爪: {gripper_pos:6.3f}rad {gripper_vel:6.3f}rad/s   ", end="", flush=True)

    time.sleep(0.001)

if __name__ == "__main__":
    # 创建机器人实例
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, "../robot_param/Follower.yaml")
    Leader = Panthera(config_path)

    # 创建零位置和零速度数组
    zero_pos = [0.0] * Leader.motor_count
    zero_vel = [0.0] * Leader.motor_count
    zero_kp = [0.0] * Leader.motor_count
    zero_kd = [0.0] * Leader.motor_count

    # 摩擦补偿参数
    Fc = np.array([0.15, 0.12, 0.12, 0.12, 0.04, 0.04])
    Fv = np.array([0.05, 0.05, 0.05, 0.03, 0.02, 0.02])
    vel_threshold = 0.02

    # 实例化记录器（如开启记录）
    if DO_RECORD:
        rec = TrajectoryRecorder(REC_FILE)
        print("开始记录轨迹（位置+速度+夹爪）...")

    try:
        # 记录轨迹之前先循环发送读取指令，避免未接收到关节状态导致关节角为999
        for i in range(10):
            Leader.send_get_motor_state_cmd()
            time.sleep(0.1)
        while True:
            main()                            # 重力补偿控制循环
            if DO_RECORD:
                # 记录关节位置速度 + 夹爪位置速度
                rec.log(
                    Leader.get_current_pos(),
                    Leader.get_current_vel(),
                    Leader.get_current_pos_gripper(),
                    Leader.get_current_vel_gripper()
                )
    except KeyboardInterrupt:
        if DO_RECORD:
            rec.close()
        print("\n程序停止" + ("，轨迹已保存" if DO_RECORD else ""))