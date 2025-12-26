#!/usr/bin/env python3
"""
回放 *.jsonl 轨迹文件（单臂+夹爪）
支持位置+速度+夹爪回放
"""
import os, sys, time
import numpy as np

parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_dir)

from scripts.Panthera_lib.Panthera import Panthera
from scripts.Panthera_lib.recorder import Recorder   # 用静态 play 方法

# ---------------- 参数区 ----------------
TRAJECTORY_FILE = "trajectory_test_1.jsonl"  # ← 改成实际记录生成的文件名

# 关节PD增益
kp_play = [10.0, 21.0, 21.0, 6.0, 5.0, 1.0]        # 回放时刚度（可微调）
kd_play = [1.0, 2.0, 2.0, 0.9, 0.7, 0.1]           # 回放时阻尼

# 夹爪PD增益
gripper_kp = 5.0   # 夹爪刚度
gripper_kd = 0.5   # 夹爪阻尼

# 摩擦补偿参数
Fc = [0.15, 0.12, 0.12, 0.12, 0.04, 0.04]
Fv = [0.05, 0.05, 0.05, 0.03, 0.02, 0.02]
vel_threshold = 0.02

# 力矩限制
tau_limit = [15.0, 30.0, 30.0, 15.0, 5.0, 5.0]
# ---------------------------------------

if __name__ == "__main__":
    if not os.path.isfile(TRAJECTORY_FILE):
        print(f"文件不存在：{TRAJECTORY_FILE}")
        sys.exit(1)

    # 创建机器人（用与记录时相同的 config）
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, "../robot_param/Follower.yaml")
    robot = Panthera(config_path)

    print(f"开始回放: {TRAJECTORY_FILE}")
    print("数据格式: 自动检测（支持位置+速度+夹爪）")

    try:
        # 一键回放（按记录时真实时间间隔发位置+速度+夹爪）
        Recorder.play(
            robot=robot,
            filepath=TRAJECTORY_FILE,
            kp=kp_play,
            kd=kd_play,
            fc=Fc,
            fv=Fv,
            vel_threshold=vel_threshold,
            tau_limit=tau_limit,
            gripper_kp=gripper_kp,
            gripper_kd=gripper_kd
        )

    except KeyboardInterrupt:
        print("\n回放被中断")
    finally:
        # robot.set_stop()
        print("电机已停止")