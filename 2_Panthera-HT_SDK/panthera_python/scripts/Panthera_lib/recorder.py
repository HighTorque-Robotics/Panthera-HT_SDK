#!/usr/bin/env python3
"""
轨迹记录与回放工具（无脏行版）
"""
import time, json, os
import numpy as np
from typing import List, Optional


class Recorder:
    def __init__(self, path: str = None):
        if path is None:
            path = time.strftime("trajectory_%Y%m%d_%H%M%S.jsonl")
        self.path = path
        self.fd = open(path, "w", encoding="utf-8")
        self.t0 = None

    # ---------------- 记录 ----------------
    def log(self, pos: List[float], vel: List[float] = None, gripper_pos: float = None, gripper_vel: float = None):
        t = time.time()
        if self.t0 is None:
            self.t0 = t
        # 一次性生成合法 JSON 字符串，避免半截写入
        data = {"t": t - self.t0, "pos": list(pos)}
        if vel is not None:
            data["vel"] = list(vel)
        if gripper_pos is not None:
            data["gripper_pos"] = gripper_pos
        if gripper_vel is not None:
            data["gripper_vel"] = gripper_vel
        line = json.dumps(data, ensure_ascii=False)
        self.fd.write(line + "\n")
        self.fd.flush()          # 立即落盘，防止 Ctrl-C 时断行

    def close(self):
        if self.fd and not self.fd.closed:
            self.fd.close()
            print(f"[Recorder] 轨迹已保存 → {self.path}")

    # ---------------- 静态回放 ----------------
    @staticmethod
    def play(
        robot,
        filepath: str,
        kp: List[float],
        kd: List[float],
        fc: Optional[List[float]] = None,
        fv: Optional[List[float]] = None,
        vel_threshold: float = 0.0,
        tau_limit: Optional[List[float]] = None,
        gripper_kp: float = 5.0,
        gripper_kd: float = 0.5,
    ):
        # 回放模式选择使用电机位置速度模式时会用到
        max_torque = [21.0, 36.0, 36.0, 21.0, 10.0, 10.0] 
        with open(filepath, "r", encoding="utf-8") as f:
            frames = [json.loads(line) for line in f if line.strip()]
        if not frames:
            print("[Player] 空文件，无数据回放")
            return

        print(f"[Player] 共 {len(frames)} 帧")

        # 移动到起始点
        first_frame = frames[0]
        print("[Player] 正在移动到轨迹起点...")

        # 关节移动到起点（使用 pos_vel_MAXtqe 模式）
        start_pos = first_frame["pos"]
        move_vel = [0.5] * len(start_pos)  # 缓慢速度 0.5 rad/s

        # 夹爪移动到起点（如果有夹爪数据）
        if "gripper_pos" in first_frame:
            gripper_start_pos = first_frame["gripper_pos"]
            print(f"[Player] 夹爪移动到起点: {gripper_start_pos:.3f} rad")
            robot.gripper_control(gripper_start_pos, 0.5, 0.5)
            time.sleep(2.0)  # 等待夹爪到达

        # 缓慢移动到起点，等待到达
        robot.pos_vel_MAXtqe(start_pos, move_vel, max_torque, iswait=True, tolerance=0.05, timeout=30.0)

        print("[Player] 已到达起点，开始回放...")
        t0 = time.time()
        
        for f in frames:
            while time.time() - t0 < f["t"]:
                time.sleep(0.001)

            # 关节控制
            if fc is not None and fv is not None:
                robot_gra = robot.get_Gravity()
                robot_vel = robot.get_current_vel()
                robot_torque = np.array(robot_gra) + robot.get_friction_compensation(robot_vel, fc, fv, vel_threshold)
                if tau_limit is not None:
                    robot_torque = np.clip(robot_torque, -np.array(tau_limit), np.array(tau_limit))
            else:
                robot_torque = [0.0] * 6
            robot.pos_vel_tqe_kp_kd(f["pos"], f["vel"], robot_torque, kp, kd)
            # robot.pos_vel_MAXtqe(f["pos"], f["vel"], max_torque)

            # 夹爪控制（如果有夹爪数据）
            if "gripper_pos" in f:
                robot.gripper_control_MIT(f["gripper_pos"], f["gripper_vel"], 0.0, gripper_kp, gripper_kd)

        print("[Player] 回放完成")
