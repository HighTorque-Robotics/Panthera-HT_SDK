#!/usr/bin/env python3
"""
逆运动学验证程序
两种验证方法
1.当前位置正解求取末端位姿，在使用当前末端位姿进行逆解验证（可拖动查看逆解效果）
2.指定位置和姿态进行逆解验证
"""
import time
import sys
import os
import numpy as np
# 添加上一级目录(python目录)到sys.path
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_dir)
from scripts.Panthera_lib.Panthera import Panthera 

def main():
    # 方法1：当前位置正解求取末端位姿，在使用当前末端位姿进行逆解验证
    # 利用运控模式发送控制帧以读取电机反馈状态
    robot.send_get_motor_state_cmd()
    current_angles = robot.get_current_pos()
    fk = robot.forward_kinematics(current_angles)
    if not fk:
        return
    ik_pos = fk['position']
    ik_rot = fk['rotation']
    # 使用当前关节角减0.1作为初始计算角（向量运算）
    init_q = current_angles - 0.1
    # # 使用零位作为初始计算角
    # init_q = np.zeros(robot.motor_count)
    # 使用当前位置和姿态进行逆解
    solved_angles = robot.inverse_kinematics(ik_pos, ik_rot, init_q)
    if solved_angles is not None:
        print(f"\n当前关节: {current_angles}")
        print(f"逆解关节: {solved_angles}")
        # 计算误差（向量运算）
        errors = np.abs(current_angles - solved_angles)
        max_error = np.max(errors)
        print(f"最大误差: {max_error:.4f} rad")
    time.sleep(0.5)

    # # 方法2: 指定位置和姿态进行逆解验证
    # target_pos = [0.3, 0.2, 0.2]
    # target_rot = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  # 单位矩阵作为示例
    # # 逆解计算
    # init_q = [0.0] * robot.motor_count
    # ik_angles = robot.inverse_kinematics(target_pos, target_rot, init_q)
    # if ik_angles:
    #     # 用逆解结果进行正解验证
    #     fk_verify = robot.forward_kinematics(ik_angles)
    #     if fk_verify:
    #         # 位置误差
    #         pos_error = np.linalg.norm(np.array(target_pos) - np.array(fk_verify['position']))
    #         # 姿态误差（旋转矩阵差的Frobenius范数）
    #         rot_error = np.linalg.norm(target_rot - fk_verify['rotation'], 'fro')
    #         print(f"\n目标位置: {target_pos}")
    #         print(f"验证位置: {[f'{p:.3f}' for p in fk_verify['position']]}")
    #         print(f"位置误差: {pos_error:.4f} m")
    #         print(f"姿态误差: {rot_error:.4f}")
    # time.sleep(0.5)


if __name__ == "__main__":
    robot = Panthera()
    try:
        while(1):
            main()
    except KeyboardInterrupt:
        robot.set_stop()
        print("\n\n程序被中断")
        print("\n\n所有电机已停止")
    except Exception as e:
        print(f"\n错误: {e}")