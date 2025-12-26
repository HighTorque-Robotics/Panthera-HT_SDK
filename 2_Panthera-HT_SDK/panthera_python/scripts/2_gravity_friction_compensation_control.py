#!/usr/bin/env python3
"""
重力+摩擦力补偿程序
补偿重力项和摩擦力项
使用库伦摩擦 + 粘性摩擦模型
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
    # 获取当前关节速度
    vel = robot.get_current_vel()

    # 获取重力补偿力矩
    tau_gravity = robot.get_Gravity()

    # 计算摩擦力补偿力矩
    tau_friction = robot.get_friction_compensation(vel, Fc, Fv, vel_threshold)

    # 总补偿力矩 = 重力补偿 + 摩擦力补偿
    tau_total = tau_gravity + tau_friction

    # 力矩限幅（基于电机规格）
    tau_limit = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0])
    tau_total = np.clip(tau_total, -tau_limit, tau_limit)

    # 发送控制指令
    robot.pos_vel_tqe_kp_kd(zero_pos, zero_vel, tau_total, zero_kp, zero_kd)

    # 打印信息
    print(f"速度: {vel}")
    print(f"重力补偿: {tau_gravity}")
    print(f"摩擦补偿: {tau_friction}")
    print(f"总补偿力矩: {tau_total}")
    print("-" * 60)

    time.sleep(0.005)


if __name__ == "__main__":
    robot = Panthera()

    # ==================== 摩擦参数配置 ====================
    # 注意：这些参数需要根据实际机器人进行辨识和调整

    # 库伦摩擦系数 Fc (Nm) - 恒定摩擦力，与速度大小无关
    # 建议初始值：较小的关节用较小值，较大的关节用较大值
    # 参数辨识方法：让关节以极低速度匀速运动，测量所需的最小恒定力矩
    Fc = np.array([
        0.20,  # 关节1
        0.15,  # 关节2
        0.15,  # 关节3
        0.15,  # 关节4
        0.04, # 关节5
        0.04  # 关节6
    ])

    # 粘性摩擦系数 Fv (Nm·s/rad) - 线性速度相关摩擦系数
    # 建议初始值：通常比库伦摩擦小一个数量级
    # 参数辨识方法：让关节以不同速度匀速运动，测量力矩-速度曲线的斜率
    Fv = np.array([
        0.06,  # 关节1
        0.06,   # 关节2
        0.06,   # 关节3
        0.03,  # 关节4
        0.02,  # 关节5
        0.02   # 关节6
    ])

    # 速度阈值 (rad/s) - 低于此速度时不使用库伦摩擦项
    # 建议值：0.01-0.05 rad/s
    vel_threshold = 0.02

    # ====================================================

    # 创建零位置、速度、刚度、阻尼数组
    zero_pos = [0.0] * robot.motor_count
    zero_vel = [0.0] * robot.motor_count
    zero_kp = [0.0] * robot.motor_count
    zero_kd = [0.0] * robot.motor_count

    print("=" * 60)
    print("重力 + 摩擦力补偿控制启动")
    print("=" * 60)
    print(f"库伦摩擦系数 Fc: {Fc}")
    print(f"粘性摩擦系数 Fv: {Fv}")
    print(f"速度阈值: {vel_threshold} rad/s")
    print("=" * 60)
    print("\n按 Ctrl+C 停止程序\n")

    try:
        while True:
            main()
    except KeyboardInterrupt:
        # robot.set_stop()
        print("\n\n程序被中断")
        print("所有电机已停止")
    except Exception as e:
        # robot.set_stop()
        print(f"\n错误: {e}")
        print("所有电机已停止")
