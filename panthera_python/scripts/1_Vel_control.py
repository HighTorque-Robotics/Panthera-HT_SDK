#!/usr/bin/env python3
"""
简单的单关节速度控制程序
第一个关节每隔3秒在正负0.2 rad/s之间切换
"""
import time
import math
from Panthera_lib import Panthera

if __name__ == "__main__":
    robot = Panthera()

    try:
        while True:
            t = time.time()
            target_vel = [0.2 if (math.floor(t) % 6) >= 3 else -0.2] + [0.0] * (robot.motor_count - 1)
            robot.Joint_Vel(target_vel)

            # 打印状态
            print(f"目标速度: {target_vel[0]:.2f} rad/s")
            print(f"当前位置: {robot.get_current_pos()[0]:.3f} rad")
            print(f"当前速度: {robot.get_current_vel()[0]:.3f} rad/s")
            print("-" * 40)

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\n程序被中断")
    except Exception as e:
        print(f"\n错误: {e}")
