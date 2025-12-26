#!/usr/bin/env python3
import time
import math
import os
import sys
# 将python目录添加到路径中以导入模块
# 从 motor_example 目录向上两层到达 panthera_python 目录
parent_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(parent_dir)
import hightorque_robot as htr


if __name__ == "__main__":
    # 创建机器人实例
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, "../../robot_param/motor_param", "robot_config.yaml")
    robot = htr.Robot(config_path)

    motors = robot.get_motors()

    cnt = 0

    print(f"控制 {len(motors)} 个电机") 

    while True:
        # 打印所有电机状态
        if(cnt% 1000 == 0):
            for motor in motors:
                state = motor.get_current_motor_state()
                print(f"电机 {motor.get_motor_id()} 状态:")
                print(f"  位置: {state.position:.4f} rad ({htr.rad_to_deg(state.position):.2f} deg)")
                print(f"  速度: {state.velocity:.4f} rad/s")
                print(f"  力矩: {state.torque:.4f} Nm")
                print(f"  模式: {state.mode}")
                print(f"  故障码: 0x{state.fault:02X}")
            print("-" * 40)
        cnt += 1

        for motor in motors:
            t = time.time()
            target_vel = 0.2 if (math.floor(t) % 6) >= 3 else -0.2 # 每隔3秒在-0.2和0.2之间切换
            motor.velocity(target_vel)
        robot.motor_send_cmd()
        
        time.sleep(0.001)

    robot.set_stop()