#!/usr/bin/env python3
"""
基础电机控制示例

演示如何使用hightorque_robot库进行基本的电机控制
"""
import time
import math
import os
import sys
# 将python目录添加到路径中以导入模块
# 从 motor_example 目录向上两层到达 panthera_python 目录
parent_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(parent_dir)
import hightorque_robot as htr


def basic_example():
    """基础示例: 单个电机位置控制"""
    print("=" * 60)
    print("基础示例: 单个电机位置控制")
    print("=" * 60)

    # 创建机器人实例
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, "../../robot_param/motor_param", "robot_config.yaml")
    robot = htr.Robot(config_path)

    # 获取所有电机
    motors = robot.get_motors()
    print(f"检测到 {len(motors)} 个电机")

    if len(motors) == 0:
        print("没有找到电机,请检查配置和连接")
        return

    # 控制第一个电机
    motor = motors[0]
    print(f"控制电机: {motor}")

    # 简单的往复运动
    positions = [0.5, -0.5, 0.0]
    for pos in positions:
        print(f"移动到位置: {pos} rad ({htr.rad_to_deg(pos):.1f} deg)")
        motor.position(pos)
        robot.motor_send_cmd()
        time.sleep(1.0)

        # 读取电机状态
        state = motor.get_current_motor_state()
        print(f"  实际位置: {state.position:.3f} rad, "
              f"速度: {state.velocity:.3f} rad/s, "
              f"力矩: {state.torque:.3f} Nm")

    # 停止电机
    robot.set_stop()
    print("电机已停止")


def multi_motor_example():
    """多电机控制示例"""
    print("\n" + "=" * 60)
    print("多电机控制示例")
    print("=" * 60)

    # 创建机器人实例
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, "../robot_param", "robot_config.yaml")
    robot = htr.Robot(config_path)

    motors = robot.get_motors()

    if len(motors) < 2:
        print("此示例需要至少2个电机")
        return

    print(f"控制 {len(motors)} 个电机")

    # 所有电机同时运动到不同位置
    for i, motor in enumerate(motors):
        angle = 0.5 if i % 2 == 0 else -0.5
        motor.position(angle)

    robot.motor_send_cmd()
    time.sleep(2.0)

    # 打印所有电机状态
    htr.print_motor_states(motors)

    robot.set_stop()


def sinusoidal_motion_example():
    """正弦运动示例"""
    print("\n" + "=" * 60)
    print("正弦运动示例")
    print("=" * 60)

    # 创建机器人实例
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, "../robot_param", "robot_config.yaml")
    robot = htr.Robot(config_path)

    motors = robot.get_motors()

    if len(motors) == 0:
        return

    motor = motors[0]
    print(f"电机 {motor.get_motor_id()} 执行正弦运动")

    # 创建正弦轨迹生成器
    trajectory = htr.create_sinusoidal_trajectory(
        amplitude=1.0,      # 振幅 1弧度
        frequency=0.5,      # 频率 0.5Hz
        offset=0.0          # 无偏移
    )

    # 运行5秒
    start_time = time.time()
    duration = 5.0

    try:
        while time.time() - start_time < duration:
            t = time.time() - start_time
            target_pos = trajectory(t)

            motor.position(target_pos)
            robot.motor_send_cmd()

            # 每0.5秒打印一次状态
            if int(t * 2) != int((t - 0.001) * 2):
                state = motor.get_current_motor_state()
                print(f"t={t:.2f}s: 目标={target_pos:.3f}, "
                      f"实际={state.position:.3f}, "
                      f"速度={state.velocity:.3f}")

            time.sleep(0.001)  # 1ms控制周期

    except KeyboardInterrupt:
        print("\n用户中断")

    robot.set_stop()
    print("运动完成")


def advanced_control_example():
    """高级控制示例: 使用混合控制模式"""
    print("\n" + "=" * 60)
    print("高级控制示例: 位置+速度+力矩控制")
    print("=" * 60)

    # 创建机器人实例
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, "../robot_param", "robot_config.yaml")
    robot = htr.Robot(config_path)

    motors = robot.get_motors()

    if len(motors) == 0:
        return

    motor = motors[0]

    # 使用位置+速度+最大力矩控制
    print("使用 pos_vel_MAXtqe 控制模式")

    target_positions = [
        (1.0, 0.5, 10.0),   # 位置1弧度, 速度0.5, 最大力矩10Nm
        (-1.0, 0.5, 10.0),
        (0.0, 0.2, 5.0),
    ]

    for pos, vel, tqe_max in target_positions:
        print(f"目标: 位置={pos:.2f}, 速度={vel:.2f}, 最大力矩={tqe_max:.2f}")
        motor.pos_vel_MAXtqe(pos, vel, tqe_max)
        robot.motor_send_cmd()
        time.sleep(2.0)

        state = motor.get_current_motor_state()
        print(f"  到达: 位置={state.position:.3f}, "
              f"速度={state.velocity:.3f}, "
              f"力矩={state.torque:.3f}\n")

    # 使用五参数控制
    print("使用 pos_vel_tqe_kp_kd 五参数控制模式")
    motor.pos_vel_tqe_kp_kd(
        position=0.5,
        velocity=0.1,
        torque=0.0,    # 前馈力矩
        kp=50.0,       # PID比例参数
        kd=5.0         # PID微分参数
    )
    robot.motor_send_cmd()
    time.sleep(2.0)

    robot.set_stop()


def motor_info_example():
    """电机信息查询示例"""
    print("\n" + "=" * 60)
    print("电机信息查询示例")
    print("=" * 60)

    # 创建机器人实例
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, "../robot_param", "robot_config.yaml")
    robot = htr.Robot(config_path)

    # 打印机器人信息
    print(f"机器人: {robot}")
    print(f"机器人名称: {robot.robot_params.robot_name}")
    print(f"超时设置: {robot.motor_timeout_ms} ms")
    print(f"CAN板数量: {robot.robot_params.CANboard_num}")

    motors = robot.get_motors()
    print(f"\n电机总数: {len(motors)}")
    print("-" * 60)

    for motor in motors:
        print(f"\n电机 {motor.get_motor_id()}:")
        print(f"  名称: {motor.get_motor_name()}")
        print(f"  类型: {motor.get_motor_enum_type()}")

        # 获取版本信息
        version = motor.get_version()
        print(f"  版本: v{version.major}.{version.minor}.{version.patch}")

        # 获取当前状态
        state = motor.get_current_motor_state()
        print(f"  位置: {state.position:.4f} rad ({htr.rad_to_deg(state.position):.2f} deg)")
        print(f"  速度: {state.velocity:.4f} rad/s")
        print(f"  力矩: {state.torque:.4f} Nm")
        print(f"  模式: {state.mode}")
        print(f"  故障码: 0x{state.fault:02X}")


def safety_features_example():
    """安全功能示例"""
    print("\n" + "=" * 60)
    print("安全功能示例")
    print("=" * 60)

    # 创建机器人实例
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, "../robot_param", "robot_config.yaml")
    robot = htr.Robot(config_path)

    motors = robot.get_motors()

    if len(motors) == 0:
        return

    motor = motors[0]

    # 设置超时
    print("设置超时: 100 ms")
    robot.set_timeout(100)

    # 测试控制
    print("\n测试电机控制...")
    motor.position(1.0)
    robot.motor_send_cmd()
    time.sleep(1.0)

    robot.set_stop()

def main():
    """主函数"""
    print("\n" + "=" * 60)
    print("高扭矩机器人电机控制 - Python示例程序")
    print("=" * 60)

    examples = [
        ("1", "基础示例", basic_example),
        ("2", "多电机控制", multi_motor_example),
        ("3", "正弦运动", sinusoidal_motion_example),
        ("4", "高级控制", advanced_control_example),
        ("5", "电机信息查询", motor_info_example),
        ("6", "安全功能", safety_features_example),
        ("0", "运行所有示例", None),
    ]

    print("\n可用示例:")
    for num, name, _ in examples:
        print(f"  {num}. {name}")

    choice = input("\n请选择示例 (0-6): ").strip()

    if choice == "0":
        # 运行所有示例
        for num, name, func in examples:
            if func is not None:
                try:
                    func()
                except Exception as e:
                    print(f"\n示例 '{name}' 出错: {e}")
                    import traceback
                    traceback.print_exc()
                input("\n按Enter继续下一个示例...")
    else:
        # 运行选定的示例
        for num, name, func in examples:
            if num == choice and func is not None:
                try:
                    func()
                except Exception as e:
                    print(f"\n示例出错: {e}")
                    import traceback
                    traceback.print_exc()
                break
        else:
            print("无效的选择")

    print("\n" + "=" * 60)
    print("示例程序结束")
    print("=" * 60)


if __name__ == "__main__":
    main()
