#!/usr/bin/env python3
"""
Panthera-HT 机械臂 Quest 手柄笛卡尔空间控制程序

功能：
    1. 通过 UDP 接收 Quest 手柄数据
    2. 当右手柄 Squeeze 按下时，使用手柄位姿控制机械臂末端
    3. 使用逆运动学进行笛卡尔空间控制
    4. 使用 MIT 模式 + 重力/摩擦前馈进行关节控制
    5. 支持夹爪开合控制
    6. 实时显示当前末端位置和手柄状态

控制说明：
    按下右手柄 Squeeze（侧键）: 激活控制（必须按下才能控制）
    移动手柄位置: 控制末端位置
    旋转手柄姿态: 控制末端姿态
    按住 Squeeze + 按下 Trigger（扳机键）: 根据按下程度控制夹爪
    未按下 Squeeze 时操作 Trigger: 不触发夹爪动作
    松开 Squeeze: 停止运动，保持当前位置
    Ctrl+C: 退出程序
"""

import socket
import struct
import time
import threading
import numpy as np
from scipy.spatial.transform import Rotation as R
from Panthera_lib import Panthera

# ==================== UDP 配置 ====================
PORT = 5005
FMT = "40f"
SIZE = struct.calcsize(FMT)

# ==================== 全局变量 ====================
left_hand_data = None
right_hand_data = None
data_lock = threading.Lock()
running = True

# 控制参数
target_position = None
target_rotation = None
control_activated = False

# 上一次手柄位姿（用于计算帧间增量）
last_hand_pos = None
last_hand_rot = None

# 当前手柄位姿（用于显示）
current_hand_pos_robot = None

# 控制缩放因子
position_scale = 1.0
rot_scale = 0.8

# ==================== VR 数据处理函数 ====================

def fmt_hand(name, vals):
    """格式化手柄数据"""
    return {
        "name": name,
        "seq": int(vals[0]),
        "hand_id": int(vals[1]),
        "pose_valid": int(vals[2]),
        "pos": tuple(round(x, 4) for x in vals[3:6]),
        "rot": tuple(round(x, 4) for x in vals[6:10]),
        "thumbstick": tuple(round(x, 4) for x in vals[10:12]),
        "trigger": round(vals[12], 4),
        "squeeze": round(vals[13], 4),
        "thumbstick_click": int(vals[14]),
        "primary": int(vals[15]),
        "secondary": int(vals[16]),
        "trigger_pressed": int(vals[17]),
        "squeeze_pressed": int(vals[18]),
    }


def transform_hand_to_robot_coords(hand_pos, hand_rot_matrix):
    """将VR手柄坐标系转换为机械臂坐标系

    映射关系：vr(x, y, z) -> arm(-y, z, -x)
    """
    robot_pos = np.array([
        -hand_pos[2],
        -hand_pos[0],
        hand_pos[1]
    ])

    T = np.array([
        [0, 0,  1],
        [1,  0,  0],
        [0, -1,  0]
    ])

    robot_rot_matrix = T @ hand_rot_matrix @ T.T

    return robot_pos, robot_rot_matrix


def is_hand_data_valid(hand_info):
    """检查手柄数据是否有效"""
    if hand_info is None:
        return False

    if hand_info['pose_valid'] == 0:
        return False

    pos = np.array(hand_info['pos'])
    if np.linalg.norm(pos) < 0.001:
        return False

    rot = np.array(hand_info['rot'])
    norm = np.linalg.norm(rot)
    if norm < 0.01:
        return False

    return True


def is_hand_data_all_zero(hand_info):
    """检查手柄数据是否全为0（手柄未被检测到）

    检查位置、旋转、按钮等所有数据是否都为0
    如果全为0，说明手柄未被检测到或连接丢失
    """
    if hand_info is None:
        return True

    # 检查位置是否全为0
    pos = np.array(hand_info['pos'])
    if np.linalg.norm(pos) > 0.001:
        return False

    # 检查旋转四元数是否全为0
    rot = np.array(hand_info['rot'])
    if np.linalg.norm(rot) > 0.001:
        return False

    # 检查按钮和摇杆是否全为0
    if (abs(hand_info['trigger']) > 0.001 or
        abs(hand_info['squeeze']) > 0.001 or
        abs(hand_info['thumbstick'][0]) > 0.001 or
        abs(hand_info['thumbstick'][1]) > 0.001):
        return False

    return True


def quaternion_to_rotation_matrix(q):
    """将四元数 (x, y, z, w) 转换为 3x3 旋转矩阵"""
    rotation = R.from_quat([q[0], q[1], q[2], q[3]])
    return rotation.as_matrix()


def rotation_matrix_to_4x4(R_mat, t):
    """将 3x3 旋转矩阵和位置向量组合成 4x4 齐次变换矩阵"""
    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3] = t
    return T


def udp_receiver_thread():
    """UDP 接收线程"""
    global left_hand_data, right_hand_data, running

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PORT))

    print(f"UDP 接收器启动，监听端口: {PORT}")

    while running:
        try:
            data, addr = sock.recvfrom(4096)

            if len(data) != SIZE:
                continue

            vals = struct.unpack(FMT, data)

            left = vals[:20]
            right = vals[20:]

            left_info = fmt_hand("LEFT", left)
            right_info = fmt_hand("RIGHT", right)

            with data_lock:
                left_hand_data = left_info
                right_hand_data = right_info

        except Exception as e:
            if running:
                print(f"UDP 接收错误: {e}")

    sock.close()
    print("UDP 接收器已停止")


# ==================== 机械臂控制函数 ====================

def move_to_safe_position(robot):
    """启动时缓慢移动到安全位置"""
    print("\n" + "=" * 60)
    print("正在移动到安全位置...")
    print("=" * 60)

    # 定义安全位置（关节空间）
    safe_joint_pos = np.array([0.0, 0.6, 0.6, 0.0, 0.0, 0.0])

    # 先刷新状态
    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    time.sleep(0.3)

    # 获取当前位置
    current_pos = robot.get_current_pos()

    print(f"当前位置: {current_pos}")
    print(f"目标位置: {safe_joint_pos}")

    # 使用插值缓慢移动到安全位置（2秒内完成）
    steps = 40
    duration = 2.0  # 总时长2秒
    dt = duration / steps

    for i in range(steps + 1):
        alpha = i / steps
        interp_pos = (1 - alpha) * current_pos + alpha * safe_joint_pos

        # 计算每一步的速度
        if i < steps:
            next_alpha = (i + 1) / steps
            next_pos = (1 - next_alpha) * current_pos + next_alpha * safe_joint_pos
            vel = (next_pos - interp_pos) / dt
        else:
            vel = np.zeros(6)

        # 限制速度
        max_vel = np.array([1.0, 1.0, 1.0, 1.0, 1.5, 1.5])
        vel = np.clip(vel, -max_vel, max_vel)

        # 使用位置速度控制
        robot.Joint_Pos_Vel(interp_pos.tolist(), vel.tolist())
        time.sleep(dt)

    # 最终确认到达安全位置
    success = robot.Joint_Pos_Vel(safe_joint_pos.tolist(), [0.0] * robot.motor_count, iswait=True)

    if success:
        print("✓ 已到达安全位置")
    else:
        print("✗ 移动到安全位置失败")
        return False

    time.sleep(0.5)
    return True


def sync_targets_to_current_state(robot):
    """将当前机械臂状态同步为新的笛卡尔目标。"""
    global target_position, target_rotation, control_activated
    global last_hand_pos, last_hand_rot

    current_joint_pos = robot.get_current_pos()
    current_fk = robot.forward_kinematics(current_joint_pos)
    target_position = current_fk['position'].copy()
    target_rotation = current_fk['rotation'].copy()
    control_activated = False
    last_hand_pos = None
    last_hand_rot = None
    return current_joint_pos


def execute_smooth_joint_reset(
    robot,
    target_joint_pos,
    kp,
    kd,
    Fc,
    Fv,
    vel_threshold,
    gripper_pos,
    gripper_vel,
    gripper_kp,
    gripper_kd,
    duration=2.0,
    control_dt=0.01,
    label="复位",
):
    """使用平滑七次轨迹 + MIT 重力/摩擦前馈执行关节复位。"""
    global running

    print("\n" + "=" * 60)
    print(f"正在执行平滑{label}: {np.round(target_joint_pos, 3)}")
    print("=" * 60)

    start_joint_pos = sync_targets_to_current_state(robot)
    target_joint_pos = np.asarray(target_joint_pos, dtype=np.float64)
    tau_limit = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0], dtype=np.float64)
    start_time = time.time()

    while running:
        elapsed = time.time() - start_time
        q_des, v_des, _ = robot.septic_interpolation(
            start_joint_pos,
            target_joint_pos,
            duration,
            elapsed,
        )

        current_joint_vel = robot.get_current_vel()
        gravity_torque = robot.get_Gravity(q_des)
        friction_torque = robot.get_friction_compensation(
            current_joint_vel, Fc, Fv, vel_threshold
        )
        feedforward_torque = gravity_torque + friction_torque
        feedforward_torque = np.clip(feedforward_torque, -tau_limit, tau_limit)

        robot.pos_vel_tqe_kp_kd(
            pos=q_des.tolist(),
            vel=v_des.tolist(),
            tqe=feedforward_torque.tolist(),
            kp=kp,
            kd=kd
        )
        robot.gripper_control_MIT(gripper_pos, gripper_vel, 0.0, gripper_kp, gripper_kd)

        if elapsed >= duration:
            break
        time.sleep(control_dt)

    synced_joint_pos = sync_targets_to_current_state(robot)
    err = np.max(np.abs(synced_joint_pos - target_joint_pos))
    print(f"✓ 平滑{label}完成，最大关节误差: {err:.3f} rad")
    return True


# ==================== 主程序 ====================

def main():
    global target_position, target_rotation, control_activated, running
    global last_hand_pos, last_hand_rot, current_hand_pos_robot

    print("==" * 60)
    print("Panthera-HT 机械臂 Quest 手柄笛卡尔空间控制")
    print("控制模式: MIT模式(PD控制 + 重力/摩擦前馈)")
    print("=" * 60)

    # 启动 UDP 接收线程
    udp_thread = threading.Thread(target=udp_receiver_thread, daemon=True)
    udp_thread.start()

    print("\n等待 Quest 手柄连接...")
    connection_timeout = 10  # 10秒超时
    start_time = time.time()
    while right_hand_data is None:
        time.sleep(0.1)
        if time.time() - start_time > connection_timeout:
            print("警告: 未在规定时间内检测到手柄连接，继续运行...")
            break
    print("已连接到 Quest 手柄")

    # 初始化机械臂
    print("\n初始化 Panthera-HT 机械臂...")
    robot = Panthera()

    # 检查是否有 Pinocchio 模型
    if robot.model is None:
        print("错误：未找到 Pinocchio 模型，无法计算逆运动学")
        running = False
        return

    # 移动到安全位置
    if not move_to_safe_position(robot):
        print("初始化失败，退出程序")
        running = False
        return

    # 获取当前关节位置
    current_joint_pos = robot.get_current_pos()

    # 计算当前末端位姿
    current_fk = robot.forward_kinematics()
    target_position = current_fk['position'].copy()
    target_rotation = current_fk['rotation'].copy()

    print(f"\n初始位置: [{target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f}] m")

    print("\n" + "=" * 60)
    print("控制说明：")
    print("  按下右手柄 Squeeze（侧键）: 激活控制（必须按下）")
    print("  移动手柄位置: 控制末端位置")
    print("  旋转手柄姿态: 控制末端姿态")
    print("  按住 Squeeze + 按下 Trigger（扳机键）: 根据按下程度控制夹爪")
    print("  未按下 Squeeze 时操作 Trigger: 不触发夹爪动作")
    print("  按下 A 键（Primary）: 复位到初始关节角")
    print("  松开 Squeeze: 停止运动，保持当前位置")
    print("  Ctrl+C: 退出程序")
    print("=" * 60)
    print("\n控制参数:")
    print(f"  位置缩放因子: {position_scale}")
    print(f"  旋转缩放因子: {rot_scale}")
    print("\n开始控制，请小心操作！\n")

    # 控制参数
    control_rate = 0.01  # 100Hz 控制频率

    # MIT 控制参数（参考 7_keyboard_cartesian_pos_control.py）
    kp = [30.0, 50.0, 60.0, 25.0, 15.0, 10.0]  # 位置增益（PD控制的位置刚度）
    kd = [3.0, 5.0, 6.0, 2.5, 1.5, 1.0]        # 阻尼增益（速度阻尼）

    # 摩擦补偿参数
    Fc = np.array([0.20, 0.15, 0.15, 0.15, 0.04, 0.04])
    Fv = np.array([0.06, 0.06, 0.06, 0.03, 0.02, 0.02])
    vel_threshold = 0.02

    # 上一次有效的关节位置（用于 IK 初始化）
    last_valid_joint_pos = current_joint_pos.copy()
    initial_joint_pos = current_joint_pos.copy()

    # Squeeze 按键阈值
    squeeze_threshold = 0.5

    # 打印计数器
    print_count = 0
    last_primary_pressed = 0
    reset_move_duration = 2.0

    # 夹爪控制参数
    gripper_open_pos = 1.6   # 夹爪完全打开位置
    gripper_close_pos = 0.0  # 夹爪完全闭合位置
    gripper_vel = 0.0        # 夹爪目标速度（MIT模式下通常为0）
    gripper_kp = 3.0        # 夹爪位置增益
    gripper_kd = 0.30         # 夹爪阻尼增益
    last_gripper_pos = gripper_open_pos  # 记录上次夹爪位置

    try:
        while running:
            # 获取手柄数据
            with data_lock:
                if right_hand_data is not None:
                    right_info = right_hand_data.copy()
                else:
                    right_info = None

            if right_info is None:
                current_hand_pos_robot = None
                if control_activated:
                    print("\n控制已停止 - 未收到手柄数据")
                control_activated = False
                last_hand_pos = None
                last_hand_rot = None
                last_primary_pressed = 0
            elif is_hand_data_all_zero(right_info):
                current_hand_pos_robot = None
                if control_activated:
                    print("\n控制已停止 - 手柄未检测到")
                control_activated = False
                last_hand_pos = None
                last_hand_rot = None
                gripper_tqe = 0.0
                robot.gripper_control_MIT(last_gripper_pos, gripper_vel, gripper_tqe, gripper_kp, gripper_kd)
                last_primary_pressed = 0
            elif is_hand_data_valid(right_info):
                # 手柄数据有效，正常控制
                # 获取手柄输入
                squeeze_value = right_info['squeeze']
                trigger_value = right_info['trigger']
                primary_pressed = right_info['primary']

                hand_pos_raw = np.array(right_info['pos'])
                hand_rot_quat = right_info['rot']

                # 转换手柄坐标系到机械臂坐标系
                hand_pos_robot, hand_rot_robot = transform_hand_to_robot_coords(
                    hand_pos_raw,
                    quaternion_to_rotation_matrix(hand_rot_quat)
                )

                current_hand_pos_robot = hand_pos_robot

                # 夹爪控制：仅在按下 Squeeze 时，根据 trigger 值控制夹爪位置
                # trigger: 0.0 (未按下) -> 夹爪打开 (1.6)
                # trigger: 1.0 (完全按下) -> 夹爪闭合 (0.0)
                if squeeze_value > squeeze_threshold:
                    gripper_target_pos = gripper_open_pos - trigger_value * (gripper_open_pos - gripper_close_pos)
                else:
                    gripper_target_pos = last_gripper_pos
                gripper_tqe = 0.0  # MIT模式下的前馈力矩
                robot.gripper_control_MIT(gripper_target_pos, gripper_vel, gripper_tqe, gripper_kp, gripper_kd)
                last_gripper_pos = gripper_target_pos  # 更新上次夹爪位置

                if primary_pressed and not last_primary_pressed:
                    print("\n检测到 A 键按下，开始复位到初始关节角")
                    if execute_smooth_joint_reset(
                        robot=robot,
                        target_joint_pos=initial_joint_pos,
                        kp=kp,
                        kd=kd,
                        Fc=Fc,
                        Fv=Fv,
                        vel_threshold=vel_threshold,
                        gripper_pos=last_gripper_pos,
                        gripper_vel=gripper_vel,
                        gripper_kp=gripper_kp,
                        gripper_kd=gripper_kd,
                        duration=reset_move_duration,
                        control_dt=control_rate,
                        label="A键复位",
                    ):
                        last_valid_joint_pos = initial_joint_pos.copy()
                    last_primary_pressed = primary_pressed
                    time.sleep(0.05)
                    continue

                # 检查是否激活控制（必须按下 Squeeze）
                if squeeze_value > squeeze_threshold:
                    if not control_activated:
                        # 首次激活：记录当前手柄位姿作为基准
                        last_hand_pos = hand_pos_robot.copy()
                        last_hand_rot = hand_rot_robot.copy()
                        control_activated = True
                        print("\n控制已激活")
                        print(f"  Squeeze 值: {squeeze_value:.2f}")

                    # 计算手柄位姿增量
                    delta_hand_pos = hand_pos_robot - last_hand_pos
                    delta_hand_rot = hand_rot_robot @ last_hand_rot.T
                    delta_rotvec = R.from_matrix(delta_hand_rot).as_rotvec()
                    scaled_delta_hand_rot = R.from_rotvec(delta_rotvec * rot_scale).as_matrix()

                    # 更新目标位姿（应用缩放因子）
                    target_position = target_position + delta_hand_pos * position_scale
                    target_rotation = scaled_delta_hand_rot @ target_rotation

                    # 更新上一次手柄位姿
                    last_hand_pos = hand_pos_robot.copy()
                    last_hand_rot = hand_rot_robot.copy()

                else:
                    if control_activated:
                        print("\n控制已停止 - 保持当前位置")
                    control_activated = False
                    last_hand_pos = None
                    last_hand_rot = None

                # 使用逆运动学求解关节角度
                joint_pos = robot.inverse_kinematics(
                    target_position=target_position,
                    target_rotation=target_rotation,
                    init_q=last_valid_joint_pos,
                    max_iter=100,
                    eps=1e-3,
                    damping=1e-2,
                    adaptive_damping=True,
                    multi_init=False
                )

                if joint_pos is not None:
                    # 获取重力/摩擦补偿力矩
                    gravity_torque = robot.get_Gravity(joint_pos)
                    current_joint_vel = robot.get_current_vel()
                    friction_torque = robot.get_friction_compensation(
                        current_joint_vel, Fc, Fv, vel_threshold
                    )
                    feedforward_torque = gravity_torque + friction_torque

                    # 力矩限幅
                    tau_limit = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0])
                    feedforward_torque = np.clip(feedforward_torque, -tau_limit, tau_limit)

                    # 使用 MIT 模式 + 重力/摩擦前馈控制
                    # 目标速度设为零（位置控制）
                    target_vel = np.zeros(robot.motor_count)

                    success = robot.pos_vel_tqe_kp_kd(
                        pos=joint_pos.tolist(),
                        vel=target_vel.tolist(),
                        tqe=feedforward_torque.tolist(),
                        kp=kp,
                        kd=kd
                    )

                    if success:
                        last_valid_joint_pos = joint_pos.copy()
                last_primary_pressed = primary_pressed
            else:
                current_hand_pos_robot = None
                if control_activated:
                    print("\n控制已停止 - 手柄姿态无效")
                control_activated = False
                last_hand_pos = None
                last_hand_rot = None
                gripper_tqe = 0.0
                robot.gripper_control_MIT(last_gripper_pos, gripper_vel, gripper_tqe, gripper_kp, gripper_kd)
                last_primary_pressed = 0

            # 获取当前末端位姿（用于显示）
            current_joint_pos = robot.get_current_pos()
            current_fk = robot.forward_kinematics()
            current_pos = current_fk['position']

            # 定期打印状态信息
            print_count += 1
            if print_count % 20 == 0:
                control_status = "激活" if control_activated else "停止"

                if right_info is not None and is_hand_data_valid(right_info) and not is_hand_data_all_zero(right_info) and current_hand_pos_robot is not None:
                    h_pos = current_hand_pos_robot
                    h_squeeze = right_info['squeeze']
                    h_trigger = right_info['trigger']
                    data_status = "有效"
                elif right_info is not None and is_hand_data_all_zero(right_info):
                    h_pos = np.array([0, 0, 0])
                    h_squeeze = 0.0
                    h_trigger = 0.0
                    data_status = "未检测"
                else:
                    h_pos = np.array([0, 0, 0])
                    h_squeeze = 0.0
                    h_trigger = 0.0
                    data_status = "无效"

                print(f"\r控制: {control_status} | 数据: {data_status} | Squeeze: {h_squeeze:.2f} | Trigger: {h_trigger:.2f} | "
                      f"手柄: ({h_pos[0]:.2f}, {h_pos[1]:.2f}, {h_pos[2]:.2f}) | "
                      f"目标: [{target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f}] | "
                      f"当前: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]", end='')

            time.sleep(control_rate)

    except KeyboardInterrupt:
        print("\n\n程序被中断")
    except Exception as e:
        print(f"\n\n错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 安全停止机械臂
        print("\n\n安全停止机械臂...")
        current_pos = robot.get_current_pos()
        current_vel = robot.get_current_vel()
        gravity_torque = robot.get_Gravity(current_pos)
        friction_torque = robot.get_friction_compensation(current_vel, Fc, Fv, vel_threshold)
        hold_torque = gravity_torque + friction_torque
        tau_limit = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0])
        hold_torque = np.clip(hold_torque, -tau_limit, tau_limit)

        # 先使用 MIT 模式短暂保持当前位置，再回到零位
        robot.pos_vel_tqe_kp_kd(
            pos=current_pos.tolist(),
            vel=[0.0] * robot.motor_count,
            tqe=hold_torque.tolist(),
            kp=kp,  # 使用位置增益
            kd=kd
        )
        time.sleep(0.5)

        print("返回零位...")
        zero_pos = [0.0] * robot.motor_count
        execute_smooth_joint_reset(
            robot=robot,
            target_joint_pos=zero_pos,
            kp=kp,
            kd=kd,
            Fc=Fc,
            Fv=Fv,
            vel_threshold=vel_threshold,
            gripper_pos=last_gripper_pos,
            gripper_vel=gripper_vel,
            gripper_kp=gripper_kp,
            gripper_kd=gripper_kd,
            duration=2.5,
            control_dt=control_rate,
            label="退出回零",
        )

        running = False
        print("程序结束")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
