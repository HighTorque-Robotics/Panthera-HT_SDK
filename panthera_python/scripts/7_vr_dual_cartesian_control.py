#!/usr/bin/env python3
"""
Panthera-HT 双臂 Quest VR 笛卡尔空间遥操作程序

控制映射:
    左手柄 -> Leader
    右手柄 -> Follower

控制说明:
    按住任一手柄 Squeeze: 激活对应机械臂控制
    移动/旋转手柄: 控制对应机械臂末端位姿
    按住 Squeeze + Trigger: 控制对应夹爪开合
    左手柄 Primary(X) / 右手柄 Primary(A): 对应机械臂复位到启动姿态
    松开 Squeeze: 停止对应机械臂运动并保持当前位置
    Ctrl+C: 退出程序
"""

import os
import socket
import struct
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

from Panthera_lib import Panthera


PORT = 5005
FMT = "40f"
SIZE = struct.calcsize(FMT)

left_hand_data = None
right_hand_data = None
data_lock = threading.Lock()
running = True

position_scale = 1.0
rot_scale = 0.8

squeeze_threshold = 0.5
control_rate = 0.01
reset_move_duration = 2.0

kp = [30.0, 50.0, 60.0, 25.0, 15.0, 10.0]
kd = [3.0, 5.0, 6.0, 2.5, 1.5, 1.0]
Fc = np.array([0.20, 0.15, 0.15, 0.15, 0.04, 0.04])
Fv = np.array([0.06, 0.06, 0.06, 0.03, 0.02, 0.02])
vel_threshold = 0.02
tau_limit = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0], dtype=np.float64)

gripper_open_pos = 1.6
gripper_close_pos = 0.0
gripper_vel = 0.0
gripper_kp = 3.0
gripper_kd = 0.30


def fmt_hand(name, vals):
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
    robot_pos = np.array([
        -hand_pos[2],
        -hand_pos[0],
        hand_pos[1],
    ])

    transform = np.array([
        [0, 0, 1],
        [1, 0, 0],
        [0, -1, 0],
    ])
    robot_rot_matrix = transform @ hand_rot_matrix @ transform.T
    return robot_pos, robot_rot_matrix


def is_hand_data_valid(hand_info):
    if hand_info is None or hand_info["pose_valid"] == 0:
        return False

    if np.linalg.norm(np.array(hand_info["pos"])) < 0.001:
        return False

    if np.linalg.norm(np.array(hand_info["rot"])) < 0.01:
        return False

    return True


def is_hand_data_all_zero(hand_info):
    if hand_info is None:
        return True

    if np.linalg.norm(np.array(hand_info["pos"])) > 0.001:
        return False

    if np.linalg.norm(np.array(hand_info["rot"])) > 0.001:
        return False

    if (
        abs(hand_info["trigger"]) > 0.001
        or abs(hand_info["squeeze"]) > 0.001
        or abs(hand_info["thumbstick"][0]) > 0.001
        or abs(hand_info["thumbstick"][1]) > 0.001
    ):
        return False

    return True


def quaternion_to_rotation_matrix(q):
    return R.from_quat([q[0], q[1], q[2], q[3]]).as_matrix()


def udp_receiver_thread():
    global left_hand_data, right_hand_data, running

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PORT))
    print(f"UDP 接收器启动，监听端口: {PORT}")

    while running:
        try:
            data, _ = sock.recvfrom(4096)
            if len(data) != SIZE:
                continue

            vals = struct.unpack(FMT, data)
            left_info = fmt_hand("LEFT", vals[:20])
            right_info = fmt_hand("RIGHT", vals[20:])

            with data_lock:
                left_hand_data = left_info
                right_hand_data = right_info
        except Exception as exc:
            if running:
                print(f"UDP 接收错误: {exc}")

    sock.close()
    print("UDP 接收器已停止")


def move_to_safe_position(robot, label):
    print("\n" + "=" * 60)
    print(f"{label} 正在移动到安全位置...")
    print("=" * 60)

    safe_joint_pos = np.array([0.0, 0.6, 0.6, 0.0, 0.0, 0.0])
    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    time.sleep(0.3)

    current_pos = robot.get_current_pos()
    steps = 40
    duration = 2.0
    dt = duration / steps

    for i in range(steps + 1):
        alpha = i / steps
        interp_pos = (1 - alpha) * current_pos + alpha * safe_joint_pos
        if i < steps:
            next_alpha = (i + 1) / steps
            next_pos = (1 - next_alpha) * current_pos + next_alpha * safe_joint_pos
            vel = (next_pos - interp_pos) / dt
        else:
            vel = np.zeros(6)

        vel = np.clip(vel, -1.5, 1.5)
        robot.Joint_Pos_Vel(interp_pos.tolist(), vel.tolist())
        time.sleep(dt)

    final_cmd_ok = robot.Joint_Pos_Vel(
        safe_joint_pos.tolist(),
        [0.0] * robot.motor_count,
        iswait=False,
    )
    actual_joint_pos = robot.get_current_pos()
    max_err = np.max(np.abs(actual_joint_pos - safe_joint_pos))

    if final_cmd_ok:
        print(f"{label} ✓ 已到达安全位置，最大误差: {max_err:.3f} rad")
    else:
        print(f"{label} ! 安全位等待超时，当前最大误差: {max_err:.3f} rad")
        print(f"{label} 当前关节: {np.round(actual_joint_pos, 3)}")
        print(f"{label} 目标关节: {np.round(safe_joint_pos, 3)}")
        print(f"{label} 将从当前位置继续初始化，不中止双臂控制")
    time.sleep(0.5)
    return True


def move_arms_to_safe_position(arm_entries):
    print("\n" + "=" * 60)
    print("双臂同时移动到安全位置...")
    print("=" * 60)

    safe_joint_pos = np.array([0.0, 0.6, 0.6, 0.0, 0.0, 0.0])
    steps = 40
    duration = 2.0
    dt = duration / steps

    start_positions = {}
    for robot, label in arm_entries:
        print(f"{label} 准备进入安全位置")
        robot.send_get_motor_state_cmd()
        robot.motor_send_cmd()

    time.sleep(0.3)

    for robot, label in arm_entries:
        start_positions[label] = robot.get_current_pos()

    for i in range(steps + 1):
        alpha = i / steps
        for robot, label in arm_entries:
            current_pos = start_positions[label]
            interp_pos = (1 - alpha) * current_pos + alpha * safe_joint_pos
            if i < steps:
                next_alpha = (i + 1) / steps
                next_pos = (1 - next_alpha) * current_pos + next_alpha * safe_joint_pos
                vel = (next_pos - interp_pos) / dt
            else:
                vel = np.zeros(6)

            vel = np.clip(vel, -1.5, 1.5)
            robot.Joint_Pos_Vel(interp_pos.tolist(), vel.tolist(), iswait=False)
        time.sleep(dt)

    for robot, label in arm_entries:
        final_cmd_ok = robot.Joint_Pos_Vel(
            safe_joint_pos.tolist(),
            [0.0] * robot.motor_count,
            iswait=False,
        )
        actual_joint_pos = robot.get_current_pos()
        max_err = np.max(np.abs(actual_joint_pos - safe_joint_pos))
        if final_cmd_ok:
            print(f"{label} ✓ 安全位指令已发送，当前最大误差: {max_err:.3f} rad")
        else:
            print(f"{label} ! 安全位指令发送失败，当前最大误差: {max_err:.3f} rad")
            print(f"{label} 当前关节: {np.round(actual_joint_pos, 3)}")
            print(f"{label} 目标关节: {np.round(safe_joint_pos, 3)}")

    time.sleep(0.5)
    return True


def create_arm_state(robot, label):
    current_joint_pos = robot.get_current_pos()
    current_fk = robot.forward_kinematics(current_joint_pos)
    return {
        "label": label,
        "robot": robot,
        "target_position": current_fk["position"].copy(),
        "target_rotation": current_fk["rotation"].copy(),
        "control_activated": False,
        "last_hand_pos": None,
        "last_hand_rot": None,
        "last_valid_joint_pos": current_joint_pos.copy(),
        "initial_joint_pos": current_joint_pos.copy(),
        "last_primary_pressed": 0,
        "last_gripper_pos": gripper_open_pos,
        "current_hand_pos_robot": None,
    }


def sync_targets_to_current_state(arm_state):
    current_joint_pos = arm_state["robot"].get_current_pos()
    current_fk = arm_state["robot"].forward_kinematics(current_joint_pos)
    arm_state["target_position"] = current_fk["position"].copy()
    arm_state["target_rotation"] = current_fk["rotation"].copy()
    arm_state["control_activated"] = False
    arm_state["last_hand_pos"] = None
    arm_state["last_hand_rot"] = None
    return current_joint_pos


def execute_smooth_joint_reset(arm_state, target_joint_pos, duration=2.0, label="复位"):
    global running

    robot = arm_state["robot"]
    print("\n" + "=" * 60)
    print(f"{arm_state['label']} 正在执行{label}: {np.round(target_joint_pos, 3)}")
    print("=" * 60)

    start_joint_pos = sync_targets_to_current_state(arm_state)
    target_joint_pos = np.asarray(target_joint_pos, dtype=np.float64)
    start_time = time.time()

    while running:
        elapsed = time.time() - start_time
        q_des, v_des, _ = robot.septic_interpolation(start_joint_pos, target_joint_pos, duration, elapsed)
        current_joint_vel = robot.get_current_vel()
        gravity_torque = robot.get_Gravity(q_des)
        friction_torque = robot.get_friction_compensation(current_joint_vel, Fc, Fv, vel_threshold)
        feedforward_torque = np.clip(gravity_torque + friction_torque, -tau_limit, tau_limit)

        robot.pos_vel_tqe_kp_kd(
            pos=q_des.tolist(),
            vel=v_des.tolist(),
            tqe=feedforward_torque.tolist(),
            kp=kp,
            kd=kd,
        )
        robot.gripper_control_MIT(
            arm_state["last_gripper_pos"],
            gripper_vel,
            0.0,
            gripper_kp,
            gripper_kd,
        )

        if elapsed >= duration:
            break
        time.sleep(control_rate)

    synced_joint_pos = sync_targets_to_current_state(arm_state)
    arm_state["last_valid_joint_pos"] = synced_joint_pos.copy()
    err = np.max(np.abs(synced_joint_pos - target_joint_pos))
    print(f"{arm_state['label']} ✓ {label}完成，最大关节误差: {err:.3f} rad")
    return True


def execute_dual_smooth_joint_reset(arm_states, target_joint_positions, duration=2.0, label="复位"):
    global running

    print("\n" + "=" * 60)
    print(f"双臂正在执行同步{label}")
    print("=" * 60)

    start_joint_positions = {}
    for arm_state in arm_states:
        start_joint_positions[arm_state["label"]] = sync_targets_to_current_state(arm_state)

    target_map = {
        arm_state["label"]: np.asarray(target_joint_pos, dtype=np.float64)
        for arm_state, target_joint_pos in zip(arm_states, target_joint_positions)
    }
    start_time = time.time()

    while running:
        elapsed = time.time() - start_time
        for arm_state in arm_states:
            robot = arm_state["robot"]
            label_name = arm_state["label"]
            q_des, v_des, _ = robot.septic_interpolation(
                start_joint_positions[label_name],
                target_map[label_name],
                duration,
                elapsed,
            )
            current_joint_vel = robot.get_current_vel()
            gravity_torque = robot.get_Gravity(q_des)
            friction_torque = robot.get_friction_compensation(current_joint_vel, Fc, Fv, vel_threshold)
            feedforward_torque = np.clip(gravity_torque + friction_torque, -tau_limit, tau_limit)

            robot.pos_vel_tqe_kp_kd(
                pos=q_des.tolist(),
                vel=v_des.tolist(),
                tqe=feedforward_torque.tolist(),
                kp=kp,
                kd=kd,
            )
            robot.gripper_control_MIT(
                arm_state["last_gripper_pos"],
                gripper_vel,
                0.0,
                gripper_kp,
                gripper_kd,
            )

        if elapsed >= duration:
            break
        time.sleep(control_rate)

    for arm_state in arm_states:
        label_name = arm_state["label"]
        synced_joint_pos = sync_targets_to_current_state(arm_state)
        arm_state["last_valid_joint_pos"] = synced_joint_pos.copy()
        err = np.max(np.abs(synced_joint_pos - target_map[label_name]))
        print(f"{label_name} ✓ 同步{label}完成，最大关节误差: {err:.3f} rad")

    return True


def hold_current_pose(arm_state):
    robot = arm_state["robot"]
    current_pos = robot.get_current_pos()
    current_vel = robot.get_current_vel()
    gravity_torque = robot.get_Gravity(current_pos)
    friction_torque = robot.get_friction_compensation(current_vel, Fc, Fv, vel_threshold)
    hold_torque = np.clip(gravity_torque + friction_torque, -tau_limit, tau_limit)
    robot.pos_vel_tqe_kp_kd(
        pos=current_pos.tolist(),
        vel=[0.0] * robot.motor_count,
        tqe=hold_torque.tolist(),
        kp=kp,
        kd=kd,
    )


def process_arm_control(arm_state, hand_info):
    robot = arm_state["robot"]

    if hand_info is None:
        arm_state["current_hand_pos_robot"] = None
        arm_state["control_activated"] = False
        arm_state["last_hand_pos"] = None
        arm_state["last_hand_rot"] = None
        arm_state["last_primary_pressed"] = 0
        return

    if is_hand_data_all_zero(hand_info) or not is_hand_data_valid(hand_info):
        arm_state["current_hand_pos_robot"] = None
        arm_state["control_activated"] = False
        arm_state["last_hand_pos"] = None
        arm_state["last_hand_rot"] = None
        robot.gripper_control_MIT(
            arm_state["last_gripper_pos"],
            gripper_vel,
            0.0,
            gripper_kp,
            gripper_kd,
        )
        arm_state["last_primary_pressed"] = 0
        return

    squeeze_value = hand_info["squeeze"]
    trigger_value = hand_info["trigger"]
    primary_pressed = hand_info["primary"]

    hand_pos_robot, hand_rot_robot = transform_hand_to_robot_coords(
        np.array(hand_info["pos"]),
        quaternion_to_rotation_matrix(hand_info["rot"]),
    )
    arm_state["current_hand_pos_robot"] = hand_pos_robot

    if squeeze_value > squeeze_threshold:
        gripper_target_pos = gripper_open_pos - trigger_value * (gripper_open_pos - gripper_close_pos)
    else:
        gripper_target_pos = arm_state["last_gripper_pos"]

    robot.gripper_control_MIT(gripper_target_pos, gripper_vel, 0.0, gripper_kp, gripper_kd)
    arm_state["last_gripper_pos"] = gripper_target_pos

    if primary_pressed and not arm_state["last_primary_pressed"]:
        print(f"\n{arm_state['label']} 检测到复位按键，开始复位到启动姿态")
        execute_smooth_joint_reset(
            arm_state,
            arm_state["initial_joint_pos"],
            duration=reset_move_duration,
            label="按键复位",
        )
        arm_state["last_primary_pressed"] = primary_pressed
        time.sleep(0.05)
        return

    if squeeze_value > squeeze_threshold:
        if not arm_state["control_activated"]:
            arm_state["last_hand_pos"] = hand_pos_robot.copy()
            arm_state["last_hand_rot"] = hand_rot_robot.copy()
            arm_state["control_activated"] = True
            print(f"\n{arm_state['label']} 控制已激活")

        delta_hand_pos = hand_pos_robot - arm_state["last_hand_pos"]
        delta_hand_rot = hand_rot_robot @ arm_state["last_hand_rot"].T
        delta_rotvec = R.from_matrix(delta_hand_rot).as_rotvec()
        scaled_delta_rot = R.from_rotvec(delta_rotvec * rot_scale).as_matrix()

        arm_state["target_position"] = arm_state["target_position"] + delta_hand_pos * position_scale
        arm_state["target_rotation"] = scaled_delta_rot @ arm_state["target_rotation"]

        arm_state["last_hand_pos"] = hand_pos_robot.copy()
        arm_state["last_hand_rot"] = hand_rot_robot.copy()
    else:
        arm_state["control_activated"] = False
        arm_state["last_hand_pos"] = None
        arm_state["last_hand_rot"] = None

    joint_pos = robot.inverse_kinematics(
        target_position=arm_state["target_position"],
        target_rotation=arm_state["target_rotation"],
        init_q=arm_state["last_valid_joint_pos"],
        max_iter=100,
        eps=1e-3,
        damping=1e-2,
        adaptive_damping=True,
        multi_init=False,
    )

    if joint_pos is not None:
        current_joint_vel = robot.get_current_vel()
        gravity_torque = robot.get_Gravity(joint_pos)
        friction_torque = robot.get_friction_compensation(current_joint_vel, Fc, Fv, vel_threshold)
        feedforward_torque = np.clip(gravity_torque + friction_torque, -tau_limit, tau_limit)
        target_vel = np.zeros(robot.motor_count)

        success = robot.pos_vel_tqe_kp_kd(
            pos=joint_pos.tolist(),
            vel=target_vel.tolist(),
            tqe=feedforward_torque.tolist(),
            kp=kp,
            kd=kd,
        )
        if success:
            arm_state["last_valid_joint_pos"] = joint_pos.copy()

    arm_state["last_primary_pressed"] = primary_pressed


def format_arm_status(arm_state, hand_info):
    robot = arm_state["robot"]
    current_fk = robot.forward_kinematics()
    current_pos = current_fk["position"]
    control_status = "激活" if arm_state["control_activated"] else "停止"

    if hand_info is not None and is_hand_data_valid(hand_info) and not is_hand_data_all_zero(hand_info):
        h_pos = arm_state["current_hand_pos_robot"] if arm_state["current_hand_pos_robot"] is not None else np.zeros(3)
        squeeze_value = hand_info["squeeze"]
        trigger_value = hand_info["trigger"]
        data_status = "有效"
    elif hand_info is not None and is_hand_data_all_zero(hand_info):
        h_pos = np.zeros(3)
        squeeze_value = 0.0
        trigger_value = 0.0
        data_status = "未检测"
    else:
        h_pos = np.zeros(3)
        squeeze_value = 0.0
        trigger_value = 0.0
        data_status = "无效"

    return (
        f"{arm_state['label']}[{control_status}/{data_status}] "
        f"S:{squeeze_value:.2f} T:{trigger_value:.2f} "
        f"手柄({h_pos[0]:.2f},{h_pos[1]:.2f},{h_pos[2]:.2f}) "
        f"目标[{arm_state['target_position'][0]:.3f},{arm_state['target_position'][1]:.3f},{arm_state['target_position'][2]:.3f}] "
        f"当前[{current_pos[0]:.3f},{current_pos[1]:.3f},{current_pos[2]:.3f}]"
    )


def main():
    global running

    print("=" * 60)
    print("Panthera-HT 双臂 Quest VR 笛卡尔空间遥操作")
    print("左手柄 -> Leader | 右手柄 -> Follower")
    print("=" * 60)

    udp_thread = threading.Thread(target=udp_receiver_thread, daemon=True)
    udp_thread.start()

    print("\n等待 Quest 手柄连接...")
    start_time = time.time()
    while left_hand_data is None and right_hand_data is None:
        time.sleep(0.1)
        if time.time() - start_time > 10.0:
            print("警告: 10 秒内未检测到手柄数据，继续初始化机械臂")
            break

    script_dir = os.path.dirname(os.path.abspath(__file__))
    leader = Panthera(os.path.join(script_dir, "../robot_param/Leader.yaml"))
    follower = Panthera(os.path.join(script_dir, "../robot_param/Follower.yaml"))

    if leader.model is None or follower.model is None:
        print("错误：Pinocchio 模型加载失败，无法进行笛卡尔控制")
        running = False
        return

    if not move_arms_to_safe_position([(leader, "Leader"), (follower, "Follower")]):
        running = False
        return

    leader_state = create_arm_state(leader, "Leader")
    follower_state = create_arm_state(follower, "Follower")

    print("\n控制说明:")
    print("  左手柄控制 Leader，右手柄控制 Follower")
    print("  按住对应手柄 Squeeze 后，移动/旋转手柄控制末端")
    print("  按住 Squeeze + Trigger 控制对应夹爪")
    print("  左 Primary(X) / 右 Primary(A) 复位对应机械臂")
    print("  Ctrl+C 退出")
    print(f"  位置缩放因子: {position_scale}")
    print(f"  姿态缩放因子: {rot_scale}")
    print("\n开始控制，请注意安全。\n")

    print_count = 0

    try:
        while running:
            with data_lock:
                left_info = left_hand_data.copy() if left_hand_data is not None else None
                right_info = right_hand_data.copy() if right_hand_data is not None else None

            process_arm_control(leader_state, left_info)
            process_arm_control(follower_state, right_info)

            print_count += 1
            if print_count % 20 == 0:
                left_status = format_arm_status(leader_state, left_info)
                right_status = format_arm_status(follower_state, right_info)
                print(f"\r{left_status} | {right_status}", end="")

            time.sleep(control_rate)

    except KeyboardInterrupt:
        print("\n\n程序被中断")
    except Exception as exc:
        print(f"\n\n错误: {exc}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n\n安全停止双臂...")
        for arm_state in (leader_state, follower_state):
            try:
                hold_current_pose(arm_state)
            except Exception as exc:
                print(f"{arm_state['label']} 保持当前位置失败: {exc}")
        time.sleep(0.5)

        try:
            execute_dual_smooth_joint_reset(
                [leader_state, follower_state],
                [
                    np.zeros(leader_state["robot"].motor_count),
                    np.zeros(follower_state["robot"].motor_count),
                ],
                duration=2.5,
                label="退出回零",
            )
        except Exception as exc:
            print(f"双臂同步回零失败: {exc}")

        running = False
        print("程序结束")


if __name__ == "__main__":
    main()
