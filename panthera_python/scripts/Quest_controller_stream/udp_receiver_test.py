import socket
import struct
from datetime import datetime
import numpy as np
from scipy.spatial.transform import Rotation as R

PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", PORT))

print(f"UDP receiver listening on :{PORT}")

FMT = "40f"
SIZE = struct.calcsize(FMT)


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


def print_hand_info(hand_info):
    """以整齐的表格格式打印单手信息"""
    name = hand_info["name"]

    # 根据左右手选择不同的边框样式
    left_border = "┌─" if name == "LEFT" else "├─"
    right_border = "┐" if name == "LEFT" else "┤"

    print(f"  {left_border} {name} HAND {right_border}")
    print(f"  │  Seq:        {hand_info['seq']}")
    print(f"  │  Hand ID:    {hand_info['hand_id']}")
    print(f"  │  Pose Valid: {hand_info['pose_valid']}")

    # 位置信息
    pos = hand_info["pos"]
    print(f"  │  Position:")
    print(f"  │    X: {pos[0]:>10.4f}")
    print(f"  │    Y: {pos[1]:>10.4f}")
    print(f"  │    Z: {pos[2]:>10.4f}")

    # 旋转四元数
    rot = hand_info["rot"]
    print(f"  │  Rotation (Quaternion):")
    print(f"  │    X: {rot[0]:>10.4f}")
    print(f"  │    Y: {rot[1]:>10.4f}")
    print(f"  │    Z: {rot[2]:>10.4f}")
    print(f"  │    W: {rot[3]:>10.4f}")

    # 旋转欧拉角（只在四元数有效时计算）
    quat = np.array([rot[0], rot[1], rot[2], rot[3]])  # [x, y, z, w]
    quat_norm = np.linalg.norm(quat)
    if quat_norm > 0.01:  # 四元数有效
        rotation = R.from_quat(quat)
        euler = rotation.as_euler('XYZ', degrees=True)  # XYZ顺序，度数
        print(f"  │  Euler Angles (XYZ):")
        print(f"  │    X: {euler[0]:>10.2f}°")
        print(f"  │    Y: {euler[1]:>10.2f}°")
        print(f"  │    Z: {euler[2]:>10.2f}°")
    else:
        print(f"  │  Euler Angles (XYZ):")
        print(f"  │    (Invalid - quaternion norm is zero)")

    # 摇杆和扳机
    thumb = hand_info["thumbstick"]
    print(f"  │  Thumbstick: ({thumb[0]:.4f}, {thumb[1]:.4f})")
    print(f"  │  Trigger:    {hand_info['trigger']:.4f}")
    print(f"  │  Squeeze:    {hand_info['squeeze']:.4f}")

    # 按钮状态
    print(f"  │  Buttons:")
    print(f"  │    Thumbstick: {'ON ' if hand_info['thumbstick_click'] else 'OFF'}")
    print(f"  │    Primary:     {'ON ' if hand_info['primary'] else 'OFF'}")
    print(f"  │    Secondary:   {'ON ' if hand_info['secondary'] else 'OFF'}")
    print(f"  │    Trigger:     {'ON ' if hand_info['trigger_pressed'] else 'OFF'}")
    print(f"  │    Squeeze:     {'ON ' if hand_info['squeeze_pressed'] else 'OFF'}")

    # 底部边框
    bottom_left = "└" if name == "RIGHT" else "├"
    bottom_right = "┘" if name == "RIGHT" else "┤"
    print(f"  {bottom_left}─────────────────────{bottom_right}")


while True:
    data, addr = sock.recvfrom(4096)

    if len(data) != SIZE:
        print("unexpected packet size:", len(data), "expected:", SIZE)
        continue

    vals = struct.unpack(FMT, data)

    left = vals[:20]
    right = vals[20:]

    left_info = fmt_hand("LEFT", left)
    right_info = fmt_hand("RIGHT", right)

    # 打印分隔线和标题
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"\n{'='*60}")
    print(f"  📡 Source: {addr[0]}:{addr[1]}  |  Time: {timestamp}")
    print(f"{'─'*60}")

    # 打印左右手信息
    print_hand_info(left_info)
    print()
    print_hand_info(right_info)
