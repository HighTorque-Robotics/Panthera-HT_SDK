#!/usr/bin/env python3
"""
正运动学测试程序
实时获取并打印机器人末端位置和姿态
"""
import time
import numpy as np
from Panthera_lib import Panthera

def rotation_matrix_to_euler(R):
    """将旋转矩阵转换为欧拉角(ZYX顺序,单位：度）"""
    sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    
    singular = sy < 1e-6
    
    if not singular:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
    
    return np.degrees([x, y, z])

def print_matrix(matrix, title, precision=3):
    """格式化打印矩阵"""
    print(f"\n{title}:")
    for row in matrix:
        print("  [" + "  ".join([f"{val:8.{precision}f}" for val in row]) + "]")

def main():
    # 获取当前末端位置和姿态
    # 利用运控模式发送控制帧以读取电机反馈状态
    # robot.pos_vel_tqe_kp_kd([0.0]*6, [0.0]*6, [0.0]*6, [0.0]*6, [0.0]*6, iswait=False)
    robot.motor_send_cmd()
    current_angles = robot.get_current_pos()
    fk = robot.forward_kinematics(current_angles)
    
    if fk:
        print("\n" + "="*60)
        print("机械臂正运动学结果")
        print("="*60)
        
        # 显示关节角度
        joint_angles_deg = [np.degrees(angle) for angle in fk['joint_angles']]
        print(f"关节角度 (度): {[f'{a:7.2f}' for a in joint_angles_deg]}")
        
        # 显示末端位置
        pos = fk['position']
        print(f"末端位置 (m): x={pos[0]:8.4f}, y={pos[1]:8.4f}, z={pos[2]:8.4f}")
        
        # 显示旋转矩阵
        rotation_matrix = fk['rotation']
        print_matrix(rotation_matrix, "旋转矩阵 (R)")
        
        # 显示欧拉角
        euler_angles = rotation_matrix_to_euler(rotation_matrix)
        print(f"\n欧拉角 (度): Roll={euler_angles[0]:7.2f}, Pitch={euler_angles[1]:7.2f}, Yaw={euler_angles[2]:7.2f}")
        
        # 显示4x4变换矩阵
        print_matrix(fk['transform'], "4x4变换矩阵 (T)", precision=4)
    
    time.sleep(1.0)

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