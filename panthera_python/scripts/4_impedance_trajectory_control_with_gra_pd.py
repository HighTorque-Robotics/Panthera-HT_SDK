#!/usr/bin/env python3
"""
基于轨迹的阻抗控制程序
使用七次多项式插值 + 重力补偿前馈的阻抗控制
在完整的阻抗方程中，令期望惯性矩阵Md=M(q)，同时在低速情况令q_des_dot,q_des_ddot=0
得到t=g(q)+tor_impedance
"""
import time
import numpy as np
from Panthera_lib import Panthera


def precise_sleep(duration):
    """高精度延时函数"""
    if duration <= 0:
        return
    
    end_time = time.perf_counter() + duration
    
    # 大部分时间用sleep（留1ms余量）
    if duration > 0.001:
        time.sleep(duration - 0.001)
    
    # 最后用忙等待保证精度
    while time.perf_counter() < end_time:
        pass

def execute_impedance_trajectory(robot, waypoints, durations, K, B, control_rate=200):
    """执行基于轨迹的阻抗控制"""
    if len(waypoints) != len(durations) + 1:
        print("路径点数量应该比时间段数量多1")
        return False
    
    dt = 1.0 / control_rate
    zero_kp = [0.0] * robot.motor_count
    zero_kd = [0.0] * robot.motor_count
    zero_pos = [0.0] * robot.motor_count
    zero_vel = [0.0] * robot.motor_count
    
    for segment in range(len(durations)):
        start_pos = waypoints[segment]
        end_pos = waypoints[segment + 1]
        duration = durations[segment]
        
        steps = int(duration * control_rate)
        segment_start = time.perf_counter()
        
        for step in range(steps):
            target_time = segment_start + (step + 1) * dt
            current_time = step * dt
            
            # 生成期望轨迹
            pos_des, vel_des, _ = robot.septic_interpolation(start_pos, end_pos, duration, current_time)
            
            # 获取当前状态
            states = robot.get_current_state()
            q_current = np.array([state.position for state in states])
            vel_current = np.array([state.velocity for state in states])
            
            # 阻抗控制
            tor_impedance = K * (np.array(pos_des) - q_current) + B * (np.array(vel_des) - vel_current)
            
            # 重力补偿
            G = np.array(robot.get_Gravity(q_current))
            
            # 总力矩
            tor = tor_impedance + G
            
            # 力矩限幅
            tau_limit = np.array([21.0, 36.0, 36.0, 21.0, 10.0, 10.0])
            tor = np.clip(tor, -tau_limit, tau_limit)
            print(tor)
            
            # 发送控制命令
            robot.pos_vel_tqe_kp_kd(zero_pos, zero_vel, tor, zero_kp, zero_kd)

            # 高精度等待
            wait_time = target_time - time.perf_counter()
            if wait_time > 0:
                precise_sleep(wait_time)
    
    return True

def main():
    # 定义轨迹路径点
    waypoints1 = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.5, 0.9, 0.9, -0.10, 0.0, 0.0]
    ]
    
    waypoints2 = [
        [0.5, 0.9, 0.9, -0.10, 0.0, 0.0],
        [0.0, 1.4, 1.1, -0.10, 0.0, 0.0] 
    ]

    waypoints3 = [
        [0.0, 1.4, 1.1, -0.10, 0.0, 0.0], 
        [0.0, 0.9, 0.9, -0.10, 0.0, 0.0]
    ]

    # 定义每段的运动时间
    durations1 = [3.0]

    execute_impedance_trajectory(robot, waypoints1, durations1, K, B, control_rate=200)
    execute_impedance_trajectory(robot, waypoints2, durations1, K, B, control_rate=200)
    execute_impedance_trajectory(robot, waypoints3, durations1, K, B, control_rate=200)

if __name__ == "__main__":
    try:
        robot = Panthera()
        # 阻抗控制参数
        K = np.array([5.0, 10.0, 15.0, 6.0, 5.0, 5.0])
        B = np.array([0.5, 1.0, 1.50, 0.6, 0.5, 0.5])
        
        main()
        
        # 定点阻抗控制
        print("开始定点阻抗控制...")
        final_pos = np.array([0.0, 0.9, 0.9, -0.10, 0.0, 0.0])
        zero_kp = [0.0] * robot.motor_count
        zero_kd = [0.0] * robot.motor_count
        zero_pos = [0.0] * robot.motor_count
        zero_vel = [0.0] * robot.motor_count
        
        while(1):
            # 获取当前状态
            states = robot.get_current_state()
            q_current = np.array([state.position for state in states])
            vel_current = np.array([state.velocity for state in states])
            
            # 定点阻抗控制
            tor_impedance = K * (final_pos - q_current) + B * (np.zeros(6) - vel_current)
            
            # 重力补偿
            G = np.array(robot.get_Gravity(q_current))
            
            # 总力矩
            tor = tor_impedance + G
            
            # 力矩限幅
            tau_limit = np.array([21.0, 36.0, 36.0, 21.0, 10.0, 10.0])
            tor = np.clip(tor, -tau_limit, tau_limit)

            # 发送控制命令
            robot.pos_vel_tqe_kp_kd(zero_pos, zero_vel, tor, zero_kp, zero_kd)
            print(tor)

            time.sleep(0.002)  # 500Hz控制频率
            
    except KeyboardInterrupt:
        print("\n程序中断")
    except Exception as e:
        print(f"\n错误: {e}")