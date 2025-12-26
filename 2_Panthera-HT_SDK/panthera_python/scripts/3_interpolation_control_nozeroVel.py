#!/usr/bin/env python3
"""
七次多项式插值控制（指定中间点速度）
实现连续运动，中间插值点速度不为零
"""

import time
import sys
import os
# 添加上一级目录(python目录)到sys.path
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_dir)
from scripts.Panthera_lib.Panthera import Panthera

def precise_sleep(duration):
    """高精度延时函数"""
    if duration <= 0:
        return
    
    end_time = time.perf_counter() + duration
    
    if duration > 0.001:
        time.sleep(duration - 0.001)
    
    while time.perf_counter() < end_time:
        pass

def execute_continuous_trajectory(robot, waypoints, velocities, durations, control_rate=200):
    """
    执行连续轨迹（速度不中断）
    waypoints: 路径点列表
    velocities: 每个路径点的速度（起始和结束速度为0）
    durations: 每段的时间
    """
    if len(waypoints) != len(velocities):
        print("路径点和速度数量必须相同")
        return False
    
    if len(waypoints) != len(durations) + 1:
        print("路径点数量应该比时间段数量多1")
        return False
    
    dt = 1.0 / control_rate
    
    for segment in range(len(durations)):
        start_pos = waypoints[segment]
        end_pos = waypoints[segment + 1]
        start_vel = velocities[segment]
        end_vel = velocities[segment + 1]
        duration = durations[segment]
        
        steps = int(duration * control_rate)
        
        # 记录段开始时间
        segment_start = time.perf_counter()
        
        for step in range(steps):
            # 计算本步应该执行的绝对时间
            target_time = segment_start + (step + 1) * dt
            current_time = step * dt
            
            # 使用带速度边界条件的七次多项式
            pos, vel, _ = robot.septic_interpolation_with_velocity(
                start_pos, end_pos, start_vel, end_vel, duration, current_time
            )
            
            # 发送控制命令
            robot.pos_vel_MAXtqe(pos, vel, [10.0]*robot.motor_count)
            
            # 高精度等待
            wait_time = target_time - time.perf_counter()
            if wait_time > 0:
                precise_sleep(wait_time)
    
    # 最终位置锁定
    final_pos = waypoints[-1]
    robot.pos_vel_MAXtqe(final_pos, [0.0]*robot.motor_count, [10.0]*robot.motor_count)
    
    return True

def main():
    robot = Panthera()
    
    # 先回零
    zero_pos = [0.0] * robot.motor_count
    robot.pos_vel_MAXtqe(zero_pos, [0.5]*6, [10.0]*6, iswait=True)
    time.sleep(1)
    
    # 定义三个关键路径点
    waypoints = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],          # 起点（零位）
        [-0.26, 0.6, 0.60, 0.4, -0.3, -0.2],     # 中间点1
        [0.2, 1.0, 1.2, 0.6, -0.5, 0.2]          # 终点
    ]
    
    # 定义每个点的速度（起点和终点速度为0，中间点速度不为0）
    velocities = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],          # 起点速度为0
        [0.2, 0.6, 0.6, 0.3, 0.4, 0.2],          # 中间点速度不为0（连续运动）
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]           # 终点速度为0
    ]
    
    # 每段运动时间
    durations = [1.0, 1.0]
    
    print("执行连续轨迹（中间点不停止）...")
    execute_continuous_trajectory(robot, waypoints, velocities, durations, control_rate=100)
    
    time.sleep(2)
    
    robot.pos_vel_MAXtqe(zero_pos, [0.5]*6, [10.0]*6, iswait=True)
    time.sleep(2)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # robot.set_stop()
        print("\n程序中断")
    except Exception as e:
        print(f"\n错误: {e}")