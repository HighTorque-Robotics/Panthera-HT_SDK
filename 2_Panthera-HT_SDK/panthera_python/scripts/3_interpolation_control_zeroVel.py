#!/usr/bin/env python3
"""
三次多项式插值轨迹控制程序
通过修改waypoints和durations来设置轨迹路径点
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
    
    # 大部分时间用sleep（留1ms余量）
    if duration > 0.001:
        time.sleep(duration - 0.001)
    
    # 最后用忙等待保证精度
    while time.perf_counter() < end_time:
        pass

def execute_trajectory(robot, waypoints, durations, control_rate=100):
    """执行轨迹跟踪（使用高精度定时）"""
    if len(waypoints) != len(durations) + 1:
        print("路径点数量应该比时间段数量多1")
        return False
    
    dt = 1.0 / control_rate
    
    for segment in range(len(durations)):
        start_pos = waypoints[segment]
        end_pos = waypoints[segment + 1]
        duration = durations[segment]
        
        steps = int(duration * control_rate)
        
        # 记录段开始时间，使用绝对时间避免累积误差
        segment_start = time.perf_counter()
        
        for step in range(steps):
            # 计算本步应该执行的绝对时间
            target_time = segment_start + (step + 1) * dt
            current_time = step * dt
            
            # 生成插值轨迹
            # 七次多项式（加加速度连续，最平滑）
            pos, vel, _ = robot.septic_interpolation(start_pos, end_pos, duration, current_time)

            # 发送控制命令
            robot.pos_vel_MAXtqe(pos, vel, [10.0]*robot.motor_count)
            
            # 高精度等待到下一个控制周期
            wait_time = target_time - time.perf_counter()
            if wait_time > 0:
                precise_sleep(wait_time)
    
    # 到达最终位置
    final_pos = waypoints[-1]
    robot.pos_vel_MAXtqe(final_pos, [0.0]*robot.motor_count, [10.0]*robot.motor_count)
    
    return True

def main():
    robot = Panthera()
    
    # 先回零
    zero_pos = [0.0] * robot.motor_count
    robot.pos_vel_MAXtqe(zero_pos, [0.5]*6, [10.0]*6, iswait=True)
    time.sleep(1)
    
    # 定义轨迹路径点（可以添加更多点位）
    waypoints = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],      # 起点
        [-0.6, 0.7, 0.90, 0.2, -0.3, -0.2],     
        [-0.4, 1.4, 1.8, 0.5, -0.7, 0.2],   
        [-0.2, 0.8, 1.2, 0.7, 0.0, 0.4],
        [-0.4, 1.4, 1.8, 0.5, -0.7, 0.2],
        [-0.6, 0.7, 0.90, 0.2, -0.3, -0.2]# 终点
    ]
    
    # 定义每段的运动时间（秒）
    durations = [1.2, 1.0, 1.0, 1.0, 1.2]
    
    execute_trajectory(robot, waypoints, durations, control_rate=100)
    time.sleep(1)
    robot.pos_vel_MAXtqe(zero_pos, [0.5]*6, [10.0]*6, iswait=True)
    time.sleep(2)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # robot.set_stop()
        print("\n程序中断")