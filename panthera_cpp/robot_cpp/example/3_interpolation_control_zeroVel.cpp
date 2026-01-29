/**
 * @file 3_interpolation_control_zeroVel.cpp
 * @brief 多项式插值轨迹控制程序（中间点速度为零）
 * 对应 Python 版本: 3_interpolation_control_zeroVel.py
 */

#include "panthera/Panthera.hpp"
#include <csignal>
#include <atomic>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

using namespace std;

// 全局原子变量，用于信号处理函数
std::atomic<bool> exitFlag(false);

// 信号处理函数
void signalHandler(int signum)
{
    exitFlag.store(true);
}

// 高精度延时函数（对应Python: precise_sleep）
void precise_sleep(double duration)
{
    if (duration <= 0) return;
    auto end_time = chrono::steady_clock::now() + chrono::duration<double>(duration);
    if (duration > 0.001) {
        this_thread::sleep_for(chrono::duration<double>(duration - 0.001));
    }
    while (chrono::steady_clock::now() < end_time) {
        // 忙等待
    }
}

// 执行轨迹跟踪函数（对应Python: execute_trajectory）
bool execute_trajectory(panthera::Panthera& robot,
                        const vector<vector<double>>& waypoints,
                        const vector<double>& durations,
                        const std::atomic<bool>& exitFlag,
                        int control_rate = 100)
{
    if (waypoints.size() != durations.size() + 1) {
        cout << "路径点数量应该比时间段数量多1" << endl;
        return false;
    }

    int motor_count = robot.getMotorCount();
    double dt = 1.0 / control_rate;

    for (size_t segment = 0; segment < durations.size(); ++segment) {
        const auto& start_pos = waypoints[segment];
        const auto& end_pos = waypoints[segment + 1];
        double duration = durations[segment];
        int steps = static_cast<int>(duration * control_rate);

        auto segment_start = chrono::steady_clock::now();

        for (int step = 0; step < steps && !exitFlag.load(); ++step) {
            auto target_time = segment_start + chrono::duration<double>((step + 1) * dt);
            double current_time = step * dt;

            // 调用可复用函数（对应Python: robot.septic_interpolation()）
            vector<double> pos, vel, acc;
            panthera::Panthera::septicInterpolation(start_pos, end_pos, duration, current_time, pos, vel, acc);

            // 发送控制命令（对应Python: robot.pos_vel_MAXtqe()）
            vector<double> max_torque(motor_count, 10.0);
            robot.posVelMaxTorque(pos, vel, max_torque);

            auto now = chrono::steady_clock::now();
            if (target_time > now) {
                precise_sleep(chrono::duration<double>(target_time - now).count());
            }
        }

        if (!exitFlag.load()) {
            cout << "段 " << (segment + 1) << " 完成" << endl;
        } else {
            break;
        }
    }

    if (!exitFlag.load()) {
        const auto& final_pos = waypoints.back();
        vector<double> zero_vel(motor_count, 0.0);
        vector<double> max_torque(motor_count, 10.0);
        robot.posVelMaxTorque(final_pos, zero_vel, max_torque);
    }

    return true;
}

int main()
{
    // 注册信号处理器
    std::signal(SIGINT, signalHandler);

    try {
        panthera::Panthera robot;

        int motor_count = robot.getMotorCount();

        // 先回零（对应Python: robot.pos_vel_MAXtqe(zero_pos, [0.5]*6, [10.0]*6, iswait=True)）
        cout << "返回零位..." << endl;
        vector<double> zero_pos(motor_count, 0.0);
        vector<double> vel(motor_count, 0.5);
        vector<double> max_torque(motor_count, 10.0);
        robot.posVelMaxTorque(zero_pos, vel, max_torque, true);
        this_thread::sleep_for(chrono::seconds(1));

        // 定义轨迹路径点（对应Python: waypoints）
        vector<vector<double>> waypoints = {
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            {-0.6, 0.7, 0.90, 0.2, -0.3, -0.2},
            {-0.4, 1.4, 1.8, 0.5, -0.7, 0.2},
            {-0.2, 0.8, 1.2, 0.7, 0.0, 0.4},
            {-0.4, 1.4, 1.8, 0.5, -0.7, 0.2},
            {-0.6, 0.7, 0.90, 0.2, -0.3, -0.2}
        };

        // 定义每段的运动时间（对应Python: durations）
        vector<double> durations = {1.2, 1.0, 1.0, 1.0, 1.2};

        // 调用可复用函数执行轨迹（对应Python: execute_trajectory(robot, waypoints, durations, control_rate=100)）
        execute_trajectory(robot, waypoints, durations, exitFlag, 100);

        if (!exitFlag.load()) {
            this_thread::sleep_for(chrono::seconds(1));
            robot.posVelMaxTorque(zero_pos, vel, max_torque, true);
            this_thread::sleep_for(chrono::seconds(2));
        }

    } catch (const exception& e) {
        cerr << "错误: " << e.what() << endl;
        return 1;
    }

    cout << "\n程序结束" << endl;
    return 0;
}
