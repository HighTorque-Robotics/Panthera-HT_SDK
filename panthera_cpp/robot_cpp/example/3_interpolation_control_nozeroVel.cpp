/**
 * @file 3_interpolation_control_nozeroVel.cpp
 * @brief 七次多项式插值轨迹控制程序（中间点速度不为零）
 *
 * **核心特性**：
 * - 使用七次多项式插值（与Python SDK一致）
 * - 中间点速度非零，实现平滑连续运动（不停顿）
 * - 严格的位置验证和安全性检查
 * - 边界条件：位置、速度连续，加速度和加加速度在边界为0
 */

#include "panthera/Panthera.hpp"
#include <csignal>
#include <atomic>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <algorithm>

// 全局原子变量，用于信号处理
std::atomic<bool> exitFlag(false);

// 信号处理函数
void signalHandler(int signum)
{
    exitFlag.store(true);
}

/**
 * @brief 验证位置在安全范围内
 */
bool isPositionSafe(const std::vector<double>& pos,
                   const std::vector<double>& lower_limits,
                   const std::vector<double>& upper_limits)
{
    for (size_t i = 0; i < pos.size() && i < lower_limits.size(); ++i) {
        if (pos[i] < lower_limits[i] || pos[i] > upper_limits[i]) {
            std::cerr << "  关节" << (i+1) << ": " << pos[i]
                      << " 超出范围 [" << lower_limits[i] << ", " << upper_limits[i] << "]" << std::endl;
            return false;
        }
    }
    return true;
}

/**
 * @brief 安全地执行连续轨迹
 */
bool executeContinuousTrajectorySafe(
    panthera::Panthera& robot,
    const std::vector<std::vector<double>>& waypoints,
    const std::vector<std::vector<double>>& velocities,
    const std::vector<double>& durations,
    int control_rate = 50)
{
    int motor_count = robot.getMotorCount();
    std::vector<double> max_torque(motor_count, 5.0);

    // 关节限位（保守：留出余量，但允许零位）
    std::vector<double> lower_limits = {-2.2, 0.0, 0.0, -1.5, -1.5, -2.3};
    std::vector<double> upper_limits = {2.2, 3.0, 3.8, 1.5, 1.5, 2.3};

    std::cout << "\n=== 开始执行连续轨迹（中间点不停顿） ===" << std::endl;
    std::cout << "路径点数量: " << waypoints.size() << std::endl;
    std::cout << "中间点速度不为零，实现平滑连续运动（无停顿）" << std::endl;

    for (size_t segment = 0; segment < durations.size(); ++segment) {
        const auto& start_pos = waypoints[segment];
        const auto& end_pos = waypoints[segment + 1];
        const auto& start_vel = velocities[segment];
        const auto& end_vel = velocities[segment + 1];
        double duration = durations[segment];

        std::cout << "\n--- 段 " << (segment+1) << " ---" << std::endl;
        std::cout << "起点: [";
        for (size_t i = 0; i < start_pos.size(); ++i) {
            std::cout << start_pos[i];
            if (i < start_pos.size()-1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        std::cout << "终点: [";
        for (size_t i = 0; i < end_pos.size(); ++i) {
            std::cout << end_pos[i];
            if (i < end_pos.size()-1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        std::cout << "持续时间: " << duration << " 秒" << std::endl;

        // 验证起点和终点
        if (!isPositionSafe(start_pos, lower_limits, upper_limits)) {
            std::cerr << "错误：起点超出限位！" << std::endl;
            return false;
        }
        if (!isPositionSafe(end_pos, lower_limits, upper_limits)) {
            std::cerr << "错误：终点超出限位！" << std::endl;
            return false;
        }

        int steps = static_cast<int>(duration * control_rate);
        double dt = 1.0 / control_rate;

        std::cout << "步数: " << steps << std::endl;
        std::cout << "开始运动..." << std::endl;

        auto segment_start = std::chrono::steady_clock::now();

        for (int step = 0; step < steps && !exitFlag.load(); ++step) {
            double current_time = step * dt;

            // 生成插值轨迹（使用七次多项式，实现平滑连续运动）
            std::vector<double> traj_pos, traj_vel, traj_acc;
            panthera::Panthera::septicInterpolationWithVelocity(
                start_pos, end_pos, start_vel, end_vel, duration, current_time,
                traj_pos, traj_vel, traj_acc);

            // 验证插值点
            if (!isPositionSafe(traj_pos, lower_limits, upper_limits)) {
                std::cerr << "错误：插值点超出限位！时间=" << current_time << "s" << std::endl;
                std::cerr << "位置: [";
                for (size_t i = 0; i < traj_pos.size(); ++i) {
                    std::cerr << traj_pos[i];
                    if (i < traj_pos.size()-1) std::cerr << ", ";
                }
                std::cerr << "]" << std::endl;
                return false;
            }

            // 每10步打印一次
            if (step % 10 == 0) {
                std::cout << "  步 " << step << "/" << steps
                          << " 时间=" << std::fixed << std::setprecision(2) << current_time << "s"
                          << " 位置=[" << std::setprecision(3);
                for (size_t i = 0; i < std::min(size_t(3), traj_pos.size()); ++i) {
                    std::cout << traj_pos[i];
                    if (i < 2) std::cout << ", ";
                }
                std::cout << "...]" << std::endl;
            }

            // 发送控制命令
            robot.posVelMaxTorque(traj_pos, traj_vel, max_torque);

            // 高精度等待
            auto target_time = segment_start + std::chrono::duration<double>((step + 1) * dt);
            auto now = std::chrono::steady_clock::now();
            if (now < target_time) {
                std::this_thread::sleep_until(target_time);
            }
        }

        std::cout << "段 " << (segment+1) << " 完成" << std::endl;

        if (exitFlag.load()) break;
    }

    std::cout << "\n=== 轨迹执行完成 ===" << std::endl;
    return true;
}

int main(int argc, char** argv)
{
    // 注册信号处理器
    std::signal(SIGINT, signalHandler);

    try {

        std::string config_path = "../robot_param/Follower.yaml";
        if (argc > 1) {
            config_path = argv[1];
        }

        panthera::Panthera robot(config_path);

        int motor_count = robot.getMotorCount();

        std::cout << "\n=== 安全多项式插值轨迹控制（中间点不停止）===" << std::endl;
        std::cout << std::string(60, '=') << std::endl;

        // 先回零并等待稳定
        std::cout << "\n返回零位并等待稳定..." << std::endl;
        std::vector<double> zero_pos(motor_count, 0.0);
        std::vector<double> vel(motor_count, 0.3);
        std::vector<double> max_torque(motor_count, 5.0);

        robot.posVelMaxTorque(zero_pos, vel, max_torque);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // **定义路径点（与Python SDK类似配置）**
        std::vector<std::vector<double>> waypoints = {
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},           // 起点（零位）
            {-0.26, 0.6, 0.60, 0.4, -0.3, -0.2},      // 中间点1
            {0.2, 1.0, 1.2, 0.6, -0.5, 0.2}           // 终点
        };

        // **定义速度（起点和终点速度为0，中间点速度不为0）**
        // 这是实现连续运动的关键！中间点速度非零，机器人不会停止
        std::vector<std::vector<double>> velocities = {
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},           // 起点速度为0
            {0.2, 0.6, 0.6, 0.3, 0.4, 0.2},           // 中间点速度不为0（连续运动）
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}            // 终点速度为0
        };

        // 每段运动时间（稍长一点以确保安全）
        std::vector<double> durations = {2.0, 2.0};

        std::cout << "\n轨迹定义：" << std::endl;
        std::cout << "  - 路径点数量: " << waypoints.size() << std::endl;
        std::cout << "  - 中间点速度不为零，实现平滑连续运动（不停顿）" << std::endl;
        std::cout << "  - 使用七次多项式插值（与Python SDK一致）" << std::endl;
        std::cout << "  - 运动时间: 2.0秒/段" << std::endl;
        std::cout << "  - 严格位置验证" << std::endl;
        std::cout << "\n按 Ctrl+C 立即停止..." << std::endl;
        std::cout << std::string(60, '=') << "\n" << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));

        bool success = executeContinuousTrajectorySafe(robot, waypoints, velocities, durations, 50);

        if (success && !exitFlag.load()) {
            std::cout << "\n轨迹执行成功，保持位置1秒..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));

            std::cout << "返回零位..." << std::endl;
            robot.posVelMaxTorque(zero_pos, vel, max_torque);
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "\n程序结束" << std::endl;
    return 0;
}
