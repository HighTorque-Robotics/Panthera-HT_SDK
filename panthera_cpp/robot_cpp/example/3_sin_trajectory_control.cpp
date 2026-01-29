/**
 * @file 3_sin_trajectory_control.cpp
 * @brief 正弦轨迹跟踪控制程序
 *
 * 机器人关节沿着正弦函数轨迹运动
 * 自动检测关节限位并调整运动范围
 */

#include "panthera/Panthera.hpp"
#include <csignal>
#include <atomic>
#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <cmath>
#include <thread>
#include <chrono>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 全局原子变量，用于信号处理
std::atomic<bool> exitFlag(false);

// 信号处理函数
void signalHandler(int signum)
{
    exitFlag.store(true);
}

/**
 * @brief 向量限幅函数（逐元素）
 */
std::vector<double> clipVector(const std::vector<double>& vec,
                                const std::vector<double>& min_val,
                                const std::vector<double>& max_val)
{
    std::vector<double> result(vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
        result[i] = std::max(min_val[i], std::min(max_val[i], vec[i]));
    }
    return result;
}

/**
 * @brief 向量限幅函数（标量上下界）
 */
std::vector<double> clipVectorScalar(const std::vector<double>& vec, double min_val, double max_val)
{
    std::vector<double> result(vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
        result[i] = std::max(min_val, std::min(max_val, vec[i]));
    }
    return result;
}

/**
 * @brief 向量元素最小值
 */
std::vector<double> minVector(const std::vector<double>& a, const std::vector<double>& b)
{
    std::vector<double> result(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        result[i] = std::min(a[i], b[i]);
    }
    return result;
}

/**
 * @brief 向量加法（标量）
 */
std::vector<double> addScalar(const std::vector<double>& vec, double scalar)
{
    std::vector<double> result(vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
        result[i] = vec[i] + scalar;
    }
    return result;
}

/**
 * @brief 向量乘法（标量）
 */
std::vector<double> mulScalar(const std::vector<double>& vec, double scalar)
{
    std::vector<double> result(vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
        result[i] = vec[i] * scalar;
    }
    return result;
}

/**
 * @brief 计算正弦值向量
 */
std::vector<double> sinVector(const std::vector<double>& vec)
{
    std::vector<double> result(vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
        result[i] = std::sin(vec[i]);
    }
    return result;
}

/**
 * @brief 计算余弦值向量
 */
std::vector<double> cosVector(const std::vector<double>& vec)
{
    std::vector<double> result(vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
        result[i] = std::cos(vec[i]);
    }
    return result;
}

int main(int argc, char** argv)
{
    // 注册信号处理器
    std::signal(SIGINT, signalHandler);

    try {
        // 创建机械臂对象
        std::string config_path = "../robot_param/Follower.yaml";
        if (argc > 1) {
            config_path = argv[1];
        }

        panthera::Panthera robot(config_path);

        int motor_count = robot.getMotorCount();

        // 控制参数
        double frequency = 0.45;  // Hz，正弦波频率（可调节：0.1-2.0 Hz）
        double duration = 600.0;  // 运动持续时间（秒）
        int control_rate = 500;   // 控制频率 Hz
        double dt = 1.0 / control_rate;
        std::vector<double> max_torque(motor_count, 10.0);

        // 定义各关节角度限制（弧度）
        // 注意：这些限制值可能需要根据实际情况调整
        std::vector<double> lower_limits = {-M_PI, 0.0, 0.0, 0.0, -M_PI/2, -M_PI};
        std::vector<double> upper_limits = {M_PI, M_PI, M_PI, M_PI/2, M_PI/2, M_PI};

        // 先移动到安全的初始位置（与Python版本一致）
        std::cout << "移动到初始位置..." << std::endl;
        std::vector<double> zero_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<double> init_pos = {-0.3, 1.1, 1.1, 0.2, -0.3, 0.0};  // 与Python版本完全一致
        std::vector<double> vel(motor_count, 0.5);

        robot.posVelMaxTorque(zero_pos, vel, max_torque, true);
        std::this_thread::sleep_for(std::chrono::seconds(3));

        robot.posVelMaxTorque(init_pos, vel, max_torque, true);
        std::cout << "到达初始位置" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // 获取初始位置作为中心位置
        std::cout << "获取初始位置..." << std::endl;
        std::vector<double> center_pos = robot.getCurrentPos();
        std::cout << "中心位置: [";
        for (size_t i = 0; i < center_pos.size(); ++i) {
            std::cout << center_pos[i];
            if (i < center_pos.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        // 检查初始位置是否在限制范围内
        for (int i = 0; i < motor_count; ++i) {
            if (center_pos[i] < lower_limits[i] || center_pos[i] > upper_limits[i]) {
                std::cout << "警告: 关节" << (i+1) << "初始位置 " << center_pos[i]
                          << " 超出限制范围 [" << lower_limits[i] << ", " << upper_limits[i] << "]" << std::endl;
            }
        }

        // 计算安全振幅
        std::vector<double> dist_to_upper(motor_count);
        std::vector<double> dist_to_lower(motor_count);
        for (int i = 0; i < motor_count; ++i) {
            dist_to_upper[i] = upper_limits[i] - center_pos[i];
            dist_to_lower[i] = center_pos[i] - lower_limits[i];
        }
        std::vector<double> safe_amplitudes = minVector(
            mulScalar(dist_to_upper, 0.8),
            mulScalar(dist_to_lower, 0.8)
        );
        std::vector<double> preset_amplitudes = {0.4, 0.6, 0.6, 0.5, 0.4, 0.0};
        std::vector<double> amplitudes = minVector(safe_amplitudes, preset_amplitudes);

        std::cout << "调整后的振幅: [";
        for (size_t i = 0; i < amplitudes.size(); ++i) {
            std::cout << amplitudes[i];
            if (i < amplitudes.size() - 1) std::cout << ", ";
        }
        std::cout << "] rad" << std::endl;

        // 计算并显示最大速度
        std::vector<double> max_velocities = mulScalar(amplitudes, 2 * M_PI * frequency);
        std::cout << "各关节最大速度: [";
        for (size_t i = 0; i < max_velocities.size(); ++i) {
            std::cout << max_velocities[i];
            if (i < max_velocities.size() - 1) std::cout << ", ";
        }
        std::cout << "] rad/s" << std::endl;

        // 相位偏移（可以让各关节运动不同步）
        std::vector<double> phase_offsets(motor_count, 0.0);  // 零相位偏移

        std::cout << "\n开始正弦轨迹运动..." << std::endl;
        std::cout << "频率: " << frequency << " Hz, 持续时间: " << duration << " 秒" << std::endl;

        auto start_time = std::chrono::steady_clock::now();
        int step = 0;

        while (!exitFlag.load()) {
            auto loop_start = std::chrono::steady_clock::now();
            auto current_time = std::chrono::duration<double>(loop_start - start_time).count();

            if (current_time >= duration) {
                break;
            }

            // 计算正弦轨迹
            double omega = 2 * M_PI * frequency;

            // 位置：x = x0 + A * sin(ωt + φ)
            std::vector<double> phase_with_time = addScalar(phase_offsets, omega * current_time);
            std::vector<double> sin_values = sinVector(phase_with_time);
            std::vector<double> pos = center_pos;
            for (int i = 0; i < motor_count; ++i) {
                pos[i] = center_pos[i] + amplitudes[i] * sin_values[i];
            }

            // 速度（位置的导数）：v = A * ω * cos(ωt + φ)
            std::vector<double> cos_values = cosVector(phase_with_time);
            std::vector<double> vel(motor_count);
            for (int i = 0; i < motor_count; ++i) {
                vel[i] = amplitudes[i] * omega * cos_values[i];
            }

            // 角度限幅
            pos = clipVector(pos, lower_limits, upper_limits);

            // 到达限位时速度置零
            for (int i = 0; i < motor_count; ++i) {
                if (pos[i] <= lower_limits[i] || pos[i] >= upper_limits[i]) {
                    vel[i] = 0.0;
                }
            }

            robot.posVelMaxTorque(pos, vel, max_torque, false);

            // 定期打印状态
            if (step % 50 == 0) {  // 每0.5秒打印一次
                std::cout << std::fixed << std::setprecision(2);
                std::cout << "\r时间: " << current_time << "s | "
                          << "关节1位置: " << pos[0] << " | "
                          << "关节2位置: " << pos[1] << " | "
                          << "关节3位置: " << pos[2] << "    " << std::flush;
            }

            step++;

            // 控制循环频率
            auto loop_end = std::chrono::steady_clock::now();
            double loop_time = std::chrono::duration<double>(loop_end - loop_start).count();
            if (loop_time < dt) {
                std::this_thread::sleep_for(std::chrono::duration<double>(dt - loop_time));
            }
        }

        // 返回中心位置
        std::cout << "\n\n返回中心位置..." << std::endl;
        robot.posVelMaxTorque(center_pos, vel, max_torque, true);

        std::cout << "运动完成" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // 返回零位
        robot.posVelMaxTorque(zero_pos, vel, max_torque, true);
        std::this_thread::sleep_for(std::chrono::seconds(2));

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "\n\n所有电机已停止" << std::endl;
    return 0;
}
