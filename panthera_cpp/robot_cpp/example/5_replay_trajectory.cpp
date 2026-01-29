/**
 * @file 5_replay_trajectory.cpp
 * @brief 回放 JSONL 轨迹文件（单臂+夹爪）
 *
 * 支持位置+速度+夹爪回放
 * 控制方式：前馈力矩（重力补偿+摩擦补偿）+ PD反馈控制（轨迹跟踪）
 * 对应 Python 版本: 5_replay_trajectory.py
 */

#include "panthera/Panthera.hpp"
#include <csignal>
#include <atomic>
#include <iostream>
#include <iomanip>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>
#include <algorithm>

// 全局原子变量，用于信号处理函数
std::atomic<bool> exitFlag(false);

// 信号处理函数
void signalHandler(int signum)
{
    exitFlag.store(true);
}


/**
 * @brief 轨迹点结构
 */
struct TrajectoryPoint {
    double timestamp;
    std::vector<double> positions;
    std::vector<double> velocities;
    double gripper_position;
    double gripper_velocity;
};

/**
 * @brief 从JSON文件读取轨迹
 */
std::vector<TrajectoryPoint> loadTrajectory(const std::string& filepath) {
    std::vector<TrajectoryPoint> trajectory;
    std::ifstream file(filepath);

    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filepath << std::endl;
        return trajectory;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;

        TrajectoryPoint point;
        point.gripper_position = 0.0;  // 初始化为0
        point.gripper_velocity = 0.0;  // 初始化为0

        // 提取时间戳 "t"
        size_t pos = line.find("\"t\":");
        if (pos != std::string::npos) {
            size_t start = pos + 4;  // 跳过 "t:
            size_t end = line.find(",", start);
            if (end == std::string::npos) {
                end = line.find("}", start);
            }
            std::string timestamp_str = line.substr(start, end - start);
            try {
                point.timestamp = std::stod(timestamp_str);
            } catch (...) {
                point.timestamp = 0.0;
            }
        }

        // 提取位置数组 "pos"
        pos = line.find("\"pos\":[");
        if (pos != std::string::npos) {
            size_t start = pos + 7;
            size_t end = line.find("]", start);
            std::string positions_str = line.substr(start, end - start);
            std::stringstream ss(positions_str);
            std::string token;
            while (std::getline(ss, token, ',')) {
                point.positions.push_back(std::stod(token));
            }
        }

        // 提取速度数组 "vel"
        pos = line.find("\"vel\":[");
        if (pos != std::string::npos) {
            size_t start = pos + 7;
            size_t end = line.find("]", start);
            std::string velocities_str = line.substr(start, end - start);
            std::stringstream ss(velocities_str);
            std::string token;
            while (std::getline(ss, token, ',')) {
                point.velocities.push_back(std::stod(token));
            }
        }

        // 提取夹爪位置 "gripper_pos"
        pos = line.find("\"gripper_pos\":");
        if (pos != std::string::npos) {
            size_t start = pos + 15;
            size_t end = line.find(",", start);
            if (end == std::string::npos) {
                end = line.find("}", start);
            }
            point.gripper_position = std::stod(line.substr(start, end - start));
        }

        // 提取夹爪速度 "gripper_vel"
        pos = line.find("\"gripper_vel\":");
        if (pos != std::string::npos) {
            size_t start = pos + 15;
            size_t end = line.find("}", start);
            point.gripper_velocity = std::stod(line.substr(start, end - start));
        }

        trajectory.push_back(point);
    }

    file.close();
    std::cout << "成功加载 " << trajectory.size() << " 个轨迹点" << std::endl;
    return trajectory;
}

int main(int argc, char** argv)
{
    // 注册信号处理器
    std::signal(SIGINT, signalHandler);

    // 默认轨迹文件
    std::string trajectory_file = "trajectory_*.jsonl";

    // 关节PD增益（用于轨迹跟踪）
    std::vector<double> kp_play = {10.0, 21.0, 21.0, 6.0, 5.0, 1.0};
    std::vector<double> kd_play = {1.0, 2.0, 2.0, 0.9, 0.7, 0.1};

    // 摩擦补偿参数
    std::vector<double> Fc = {0.15, 0.12, 0.12, 0.12, 0.04, 0.04};
    std::vector<double> Fv = {0.05, 0.05, 0.05, 0.03, 0.02, 0.02};
    double vel_threshold = 0.02;

    // 力矩限制
    std::vector<double> tau_limit = {15.0, 30.0, 30.0, 15.0, 5.0, 5.0};

    // 夹爪PD增益
    double gripper_kp = 5.0;
    double gripper_kd = 0.5;

    try {

        std::string config_path = "../robot_param/Follower.yaml";  // 固定使用此配置文件，无需命令行指定

        // 解析命令行参数（只接受轨迹文件参数）
        if (argc > 1) {
            trajectory_file = argv[1];
        } else {
            std::cout << "请指定轨迹文件" << std::endl;
            std::cout << "用法: " << argv[0] << " trajectory_file.jsonl" << std::endl;
            std::cout << "示例:" << std::endl;
            std::cout << "  " << argv[0] << " trajectory_20260123_013836.jsonl" << std::endl;
            return 1;
        }

        // 查找最新的轨迹文件（如果使用通配符）
        if (trajectory_file.find('*') != std::string::npos) {
            std::cout << "请指定具体的轨迹文件" << std::endl;
            std::cout << "用法: " << argv[0] << " trajectory_file.jsonl" << std::endl;
            std::cout << "示例:" << std::endl;
            std::cout << "  " << argv[0] << " trajectory_20260123_013836.jsonl" << std::endl;
            return 1;
        }

        panthera::Panthera robot(config_path);

        int motor_count = robot.getMotorCount();

        // 加载轨迹
        std::cout << "\n开始回放: " << trajectory_file << std::endl;
        std::cout << "数据格式: 自动检测（支持位置+速度+夹爪）" << std::endl;

        auto trajectory = loadTrajectory(trajectory_file);
        if (trajectory.empty()) {
            std::cerr << "轨迹为空或加载失败" << std::endl;
            return 1;
        }

        // 检查并修正轨迹数据的关节数量
        for (auto& point : trajectory) {
            // 如果关节数量多于 motor_count，截取前 motor_count 个
            if (point.positions.size() > motor_count) {
                point.positions.resize(motor_count);
            }
            // 如果关节数量少于 motor_count，填充 0
            while (point.positions.size() < motor_count) {
                point.positions.push_back(0.0);
            }

            // 同样处理速度
            if (point.velocities.size() > motor_count) {
                point.velocities.resize(motor_count);
            }
            while (point.velocities.size() < motor_count) {
                point.velocities.push_back(0.0);
            }
        }

        std::cout << "[Player] 共 " << trajectory.size() << " 帧" << std::endl;

        // 打印第一帧和最后一帧的时间戳（调试用）
        if (!trajectory.empty()) {
            std::cout << "[Player] 第一帧时间戳: " << std::fixed << std::setprecision(6) << trajectory[0].timestamp << "s" << std::endl;
            std::cout << "[Player] 最后一帧时间戳: " << std::fixed << std::setprecision(6) << trajectory.back().timestamp << "s" << std::endl;
            std::cout << "[Player] 总时长: " << std::fixed << std::setprecision(6) << trajectory.back().timestamp << "s" << std::endl;
            std::cout << "[Player] 平均帧率: " << std::fixed << std::setprecision(1) << trajectory.size() / trajectory.back().timestamp << " Hz" << std::endl;
        }

        // 移动到起始点
        auto& first_frame = trajectory[0];
        std::cout << "[Player] 正在移动到轨迹起点..." << std::endl;

        // 关节移动到起点（使用缓慢速度）
        std::vector<double> start_pos = first_frame.positions;
        std::vector<double> move_vel(motor_count, 0.5);  // 缓慢速度 0.5 rad/s
        std::vector<double> max_torque(motor_count, 21.0);
        max_torque = {21.0, 36.0, 36.0, 21.0, 10.0, 10.0};

        robot.posVelMaxTorque(start_pos, move_vel, max_torque, true);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::cout << "[Player] 已到达起点，开始回放..." << std::endl;
        std::cout << "\n按 Ctrl+C 停止" << std::endl;
        std::cout << std::string(60, '=') << "\n" << std::endl;

        auto start_time = std::chrono::steady_clock::now();

        for (size_t i = 0; i < trajectory.size() && !exitFlag.load(); ++i) {
            const auto& point = trajectory[i];

            // 计算应该到达的时间
            auto target_time = start_time + std::chrono::duration<double>(point.timestamp);
            auto now = std::chrono::steady_clock::now();

            // 如果还没到时间，等待
            if (now < target_time) {
                std::this_thread::sleep_until(target_time);
            }

            // 调试：打印前10帧的时间戳和等待时间
            if (i < 10) {
                auto wait_time = std::chrono::duration<double>(target_time - now).count();
                std::cout << "[Player] 帧 " << i << " 时间戳=" << std::fixed << std::setprecision(6) << point.timestamp
                         << "s, 等待=" << wait_time << "s" << std::endl;
            }

            // 获取当前速度
            std::vector<double> current_vel = robot.getCurrentVel();

            // 计算重力补偿力矩
            std::vector<double> gravity_torque = robot.getGravity();

            // 计算摩擦补偿力矩
            std::vector<double> friction_torque = robot.getFrictionCompensation(
                current_vel, Fc, Fv, vel_threshold
            );

            // 总力矩 = 重力补偿 + 摩擦补偿（不是PD控制）
            std::vector<double> total_torque(motor_count);
            for (int j = 0; j < motor_count; ++j) {
                total_torque[j] = gravity_torque[j] + friction_torque[j];
            }

            // 力矩限幅
            total_torque = robot.clipTorque(total_torque, tau_limit);

            // 发送控制命令
            // 前馈力矩 = 重力补偿 + 摩擦补偿
            // 反馈力矩 = PD 控制（用于轨迹跟踪）
            robot.posVelTorqueKpKd(point.positions, point.velocities, total_torque, kp_play, kd_play);

            // 夹爪控制（如果有夹爪数据）
            if (point.gripper_position != 0.0 || point.gripper_velocity != 0.0) {
                robot.gripperControlMIT(point.gripper_position, point.gripper_velocity, 0.0, gripper_kp, gripper_kd);
            }

            // 打印进度（每10个点）
            if (i % 10 == 0) {
                std::cout << "\r进度: " << i + 1 << "/" << trajectory.size()
                          << " (" << std::fixed << std::setprecision(1)
                          << (i + 1) * 100.0 / trajectory.size() << "%)" << std::flush;
            }
        }

        std::cout << "\n\n回放完成！" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "\n错误: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "电机已停止" << std::endl;
    return 0;
}
