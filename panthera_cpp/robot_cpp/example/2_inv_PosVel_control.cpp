/**
 * @file 2_inv_PosVel_control.cpp
 * @brief 简单的逆解运算程序
 *
 * 设定末端位姿后解算并发送关节执行
 * 考虑工具偏移，使用完整的6D IK（位置+姿态）
 */

#include "panthera/Panthera.hpp"
#include <csignal>
#include <atomic>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <thread>
#include <Eigen/Dense>

// 全局原子变量，用于信号处理函数
std::atomic<bool> exitFlag(false);

// 信号处理函数
void signalHandler(int signum)
{
    exitFlag.store(true);
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

        if (!robot.isModelLoaded()) {
            std::cerr << "错误: URDF 模型未加载，无法进行逆运动学求解" << std::endl;
            return 1;
        }

        std::cout << "URDF 模型加载成功，逆运动学功能已启用" << std::endl;

        // 控制参数
        std::vector<double> vel(motor_count, 0.5);
        std::vector<double> max_torque(motor_count, 10.0);

        std::cout << "\n逆运动学控制程序（考虑工具偏移）" << std::endl;
        std::cout << "工具偏移: 0.14m" << std::endl;
        std::cout << std::string(60, '=') << std::endl;

        // 定义目标末端位置
        Eigen::Vector3d ik_pos1(0.3 + 0.138, 0.0, 0.3);
        Eigen::Vector3d ik_pos2(0.5 + 0.138, 0.0, 0.3);
        Eigen::Vector3d ik_pos3(0.74, 0.0, 0.2);

        // 获取初始关节位置
        std::vector<double> current_q = robot.getCurrentPos();

        // ====== 执行轨迹1 ======
        std::cout << "\n执行轨迹1..." << std::endl;
        std::cout << "目标末端位置: (" << ik_pos1.transpose() << ")" << std::endl;

        std::vector<double> pos1 = robot.inverseKinematics(ik_pos1, nullptr, current_q, 1000, 1e-4);

        if (!pos1.empty()) {
            robot.posVelMaxTorque(pos1, vel, max_torque);
            std::cout << "执行状态1: 成功" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(3));
            current_q = pos1;
        } else {
            std::cout << "执行状态1: IK未收敛" << std::endl;
        }

        if (exitFlag.load()) return 0;

        // ====== 执行轨迹2 ======
        std::cout << "\n执行轨迹2..." << std::endl;
        std::cout << "目标末端位置: (" << ik_pos2.transpose() << ")" << std::endl;

        std::vector<double> pos2 = robot.inverseKinematics(ik_pos2, nullptr, current_q, 1000, 1e-4);

        if (!pos2.empty()) {
            robot.posVelMaxTorque(pos2, vel, max_torque);
            std::cout << "执行状态2: 成功" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(3));
            current_q = pos2;
        } else {
            std::cout << "执行状态2: IK未收敛" << std::endl;
        }

        if (exitFlag.load()) return 0;

        // ====== 执行轨迹3（超限示例）======
        std::cout << "\n执行轨迹3（超限示例）..." << std::endl;
        std::cout << "目标末端位置: (" << ik_pos3.transpose() << ")" << std::endl;

        std::vector<double> pos3 = robot.inverseKinematics(ik_pos3, nullptr, current_q, 1000, 1e-4);

        if (!pos3.empty()) {
            robot.posVelMaxTorque(pos3, vel, max_torque);
            std::cout << "执行状态3: 成功" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(3));
        } else {
            std::cout << "执行状态3: IK未收敛（位置可能超出工作空间）" << std::endl;
        }

        if (exitFlag.load()) return 0;

        // ====== 返回零位 ======
        std::cout << "\n返回零位..." << std::endl;
        std::vector<double> zero_pos(motor_count, 0.0);
        robot.posVelMaxTorque(zero_pos, vel, max_torque);
        std::cout << "保持位置2秒..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "\n\n程序结束，所有电机已停止" << std::endl;
    return 0;
}
