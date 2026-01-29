#include "panthera/Panthera.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <iomanip>
#include <algorithm>
#include <csignal>
#include <atomic>

using namespace panthera;
using namespace std::chrono_literals;

// 全局原子变量，用于信号处理
std::atomic<bool> exitFlag(false);

// 信号处理函数
void signalHandler(int signum)
{
    exitFlag.store(true);
}

/**
 * @brief 关节阻抗控制（PD刚度阻尼项力矩+重力力矩前馈）
 * 其实相当于六个电机PD控制加上前馈力矩
 * （可以对比1_PD_control的效果）
 */

int main()
{
    // 注册信号处理器
    std::signal(SIGINT, signalHandler);

    Panthera robot;

    // 刚度系数和阻尼系数
    std::vector<double> K = {4.0, 10.0, 10.0, 2.0, 2.0, 1.0};
    std::vector<double> B = {0.5, 0.8, 0.8, 0.2, 0.2, 0.1};

    // 都为零则为重力补偿模式
    // std::vector<double> K = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // std::vector<double> B = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    std::vector<double> q_des = {0.0, 0.7, 0.7, -0.1, 0.0, 0.0};  // 期望目标位置
    // std::vector<double> q_des(robot.getMotorCount(), 0.0);  // 期望目标位置
    std::vector<double> v_des(robot.getMotorCount(), 0.0);  // 期望目标速度为0

    // 创建零位置和零速度数组
    std::vector<double> zero_kp(robot.getMotorCount(), 0.0);
    std::vector<double> zero_kd(robot.getMotorCount(), 0.0);
    std::vector<double> zero_pos(robot.getMotorCount(), 0.0);
    std::vector<double> zero_vel(robot.getMotorCount(), 0.0);

    try {
        while (!exitFlag.load()) {
            // 计算阻抗控制输出力矩
            std::vector<double> q_current = robot.getCurrentPos();
            std::vector<double> vel_current = robot.getCurrentVel();

            std::vector<double> tor_impedance(robot.getMotorCount());
            for (size_t i = 0; i < robot.getMotorCount(); ++i) {
                tor_impedance[i] = K[i] * (q_des[i] - q_current[i]) + B[i] * (v_des[i] - vel_current[i]);
            }

            // 力矩计算（加上重力补偿前馈力矩）
            std::vector<double> G = robot.getGravity();
            std::vector<double> tor(robot.getMotorCount());
            for (size_t i = 0; i < robot.getMotorCount(); ++i) {
                tor[i] = tor_impedance[i] + G[i];
            }

            // 力矩限幅（基于电机规格）
            std::vector<double> tau_limit = {10.0, 20.0, 20.0, 10.0, 5.0, 5.0};
            for (size_t i = 0; i < robot.getMotorCount(); ++i) {
                tor[i] = std::max(-tau_limit[i], std::min(tau_limit[i], tor[i]));
            }

            robot.posVelTorqueKpKd(zero_pos, zero_vel, tor, zero_kp, zero_kd);

            std::cout << "阻抗力矩：[";
            for (size_t i = 0; i < tor_impedance.size(); ++i) {
                std::cout << std::fixed << std::setprecision(3) << tor_impedance[i];
                if (i < tor_impedance.size() - 1) std::cout << ", ";
            }
            std::cout << "], \n重力补偿力矩：[";
            for (size_t i = 0; i < G.size(); ++i) {
                std::cout << std::fixed << std::setprecision(3) << G[i];
                if (i < G.size() - 1) std::cout << ", ";
            }
            std::cout << "], \n总力矩：[";
            for (size_t i = 0; i < tor.size(); ++i) {
                std::cout << std::fixed << std::setprecision(3) << tor[i];
                if (i < tor.size() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        // 结束后电机会自动掉电，请注意安全！！

    } catch (const std::exception& e) {
        std::cout << "\n错误: " << e.what() << std::endl;
        std::cout << "\n\n所有电机已停止" << std::endl;
        return 1;
    }

    std::cout << "\n\n所有电机已停止" << std::endl;
    return 0;
}
