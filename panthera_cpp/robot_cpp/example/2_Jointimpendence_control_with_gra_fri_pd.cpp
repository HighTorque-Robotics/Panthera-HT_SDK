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
 * @brief 关节阻抗控制+摩擦补偿
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

    // ==================== 摩擦参数配置 ====================
    // 注意：这些参数需要根据实际机器人进行辨识和调整

    // 库伦摩擦系数 Fc (Nm) - 恒定摩擦力，与速度大小无关
    // 建议初始值：较小的关节用较小值，较大的关节用较大值
    // 参数辨识方法：让关节以极低速度匀速运动，测量所需的最小恒定力矩
    std::vector<double> Fc = {
        0.20,  // 关节1
        0.15,  // 关节2
        0.15,  // 关节3
        0.15,  // 关节4
        0.04,  // 关节5
        0.04   // 关节6
    };

    // 粘性摩擦系数 Fv (Nm·s/rad) - 线性速度相关摩擦系数
    // 建议初始值：通常比库伦摩擦小一个数量级
    // 参数辨识方法：让关节以不同速度匀速运动，测量力矩-速度曲线的斜率
    std::vector<double> Fv = {
        0.06,  // 关节1
        0.06,  // 关节2
        0.06,  // 关节3
        0.03,  // 关节4
        0.02,  // 关节5
        0.02   // 关节6
    };

    // 速度阈值 (rad/s) - 低于此速度时不使用库伦摩擦项
    // 建议值：0.01-0.05 rad/s
    double vel_threshold = 0.02;

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
            std::vector<double> f = robot.getFrictionCompensation(vel_current, Fc, Fv, vel_threshold);

            std::vector<double> tor(robot.getMotorCount());
            for (size_t i = 0; i < robot.getMotorCount(); ++i) {
                tor[i] = tor_impedance[i] + G[i] + f[i];
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

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
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
