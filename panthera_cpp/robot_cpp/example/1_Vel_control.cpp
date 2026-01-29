#include "panthera/Panthera.hpp"
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <vector>
#include <csignal>
#include <atomic>

using namespace panthera;

// 全局原子变量，用于信号处理函数
std::atomic<bool> exitFlag(false);

// 信号处理函数
void signalHandler(int signum)
{
    exitFlag.store(true);
}

/**
 * @brief 简单的单关节速度控制程序
 * 第一个关节每隔3秒在正负0.2 rad/s之间切换
 */
int main()
{
    // 注册信号处理器
    std::signal(SIGINT, signalHandler);

    try {
        Panthera robot;

        while (!exitFlag.load()) {
            // 获取当前时间（秒）
            auto now = std::chrono::steady_clock::now();
            auto duration = now.time_since_epoch();
            double t = std::chrono::duration<double>(duration).count();

            // 每6秒一个周期，前3秒正速度，后3秒负速度
            int cycle = static_cast<int>(std::floor(t)) % 6;
            double target_vel_1 = (cycle >= 3) ? 0.2 : -0.2;

            // 构建目标速度数组
            std::vector<double> target_vel(robot.getMotorCount(), 0.0);
            target_vel[0] = target_vel_1;

            // 发送速度控制指令
            robot.jointVel(target_vel);

            // 获取当前状态
            std::vector<double> current_pos = robot.getCurrentPos();
            std::vector<double> current_vel = robot.getCurrentVel();

            // 打印状态
            std::cout << "目标速度: " << target_vel_1 << " rad/s" << std::endl;
            std::cout << "当前位置: " << current_pos[0] << " rad" << std::endl;
            std::cout << "当前速度: " << current_vel[0] << " rad/s" << std::endl;
            std::cout << std::string(40, '-') << std::endl;

            // 可中断的睡眠
            for (int i = 0; i < 10; ++i) {  // 100ms = 10 * 10ms
                if (exitFlag.load()) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

    } catch (const std::exception& e) {
        std::cout << "\n错误: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "\n程序已退出" << std::endl;
    return 0;
}
