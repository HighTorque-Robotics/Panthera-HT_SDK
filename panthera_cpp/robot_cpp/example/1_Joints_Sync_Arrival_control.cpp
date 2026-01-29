#include "panthera/Panthera.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <csignal>
#include <atomic>

using namespace std::chrono_literals;

// 全局原子变量，用于信号处理函数
std::atomic<bool> exitFlag(false);

// 信号处理函数
void signalHandler(int signum)
{
    exitFlag.store(true);
}

/**
 * @brief 简单的多关节机器人位置时间控制程���
 * 通过指定运动时间，自动计算速度实现所有关节同时到达目标位置
 */
int main()
{
    // 注册信号处理器
    std::signal(SIGINT, signalHandler);

    try {
        panthera::Panthera robot;

        // 定义目标位置（弧度）
        std::vector<double> zero_pos(robot.getMotorCount(), 0.0);
        std::vector<double> pos1 = {0.0, 0.8, 0.8, 0.3, 0.0, 0.0};
        std::vector<double> pos2 = {0.0, 1.2, 1.2, 0.4, 0.0, 0.0};
        std::vector<double> pos3 = {0.0, 0.0, 0.0, 0.0, 0.0, 2.0};

        // 定义最大力矩（Nm）
        std::vector<double> max_torque = {21.0, 36.0, 36.0, 21.0, 10.0, 10.0};

        // 可中断的睡眠辅助函数
        auto sleep = [](int ms) {
            for (int i = 0; i < ms / 10; ++i) {
                if (exitFlag.load()) return;
                std::this_thread::sleep_for(10ms);
            }
        };

        // 发送位置时间控制命令（使用阻塞模式）
        std::cout << "\n发送控制命令..." << std::endl;
        if (!exitFlag.load()) {
            bool zero_success = robot.jointsSyncArrival(zero_pos, 2.0, max_torque, true);
            std::cout << "执行状态0：" << (zero_success ? "成功" : "失败") << std::endl;
            sleep(1000);
        }

        // 运动到位置1，使用3秒到达
        if (!exitFlag.load()) {
            robot.jointsSyncArrival(pos1, 3.0, max_torque, true);
            robot.gripperClose();
            sleep(2000);
        }

        // 运动到位置2，使用2.5秒到达
        if (!exitFlag.load()) {
            robot.jointsSyncArrival(pos2, 2.5, max_torque, true);
            robot.gripperOpen();
            sleep(2000);
        }

        // 运动到位置1，使用3秒到达
        if (!exitFlag.load()) {
            robot.jointsSyncArrival(pos1, 3.0, max_torque, true);
            robot.gripperClose();
            sleep(2000);
        }

        // 回到零位，使用2秒到达
        if (!exitFlag.load()) {
            bool zero_success = robot.jointsSyncArrival(zero_pos, 2.0, max_torque, true);
            std::cout << "执行状态0：" << (zero_success ? "成功" : "失败") << std::endl;
            sleep(2000);
        }

        if (!exitFlag.load()) {
            // 保持位置2秒
            std::cout << "\n保持位置2秒..." << std::endl;
            sleep(2000);
            std::cout << "\n运动完成！结束后电机会自动掉电，请注意安全！！" << std::endl;
        } else {
            std::cout << "\n程序被中断" << std::endl;
        }

    } catch (const std::exception& e) {
        std::cout << "\n错误: " << e.what() << std::endl;
        std::cout << "\n\n所有电机已停止" << std::endl;
        return 1;
    }

    std::cout << "\n\n所有电机已停止" << std::endl;
    return 0;
}
