/**
 * @file 1_PosVel_control.cpp
 * @brief 简单的六关节机器人位置速度控制程序
 *
 * 演示如何通过目标位置数组控制机器人运动
 * 执行一系列预定义的位置控制命令，包括夹爪开合操作
 * 对应 Python 版本: 1_PosVel_control.py
 */

#include "panthera/Panthera.hpp"
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>

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
        // 创建机械臂对象
        std::string config_path = "../robot_param/Follower.yaml";
        if (argc > 1) {
            config_path = argv[1];
        }

        panthera::Panthera robot(config_path);

        int motor_count = robot.getMotorCount();

        // 定义位置控制参数
        std::vector<double> zero_pos(motor_count, 0.0);
        std::vector<double> pos1 = {0.0, 0.8, 0.8, 0.3, 0.0, 0.0};
        std::vector<double> pos2 = {0.0, 1.2, 1.2, 0.4, 0.0, 0.0};
        std::vector<double> vel(motor_count, 0.5);
        std::vector<double> max_torque = {21.0, 36.0, 36.0, 21.0, 10.0, 10.0};

        std::cout << "\n位置速度控制程序" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "发送控制命令..." << std::endl;

        // 控制序列
        // 1. 移动到零位
        if (!exitFlag.load()) {
            bool zero_success = robot.posVelMaxTorque(zero_pos, vel, max_torque, true);
            std::cout << "执行状态0：" << (zero_success ? "成功" : "失败") << std::endl;
        }

        // 可中断的睡眠辅助函数
        auto sleep = [](int ms) {
            for (int i = 0; i < ms / 10; ++i) {
                if (exitFlag.load()) return;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        };

        sleep(1000);

        // 2. 移动到位置2，夹爪闭合
        if (!exitFlag.load()) {
            std::cout << "\n移动到位置2，夹爪闭合..." << std::endl;
            robot.posVelMaxTorque(pos2, vel, max_torque, true);
            robot.gripperClose(0.0, 0.5, 0.5);
            sleep(2000);
        }

        // 3. 移动到位置1，夹爪打开
        if (!exitFlag.load()) {
            std::cout << "移动到位置1，夹爪打开..." << std::endl;
            robot.posVelMaxTorque(pos1, vel, max_torque, true);
            robot.gripperOpen(0.5, 0.5);
            sleep(2000);
        }

        // 4. 移动到位置2，夹爪闭合
        if (!exitFlag.load()) {
            std::cout << "移动到位置2，夹爪闭合..." << std::endl;
            robot.posVelMaxTorque(pos2, vel, max_torque, true);
            robot.gripperClose(0.0, 0.5, 0.5);
            sleep(2000);
        }

        // 5. 返回零位
        if (!exitFlag.load()) {
            std::cout << "\n返回零位..." << std::endl;
            bool zero_success = robot.posVelMaxTorque(zero_pos, vel, max_torque, true);
            std::cout << "执行状态0：" << (zero_success ? "成功" : "失败") << std::endl;
            sleep(2000);
        }

        if (!exitFlag.load()) {
            std::cout << "\n保持位置2秒..." << std::endl;
            sleep(2000);
            std::cout << "\n运动完成！结束后电机会自动掉电，请注意安全！！" << std::endl;
        } else {
            std::cout << "\n程序被中断，运动已停止" << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "\n\n所有电机已停止" << std::endl;
    return 0;
}
