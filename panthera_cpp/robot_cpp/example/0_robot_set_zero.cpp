/**
 * @file 0_robot_set_zero.cpp
 * @brief 设置机械臂零位并显示状态
 *
 * 设置机器人零位，然后实时显示6个关节和夹爪的当前状态
 */

#include "panthera/Panthera.hpp"
#include <csignal>
#include <atomic>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

// 全局原子变量，用于信号处理函数
std::atomic<bool> exitFlag(false);

// 信号处理函数
void signalHandler(int signum)
{
    exitFlag.store(true);
}

/**
 * @brief 打印机器人状态信息
 * @param robot Panthera机械臂对象引用
 */
void printRobotState(panthera::Panthera& robot)
{
    // 获取关节角度
    robot.send_get_motor_state_cmd();
    robot.motor_send_cmd();

    auto positions = robot.getCurrentPos();
    auto velocities = robot.getCurrentVel();

    // 获取夹爪状态
    double gripper_pos = robot.getCurrentPosGripper();
    double gripper_vel = robot.getCurrentVelGripper();

    std::cout << "\n" << std::string(50, '=') << std::endl;
    std::cout << "机械臂状态信息" << std::endl;
    std::cout << std::string(50, '=') << std::endl;

    // 打印关节信息
    std::cout << std::fixed << std::setprecision(3);
    for (int i = 0; i < robot.getMotorCount(); ++i) {
        std::cout << "关节" << (i + 1) << ": "
                  << "位置=" << std::setw(7) << positions[i] << " rad, "
                  << "速度=" << std::setw(7) << velocities[i] << " rad/s" << std::endl;
    }

    // 打印夹爪信息
    std::cout << "夹爪:   "
              << "位置=" << std::setw(7) << gripper_pos << " rad, "
              << "速度=" << std::setw(7) << gripper_vel << " rad/s" << std::endl;
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

        // 设置零位
        std::cout << "设置机器人零位..." << std::endl;
        robot.set_reset_zero();
        robot.motor_send_cmd();
        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::cout << "开始获取机器人状态，按 Ctrl+C 退出..." << std::endl;

        // 主循环：每0.5秒更新一次
        while (!exitFlag.load()) {
            printRobotState(robot);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
