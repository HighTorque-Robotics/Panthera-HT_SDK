/**
 * @file 0_robot_get_state.cpp
 * @brief 获取并打印机械臂关节角度信息
 * 对应 Python 版本: 0_robot_get_state.py
 */

#include "panthera/Panthera.hpp"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>

using namespace std;

// 全局原子变量，用于信号处理函数
std::atomic<bool> exitFlag(false);

// 信号处理函数
void signalHandler(int signum)
{
    exitFlag.store(true);
}

// 打印机器人状态信息
void print_robot_state(panthera::Panthera& robot)
{
    robot.send_get_motor_state_cmd();
    robot.motor_send_cmd();

    auto positions = robot.getCurrentPos();
    auto velocities = robot.getCurrentVel();
    auto torques = robot.getCurrentTorque();

    double gripper_pos = robot.getCurrentPosGripper();
    double gripper_vel = robot.getCurrentVelGripper();

    cout << "\n" << string(50, '=') << endl;
    cout << "机械臂状态信息" << endl;
    cout << string(50, '=') << endl;

    for (int i = 0; i < robot.getMotorCount(); ++i) {
        cout << "关节" << (i + 1) << ": "
             << "位置=" << fixed << setw(7) << setprecision(3) << positions[i] << " rad, "
             << "速度=" << setw(7) << velocities[i] << " rad/s, "
             << "力矩=" << setw(7) << torques[i] << " Nm" << endl;
    }

    cout << "夹爪  : "
         << "位置=" << fixed << setw(7) << setprecision(3) << gripper_pos << " rad, "
         << "速度=" << setw(7) << gripper_vel << " rad/s" << endl;
}

int main()
{
    // 注册信号处理器
    std::signal(SIGINT, signalHandler);

    try {
        panthera::Panthera robot;

        cout << "\n机器人状态获取程序已启动" << endl;
        cout << "按 Ctrl+C 退出程序" << endl;
        cout << string(60, '=') << endl;

        while (!exitFlag.load()) {
            print_robot_state(robot);

            // 可中断的睡眠
            for (int i = 0; i < 10; ++i) {  // 100ms = 10 * 10ms
                if (exitFlag.load()) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

    } catch (const std::exception& e) {
        cerr << "\n错误: " << e.what() << endl;
        return 1;
    }

    cout << "\n程序已安全退出" << endl;
    return 0;
}
