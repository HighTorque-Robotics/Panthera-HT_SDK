/**
 * @file 1_PD_control.cpp
 * @brief 简单的六关节机器人PD控制程序
 * 对应 Python 版本: 1_PD_control.py
 */

#include "panthera/Panthera.hpp"
#include <csignal>
#include <atomic>
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

using namespace std;
// 全局原子变量，用于信号处理函数
std::atomic<bool> exitFlag(false);

// 信号处理函数
void signalHandler(int signum)
{
    exitFlag.store(true);
}

int main()
{
    // 注册信号处理器
    std::signal(SIGINT, signalHandler);

    try {
        panthera::Panthera robot;

        int motor_count = robot.getMotorCount();

        // 定义控制参数（对应Python: pos1, kp, kd）
        vector<double> zero_pos(motor_count, 0.0);
        vector<double> zero_vel(motor_count, 0.0);
        vector<double> zero_torque(motor_count, 0.0);
        vector<double> pos1 = {0.0, 0.7, 0.7, -0.1, 0.0, 0.0};
        vector<double> kp = {4.0, 10.0, 10.0, 2.0, 2.0, 1.0};
        vector<double> kd = {0.5, 0.8, 0.8, 0.2, 0.2, 0.1};

        // 主循环（对应Python: while(!exitFlag.load())）
        while (!exitFlag.load()) {
            // 调用可复用函数（对应Python: robot.pos_vel_tqe_kp_kd()）
            robot.posVelTorqueKpKd(pos1, zero_vel, zero_torque, kp, kd);

            // 获取当前状态（对应Python: robot.get_current_pos()等）
            auto positions = robot.getCurrentPos();
            auto velocities = robot.getCurrentVel();
            auto torques = robot.getCurrentTorque();

            // 打印6个关节信息
            cout << fixed << setprecision(3);
            for (int i = 0; i < motor_count; ++i) {
                cout << "关节" << (i + 1) << ": "
                     << "位置=" << setw(7) << positions[i] << " rad, "
                     << "速度=" << setw(7) << velocities[i] << " rad/s, "
                     << "力矩=" << setw(7) << torques[i] << " Nm" << endl;
            }
            cout << string(60, '-') << endl;

            this_thread::sleep_for(chrono::seconds(1));
        }

    } catch (const exception& e) {
        cerr << "错误: " << e.what() << endl;
        return 1;
    }

    cout << "\n\n程序被中断\n所有电机已停止" << endl;
    return 0;
}
