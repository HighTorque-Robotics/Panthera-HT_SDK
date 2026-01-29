/**
 * @file 2_gravity_compensation_control.cpp
 * @brief 重力补偿控制程序
 * 对应 Python 版本: 2_gravity_compensation_control.py
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


// 主控制函数（对应Python的main函数）
void control_loop(panthera::Panthera& robot,
                  vector<double>& zero_pos, vector<double>& zero_vel,
                  vector<double>& zero_kp, vector<double>& zero_kd,
                  vector<double>& tau_limit)
{
    int motor_count = robot.getMotorCount();

    // 调用函数获取重力补偿力矩（对应Python: tor = robot.get_Gravity()）
    vector<double> gravity_torque = robot.getGravity();

    // 力矩限幅（对应Python: tor = np.clip(tor, -tau_limit, tau_limit)）
    gravity_torque = robot.clipTorque(gravity_torque, tau_limit);

    // 发送控制命令（对应Python: robot.pos_vel_tqe_kp_kd()）
    robot.posVelTorqueKpKd(zero_pos, zero_vel, gravity_torque, zero_kp, zero_kd);
    robot.gripperControlMIT(0, 0, 0, 0, 0);

    // 打印重力补偿力矩（对应Python: print(f"重力补偿力矩：",tor)）
    cout << "重力补偿力矩: [";
    for (int i = 0; i < motor_count; ++i) {
        cout << fixed << setprecision(3) << gravity_torque[i];
        if (i < motor_count - 1) cout << ", ";
    }
    cout << "]" << endl;
}

int main()
{
    // 注册信号处理器
    std::signal(SIGINT, signalHandler);

    try {
        panthera::Panthera robot;

        if (!robot.isModelLoaded()) {
            cerr << "错误: URDF模型未加载" << endl;
            return 1;
        }

        int motor_count = robot.getMotorCount();

        // 创建零位置和零速度数组（对应Python: zero_pos = [0.0] * robot.motor_count）
        vector<double> zero_pos(motor_count, 0.0);
        vector<double> zero_vel(motor_count, 0.0);
        vector<double> zero_kp(motor_count, 0.0);
        vector<double> zero_kd(motor_count, 0.0);
        vector<double> tau_limit = {15.0, 30.0, 30.0, 15.0, 5.0, 5.0};

        // 主循环（对应Python: while(!exitFlag.load())）
        while (!exitFlag.load()) {
            control_loop(robot, zero_pos, zero_vel, zero_kp, zero_kd, tau_limit);
            this_thread::sleep_for(chrono::milliseconds(2));
        }

    } catch (const exception& e) {
        cerr << "错误: " << e.what() << endl;
        return 1;
    }

    cout << "\n程序被中断\n所有电机已停止" << endl;
    return 0;
}
