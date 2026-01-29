/**
 * @file 3_gravity_compensation_with_fk.cpp
 * @brief 重力补偿控制并输出正运动学结果
 * 对应 Python 版本: 3_gravity_compensation_with_fk.py
 */

#include "panthera/Panthera.hpp"
#include <csignal>
#include <atomic>
#include <iostream>
#include <iomanip>
#include <vector>
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
void control_and_print_fk(panthera::Panthera& robot,
                          vector<double>& zero_pos, vector<double>& zero_vel,
                          vector<double>& zero_kp, vector<double>& zero_kd,
                          vector<double>& tau_limit)
{
    int motor_count = robot.getMotorCount();

    // 获取当前关节角度（对应Python: current_angles = robot.get_current_pos()）
    vector<double> current_angles = robot.getCurrentPos();

    // 计算重力补偿力矩（对应Python: gravity_torque = robot.get_Gravity(current_angles)）
    vector<double> gravity_torque = robot.getGravity(current_angles);

    // 力矩限幅（对应Python: gravity_torque = np.clip(gravity_torque, -tau_limit, tau_limit)）
    gravity_torque = robot.clipTorque(gravity_torque, tau_limit);

    // 应用重力补偿控制（对应Python: robot.pos_vel_tqe_kp_kd()）
    robot.posVelTorqueKpKd(zero_pos, zero_vel, gravity_torque, zero_kp, zero_kd);

    // 计算正运动学（对应Python: fk = robot.forward_kinematics(current_angles)）
    auto fk_result = robot.forwardKinematics(current_angles);

    // 输出结果
    cout << "\n" << string(80, '=') << endl;
    cout << "重力补偿控制 + 正运动学结果" << endl;
    cout << string(80, '=') << endl;

    // 显示关节角度（度）
    cout << "关节角度 (度): [";
    for (int i = 0; i < motor_count; ++i) {
        cout << fixed << setprecision(2) << setw(7)
             << current_angles[i] * 180.0 / M_PI;
        if (i < motor_count - 1) cout << ", ";
    }
    cout << "]" << endl;

    // 显示重力补偿力矩
    cout << "重力补偿力矩 (Nm): [";
    for (int i = 0; i < motor_count; ++i) {
        cout << fixed << setprecision(2) << setw(7) << gravity_torque[i];
        if (i < motor_count - 1) cout << ", ";
    }
    cout << "]" << endl;

    // 显示末端位置（对应Python: print(f"末端位置 (m): x={pos[0]:8.4f}...")）
    cout << "\n末端位置 (m):" << endl;
    cout << "  x = " << fixed << setprecision(4) << fk_result.position[0] << " m" << endl;
    cout << "  y = " << fixed << setprecision(4) << fk_result.position[1] << " m" << endl;
    cout << "  z = " << fixed << setprecision(4) << fk_result.position[2] << " m" << endl;

    // 显示旋转矩阵
    cout << "\n旋转矩阵 (R):" << endl;
    for (int i = 0; i < 3; ++i) {
        cout << "  [";
        for (int j = 0; j < 3; ++j) {
            cout << fixed << setprecision(3) << setw(8) << fk_result.rotation(i, j);
            if (j < 2) cout << ", ";
        }
        cout << "]" << endl;
    }

    // 显示欧拉角（对应Python: rotation_matrix_to_euler()）
    vector<double> euler = panthera::Panthera::rotationMatrixToEuler(fk_result.rotation);
    cout << "\n欧拉角 (度, ZYX):" << endl;
    cout << "  Roll  = " << fixed << setprecision(2) << euler[0] << "°" << endl;
    cout << "  Pitch = " << fixed << setprecision(2) << euler[1] << "°" << endl;
    cout << "  Yaw   = " << fixed << setprecision(2) << euler[2] << "°" << endl;

    cout << string(80, '-') << endl;
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

        // 力矩限幅
        vector<double> tau_limit = {15.0, 30.0, 30.0, 15.0, 5.0, 5.0};

        // 零控制参数
        vector<double> zero_kp(motor_count, 0.0);
        vector<double> zero_kd(motor_count, 0.0);
        vector<double> zero_pos(motor_count, 0.0);
        vector<double> zero_vel(motor_count, 0.0);

        cout << "\n开始重力补偿控制，同时输出正运动学结果..." << endl;
        cout << "按 Ctrl+C 停止程序" << endl;

        auto last_print_time = chrono::steady_clock::now();
        const double print_interval = 0.5; // 打印间隔（秒）

        // 主循环（对应Python: while True）
        while (!exitFlag.load()) {
            auto current_time = chrono::steady_clock::now();
            double elapsed = chrono::duration<double>(current_time - last_print_time).count();

            if (elapsed >= print_interval) {
                control_and_print_fk(robot, zero_pos, zero_vel, zero_kp, zero_kd, tau_limit);
                last_print_time = current_time;
            }

            this_thread::sleep_for(chrono::milliseconds(2));
        }

    } catch (const exception& e) {
        cerr << "错误: " << e.what() << endl;
        return 1;
    }

    cout << "\n程序结束，所有电机已停止" << endl;
    return 0;
}
