/**
 * @file 5_teleop_control.cpp
 * @brief 主从臂遥操作程序
 *
 * 主臂控制从臂，支持力反馈（可选）
 * 对应 Python 版本: 5_teleop_control.py
 */

#include "panthera/Panthera.hpp"
#include <csignal>
#include <atomic>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
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

int main(int argc, char** argv)
{
    // 注册信号处理器
    std::signal(SIGINT, signalHandler);

    try {
        // 初始化信号处理函数器

        // 主臂配置文件路径
        string leader_config = "../robot_param/Leader.yaml";
        // 从臂配置文件路径
        string follower_config = "../robot_param/Follower.yaml";

        if (argc > 1) {
            leader_config = argv[1];
        }
        if (argc > 2) {
            follower_config = argv[2];
        }

        // 创建两个机器人实例
        cout << "创建主臂..." << endl;
        panthera::Panthera leader(leader_config);

        cout << "创建从臂..." << endl;
        panthera::Panthera follower(follower_config);

        if (!leader.isModelLoaded() || !follower.isModelLoaded()) {
            cerr << "错误: URDF模型未加载" << endl;
            return 1;
        }

        int motor_count = leader.getMotorCount();

        // 零控制参数
        vector<double> zero_pos(motor_count, 0.0);
        vector<double> zero_vel(motor_count, 0.0);
        vector<double> zero_tor(motor_count, 0.0);
        vector<double> zero_kp(motor_count, 0.0);
        vector<double> zero_kd(motor_count, 0.0);

        // PD增益（从臂跟踪）
        vector<double> kp = {10.0, 21.0, 21.0, 16.0, 13.0, 1.0};
        vector<double> kd = {1.0, 2.0, 2.0, 0.9, 0.8, 0.1};

        // 夹爪PD增益
        double gripper_kp = 4.0;
        double gripper_kd = 0.4;

        // 摩擦补偿参数
        vector<double> Fc = {0.15, 0.12, 0.12, 0.12, 0.04, 0.04};
        vector<double> Fv = {0.05, 0.05, 0.05, 0.03, 0.02, 0.02};
        double vel_threshold = 0.02;

        // 力反馈阈值
        vector<double> tor_threshold = {0.5, 1.0, 1.0, 0.5, 0.3, 0.3};

        // 力矩限幅
        vector<double> tau_limit = {15.0, 30.0, 30.0, 15.0, 5.0, 5.0};

        // 是否启用力反馈模式
        bool force_feedback_mode = true;  // 默认使用无力反馈模式（更丝滑）

        cout << "\n主从臂遥操作控制" << endl;
        cout << string(60, '=') << endl;
        cout << "主臂: " << leader_config << endl;
        cout << "从臂: " << follower_config << endl;
        cout << "力反馈模式: " << (force_feedback_mode ? "开启" : "关闭") << endl;
        cout << "\n按 Ctrl+C 停止..." << endl;
        cout << string(60, '=') << "\n" << endl;

        while (!exitFlag.load()) {
            // ====== 关节控制 ======

            // 获取主臂位置速度作为从臂同步信息
            vector<double> leader_pos = leader.getCurrentPos();
            vector<double> leader_vel = leader.getCurrentVel();
            vector<double> follower_vel = follower.getCurrentVel();

            // 获取从臂力矩反馈
            vector<double> follower_torque = follower.getCurrentTorque();

            // 计算重力补偿（使用 Panthera 类的方法）
            vector<double> leader_gra = leader.getGravity(leader_pos);
            vector<double> follower_pos = follower.getCurrentPos();
            vector<double> follower_gra = follower.getGravity(follower_pos);

            // 计算从臂受到的外力
            vector<double> tor_diff(motor_count);
            for (int i = 0; i < motor_count; ++i) {
                tor_diff[i] = follower_torque[i] - follower_gra[i];
                // 对每个元素单独判断，小于阈值的设为 0
                if (abs(tor_diff[i]) < tor_threshold[i]) {
                    tor_diff[i] = 0.0;
                }
            }

            // 主臂力矩
            vector<double> leader_friction = leader.getFrictionCompensation(leader_vel, Fc, Fv, vel_threshold);
            vector<double> leader_tor(motor_count);

            if (force_feedback_mode) {
                // 力反馈模式：主臂感受到从臂的外力
                for (int i = 0; i < motor_count; ++i) {
                    leader_tor[i] = leader_gra[i] + leader_friction[i] - tor_diff[i] * 0.8;
                }
            } else {
                // 无力反馈模式（更丝滑）
                for (int i = 0; i < motor_count; ++i) {
                    leader_tor[i] = leader_gra[i] + leader_friction[i];
                }
            }

            // 从臂力矩
            vector<double> follower_friction = follower.getFrictionCompensation(follower_vel, Fc, Fv, vel_threshold);
            vector<double> follower_tor(motor_count);
            for (int i = 0; i < motor_count; ++i) {
                follower_tor[i] = follower_gra[i] + follower_friction[i];
            }

            // 力矩限幅（使用 Panthera 类的方法）
            leader_tor = leader.clipTorque(leader_tor, tau_limit);
            follower_tor = follower.clipTorque(follower_tor, tau_limit);

            // 运行控制
            leader.posVelTorqueKpKd(zero_pos, zero_vel, leader_tor, zero_kp, zero_kd);
            follower.posVelTorqueKpKd(leader_pos, leader_vel, follower_tor, kp, kd);

            // ====== 夹爪控制 ======
            // 获取主臂夹爪位置和速度
            double leader_gripper_pos = leader.getCurrentPosGripper();
            double leader_gripper_vel = leader.getCurrentVelGripper();
            double follower_gripper_torque = follower.getCurrentTorqueGripper();

            // 计算夹爪摩擦补偿
            double gripper_Fc = 0.06;
            double gripper_Fv = 0.0;
            double gripper_vel_threshold = 0.15;

            vector<double> leader_gripper_vel_vec = {leader_gripper_vel};
            vector<double> gripper_Fc_vec = {gripper_Fc};
            vector<double> gripper_Fv_vec = {gripper_Fv};
            vector<double> gripper_friction_vec = leader.getFrictionCompensation(
                leader_gripper_vel_vec, gripper_Fc_vec, gripper_Fv_vec, gripper_vel_threshold);
            double gripper_friction = gripper_friction_vec[0];

            // 计算夹爪力矩
            double gripper_torque = gripper_friction - follower_gripper_torque * 0.5;

            // 对夹爪力矩应用阈值
            if (abs(gripper_torque) < 0.2) {
                gripper_torque = 0.0;
            }

            // 发送夹爪控制命令
            leader.gripperControlMIT(1.5, 0.0, gripper_torque, 0.2, 0.02);
            follower.gripperControlMIT(leader_gripper_pos, leader_gripper_vel, 0.0, gripper_kp, gripper_kd);

            // ====== 打印信息 ======
            for (int i = 0; i < motor_count; ++i) {
                cout << "关节" << (i+1) << ": 位置=" << fixed << setprecision(3)
                          << setw(7) << leader_pos[i] << " rad, "
                          << "速度=" << setw(7) << leader_vel[i] << " rad/s" << endl;
            }

            cout << "反馈力矩: [";
            for (int i = 0; i < motor_count; ++i) {
                cout << fixed << setprecision(3) << setw(6) << tor_diff[i];
                if (i < motor_count - 1) cout << ", ";
            }
            cout << "]" << endl;
            cout << "夹爪力矩: " << fixed << setprecision(3) << gripper_torque << " Nm" << endl;
            cout << "主臂夹爪位置: " << fixed << setprecision(3) << leader_gripper_pos << " rad" << endl;
            cout << "从臂夹爪位置: " << fixed << setprecision(3) << follower.getCurrentPosGripper() << " rad" << endl;
            cout << string(40, '-') << endl;

            this_thread::sleep_for(chrono::milliseconds(1));
        }

    } catch (const exception& e) {
        cerr << "错误: " << e.what() << endl;
        return 1;
    }

    cout << "\n\n程序结束，所有电机已停止" << endl;
    return 0;
}
