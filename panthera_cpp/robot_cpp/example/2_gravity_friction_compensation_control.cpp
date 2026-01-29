/**
 * @file 2_gravity_friction_compensation_control.cpp
 * @brief 重力 + 摩擦力补偿控制程序
 *
 * 实现重力和摩擦力补偿：
 * - 使用 Pinocchio 库计算真实的重力补偿力矩
 * - 使用库伦摩擦 + 粘性摩擦模型计算摩擦力补偿
 *
 * 摩擦模型：
 *   τ_friction = Fc * sign(vel) + Fv * vel
 *   当 |vel| < vel_threshold 时，只使用粘性摩擦项避免抖动
 */

#include "panthera/Panthera.hpp"
#include <csignal>
#include <atomic>
#include <iostream>
#include <iomanip>
#include <vector>
#include <thread>

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
        // 创建机械臂对象
        string config_path = "../robot_param/Follower.yaml";
        if (argc > 1) {
            config_path = argv[1];
        }

        panthera::Panthera robot(config_path);

        if (!robot.isModelLoaded()) {
            cerr << "错误: URDF模型未加载" << endl;
            return 1;
        }

        int motor_count = robot.getMotorCount();

        // ==================== 摩擦参数配置 ====================
        // 注意：这些参数需要根据实际机器人进行辨识和调整

        // 库伦摩擦系数 Fc (Nm) - 恒定摩擦力，与速度大小无关
        // 建议初始值：较小的关节用较小值，较大的关节用较大值
        // 参数辨识方法：让关节以极低速度匀速运动，测量所需的最小恒定力矩
        vector<double> Fc = {
            0.20,  // 关节1
            0.15,  // 关节2
            0.15,  // 关节3
            0.12,  // 关节4
            0.07,  // 关节5
            0.07   // 关节6
        };

        // 粘性摩擦系数 Fv (Nm·s/rad) - 线性速度相关摩擦系数
        // 建议初始值：通常比库伦摩擦小一个数量级
        // 参数辨识方法：让关节以不同速度匀速运动，测量力矩-速度曲线的斜率
        vector<double> Fv = {
            0.06,  // 关节1
            0.06,  // 关节2
            0.06,  // 关节3
            0.025, // 关节4
            0.035, // 关节5
            0.035  // 关节6
        };

        // 速度阈值 (rad/s) - 低于此速度时不使用库伦摩擦项
        double vel_threshold = 0.02;

        // ====================================================

        // 定义零控制参数
        vector<double> zero_kp(motor_count, 0.0);
        vector<double> zero_kd(motor_count, 0.0);
        vector<double> zero_pos(motor_count, 0.0);
        vector<double> zero_vel(motor_count, 0.0);

        // 力矩限幅（基于电机规格）
        vector<double> tau_limit = {15.0, 30.0, 30.0, 15.0, 5.0, 5.0};

        cout << "\n" << string(60, '=') << endl;
        cout << "重力 + 摩擦力补偿控制" << endl;
        cout << string(60, '=') << endl;
        cout << "\n摩擦参数：" << endl;
        cout << "库伦摩擦系数 Fc: [";
        for (size_t i = 0; i < Fc.size(); ++i) {
            cout << Fc[i] << (i < Fc.size() - 1 ? ", " : "");
        }
        cout << "] Nm" << endl;

        cout << "粘性摩擦系数 Fv: [";
        for (size_t i = 0; i < Fv.size(); ++i) {
            cout << Fv[i] << (i < Fv.size() - 1 ? ", " : "");
        }
        cout << "] Nm·s/rad" << endl;

        cout << "速度阈值: " << vel_threshold << " rad/s" << endl;
        cout << "\n说明: 使用 Pinocchio 计算重力补偿，使用库伦+粘性模型计算摩擦补偿" << endl;
        cout << "按 Ctrl+C 退出..." << endl;
        cout << string(60, '=') << "\n" << endl;

        auto last_print_time = chrono::steady_clock::now();
        const double print_interval = 0.5; // 打印间隔（秒）

        // 主循环
        while (!exitFlag.load()) {
            // 1. 获取当前关节速度
            vector<double> vel = robot.getCurrentVel();

            // 2. 获取当前关节位置
            vector<double> q_current = robot.getCurrentPos();

            // 3. 计算重力补偿力矩（使用 Panthera 类的方法）
            vector<double> tau_gravity = robot.getGravity(q_current);

            // 4. 计算摩擦力补偿力矩（使用 Panthera 类的方法）
            vector<double> tau_friction = robot.getFrictionCompensation(vel, Fc, Fv, vel_threshold);

            // 5. 总补偿力矩 = 重力补偿 + 摩擦力补偿
            vector<double> tau_total(motor_count);
            for (int i = 0; i < motor_count; ++i) {
                tau_total[i] = tau_gravity[i] + tau_friction[i];
            }

            // 6. 力矩限幅（使用 Panthera 类的方法）
            tau_total = robot.clipTorque(tau_total, tau_limit);

            // 7. 发送控制指令
            robot.posVelTorqueKpKd(zero_pos, zero_vel, tau_total, zero_kp, zero_kd);

            // 8. 定期打印状态信息
            auto current_time = chrono::steady_clock::now();
            double elapsed = chrono::duration<double>(current_time - last_print_time).count();

            if (elapsed >= print_interval) {
                cout << fixed << setprecision(3);
                cout << "速度: [";
                for (int i = 0; i < motor_count; ++i) {
                    cout << setw(6) << vel[i];
                    if (i < motor_count - 1) cout << ", ";
                }
                cout << "] rad/s" << endl;

                cout << "重力补偿: [";
                for (int i = 0; i < motor_count; ++i) {
                    cout << setw(6) << tau_gravity[i];
                    if (i < motor_count - 1) cout << ", ";
                }
                cout << "] Nm" << endl;

                cout << "摩擦补偿: [";
                for (int i = 0; i < motor_count; ++i) {
                    cout << setw(6) << tau_friction[i];
                    if (i < motor_count - 1) cout << ", ";
                }
                cout << "] Nm" << endl;

                cout << "总补偿力矩: [";
                for (int i = 0; i < motor_count; ++i) {
                    cout << setw(6) << tau_total[i];
                    if (i < motor_count - 1) cout << ", ";
                }
                cout << "] Nm" << endl;
                cout << string(60, '-') << endl;

                last_print_time = current_time;
            }

            // 控制频率：200Hz
            this_thread::sleep_for(chrono::milliseconds(5));
        }

    } catch (const exception& e) {
        cerr << "错误: " << e.what() << endl;
        return 1;
    }

    cout << "\n\n所有电机已停止" << endl;
    return 0;
}
