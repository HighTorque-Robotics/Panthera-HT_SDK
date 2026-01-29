/**
 * @file 4_impedance_trajectory_control_with_gra_pd.cpp
 * @brief 基于轨迹的阻抗控制程序
 *
 * 使用七次多项式插值 + 重力补偿前馈的阻抗控制
 * 七次多项式保证位置、速度、加速度、加加速度连续
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

// 全局原子变量，用于信号处理
std::atomic<bool> exitFlag(false);

// 信号处理函数
void signalHandler(int signum)
{
    exitFlag.store(true);
}

/**
 * @brief 高精度延时函数
 */
void precise_sleep(double duration)
{
    if (duration <= 0) return;

    auto end_time = chrono::steady_clock::now() + chrono::duration<double>(duration);

    // 大部分时间用sleep（留1ms余量）
    if (duration > 0.001) {
        this_thread::sleep_for(chrono::duration<double>(duration - 0.001));
    }

    // 最后用忙等待保证精度
    while (chrono::steady_clock::now() < end_time) {
        // 忙等待
    }
}

/**
 * @brief 执行基于轨迹的阻抗控制
 */
void executeImpedanceTrajectory(
    panthera::Panthera& robot,
    const vector<vector<double>>& waypoints,
    const vector<double>& durations,
    const vector<double>& K,
    const vector<double>& B,
    const vector<double>& tau_limit,
    int control_rate = 200)
{
    int motor_count = robot.getMotorCount();
    vector<double> zero_pos(motor_count, 0.0);
    vector<double> zero_vel(motor_count, 0.0);
    vector<double> zero_kp(motor_count, 0.0);
    vector<double> zero_kd(motor_count, 0.0);

    for (size_t segment = 0; segment < durations.size(); ++segment) {
        const auto& start_pos = waypoints[segment];
        const auto& end_pos = waypoints[segment + 1];
        double duration = durations[segment];

        int steps = static_cast<int>(duration * control_rate);
        double dt = 1.0 / control_rate;

        auto segment_start = chrono::steady_clock::now();

        auto segment_print_time = chrono::steady_clock::now();
        const double segment_print_interval = 0.5;

        for (int step = 0; step < steps && !exitFlag.load(); ++step) {
            double current_time = step * dt;

            // 使用 Panthera 类的七次多项式插值方法
            vector<double> pos_des, vel_des, acc_des;
            panthera::Panthera::septicInterpolation(
                start_pos, end_pos, duration, current_time,
                pos_des, vel_des, acc_des);

            vector<double> q_current = robot.getCurrentPos();
            vector<double> vel_current = robot.getCurrentVel();

            // 阻抗控制力矩
            vector<double> tor_impedance(motor_count);
            for (int i = 0; i < motor_count; ++i) {
                double pos_error = pos_des[i] - q_current[i];
                double vel_error = vel_des[i] - vel_current[i];
                tor_impedance[i] = K[i] * pos_error + B[i] * vel_error;
            }

            // 重力补偿（使用 Panthera 类的方法）
            vector<double> G = robot.getGravity(q_current);

            // 总力矩 = 阻抗力矩 + 重力补偿
            vector<double> tor_total(motor_count);
            for (int i = 0; i < motor_count; ++i) {
                tor_total[i] = tor_impedance[i] + G[i];
            }

            // 力矩限幅
            tor_total = robot.clipTorque(tor_total, tau_limit);

            // 发送控制命令
            robot.posVelTorqueKpKd(zero_pos, zero_vel, tor_total, zero_kp, zero_kd);

            // 打印力矩
            auto now_time = chrono::steady_clock::now();
            double segment_elapsed = chrono::duration<double>(now_time - segment_print_time).count();
            if (segment_elapsed >= segment_print_interval) {
                cout << fixed << setprecision(3);
                cout << "力矩: [";
                for (int i = 0; i < 6; ++i)
                    cout << setw(6) << tor_total[i] << (i<5?", ":"");
                cout << "]" << endl;
                segment_print_time = now_time;
            }

            // 高精度等待
            auto target_time = segment_start + chrono::duration<double>((step + 1) * dt);
            auto now = chrono::steady_clock::now();
            if (now < target_time) {
                precise_sleep(chrono::duration<double>(target_time - now).count());
            }
        }

        if (exitFlag.load()) break;
    }

    // 移动到最终位置
    const auto& final_pos = waypoints.back();
    vector<double> move_vel(motor_count, 0.5);
    robot.posVelMaxTorque(final_pos, move_vel, tau_limit, true);
}

int main(int argc, char** argv)
{
    // 注册信号处理器
    std::signal(SIGINT, signalHandler);

    try {

        string config_path = "../robot_param/Follower.yaml";
        if (argc > 1) config_path = argv[1];

        panthera::Panthera robot(config_path);

        if (!robot.isModelLoaded()) {
            cerr << "错误: URDF模型未加载" << endl;
            return 1;
        }

        int motor_count = robot.getMotorCount();

        vector<double> K = {2.0, 4.0, 8.0, 1.0, 1.0, 1.0};
        vector<double> B = {0.5, 1.0, 1.40, 0.1, 0.1, 0.1};
        vector<double> tau_limit = {21.0, 36.0, 36.0, 21.0, 10.0, 10.0};

        vector<vector<double>> waypoints1 = {
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            {0.0, 0.9, 0.9, -0.10, 0.0, 0.0}
        };

        vector<vector<double>> waypoints2 = {
            {0.0, 0.9, 0.9, -0.10, 0.0, 0.0},
            {0.0, 1.4, 1.1, -0.5, 0.0, 0.0}
        };

        vector<vector<double>> waypoints3 = {
            {0.0, 1.4, 1.1, -0.5, 0.0, 0.0},
            {0.0, 0.9, 0.9, -0.10, 0.0, 0.0}
        };

        vector<double> durations1 = {3.0};

        cout << "\n基于轨迹的阻抗控制（七次多项式插值）" << endl;
        cout << string(60, '=') << endl;
        cout << "执行轨迹1: 零位 -> [0.0, 0.9, 0.9, -0.10, 0.0, 0.0]" << endl;
        executeImpedanceTrajectory(robot, waypoints1, durations1, K, B, tau_limit, 200);

        if (exitFlag.load()) goto end;

        cout << "执行轨迹2: -> [0.0, 1.4, 1.1, -0.5, 0.0, 0.0]" << endl;
        executeImpedanceTrajectory(robot, waypoints2, durations1, K, B, tau_limit, 200);

        if (exitFlag.load()) goto end;

        cout << "执行轨迹3: -> [0.0, 0.9, 0.9, -0.10, 0.0, 0.0]" << endl;
        executeImpedanceTrajectory(robot, waypoints3, durations1, K, B, tau_limit, 200);

        if (exitFlag.load()) goto end;

        cout << "\n开始定点阻抗控制..." << endl;
        cout << "按 Ctrl+C 退出..." << endl;
        cout << string(60, '=') << "\n" << endl;

        {
            vector<double> final_pos = {0.0, 0.9, 0.9, -0.10, 0.0, 0.0};
            vector<double> zero_pos(motor_count, 0.0);
            vector<double> zero_vel(motor_count, 0.0);
            vector<double> zero_kp(motor_count, 0.0);
            vector<double> zero_kd(motor_count, 0.0);
            vector<double> zero_des_vel(motor_count, 0.0);

            auto last_print_time = chrono::steady_clock::now();
            const double print_interval = 0.5;

            while (!exitFlag.load()) {
                vector<double> q_current = robot.getCurrentPos();
                vector<double> vel_current = robot.getCurrentVel();

                // 定点阻抗控制
                vector<double> tor_impedance(motor_count);
                for (int i = 0; i < motor_count; ++i) {
                    double pos_error = final_pos[i] - q_current[i];
                    double vel_error = zero_des_vel[i] - vel_current[i];
                    tor_impedance[i] = K[i] * pos_error + B[i] * vel_error;
                }

                // 重力补偿
                vector<double> G = robot.getGravity(q_current);

                // 总力矩
                vector<double> tor_total(motor_count);
                for (int i = 0; i < motor_count; ++i) {
                    tor_total[i] = tor_impedance[i] + G[i];
                }

                tor_total = robot.clipTorque(tor_total, tau_limit);

                robot.posVelTorqueKpKd(zero_pos, zero_vel, tor_total, zero_kp, zero_kd);

                auto current_time = chrono::steady_clock::now();
                double elapsed = chrono::duration<double>(current_time - last_print_time).count();

                if (elapsed >= print_interval) {
                    cout << fixed << setprecision(3);
                    cout << "位置: [";
                    for (int i = 0; i < 6; ++i)
                        cout << setw(6) << q_current[i] << (i<5?", ":"");
                    cout << "]" << endl;
                    cout << "总力矩: [";
                    for (int i = 0; i < 6; ++i)
                        cout << setw(6) << tor_total[i] << (i<5?", ":"");
                    cout << "]" << endl;
                    cout << string(60, '-') << endl;
                    last_print_time = current_time;
                }

                this_thread::sleep_for(chrono::milliseconds(2));
            }
        }

end:
    {}  // 空语句用于标签

    } catch (const exception& e) {
        cerr << "错误: " << e.what() << endl;
        return 1;
    }

    cout << "\n\n程序结束" << endl;
    return 0;
}
