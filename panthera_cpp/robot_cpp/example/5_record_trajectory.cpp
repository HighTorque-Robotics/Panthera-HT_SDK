/**
 * @file 5_record_trajectory.cpp
 * @brief 单臂重力补偿程序 + 实时轨迹记录（位置+速度+夹爪）
 *
 * 记录格式：JSON Lines (.jsonl)
 * 每行一个JSON对象，包含时间戳、关节位置、速度、夹爪状态
 */

#include "panthera/Panthera.hpp"
#include <csignal>
#include <atomic>
#include <iostream>
#include <iomanip>
#include <vector>
#include <fstream>
#include <chrono>
#include <thread>
#include <cmath>

using namespace std;

// 全局原子变量，用于信号处理
std::atomic<bool> exitFlag(false);

// 信号处理函数
void signalHandler(int signum)
{
    exitFlag.store(true);
}

/**
 * @brief 轨迹记录器类
 */
class TrajectoryRecorder {
private:
    ofstream file_;
    string filename_;
    bool is_open_;
    chrono::steady_clock::time_point start_time_;

public:
    TrajectoryRecorder(const string& filename = "") : is_open_(false) {
        if (filename.empty()) {
            // 自动生成文件名（带时间戳）
            auto now = chrono::system_clock::now();
            auto time_t = chrono::system_clock::to_time_t(now);
            stringstream ss;
            ss << "trajectory_" << put_time(localtime(&time_t), "%Y%m%d_%H%M%S") << ".jsonl";
            filename_ = ss.str();
        } else {
            filename_ = filename;
        }

        file_.open(filename_, ios::out);
        if (file_.is_open()) {
            is_open_ = true;
            start_time_ = chrono::steady_clock::now();
            cout << "轨迹记录器已创建: " << filename_ << endl;
        } else {
            cerr << "无法创建文件: " << filename_ << endl;
        }
    }

    ~TrajectoryRecorder() {
        close();
    }

    /**
     * @brief 记录关节状态
     */
    void log(const vector<double>& positions,
             const vector<double>& velocities,
             double gripper_pos = 0.0,
             double gripper_vel = 0.0) {

        if (!is_open_) return;

        // 计算相对时间（秒）
        auto now = chrono::steady_clock::now();
        double timestamp = chrono::duration<double>(now - start_time_).count();

        // 写入JSON格式（与Python版本一致）
        file_ << "{";
        file_ << "\"t\":" << fixed << setprecision(6) << timestamp << ",";
        file_ << "\"pos\":[";
        for (size_t i = 0; i < positions.size(); ++i) {
            file_ << fixed << setprecision(6) << positions[i];
            if (i < positions.size() - 1) file_ << ",";
        }
        file_ << "],";
        file_ << "\"vel\":[";
        for (size_t i = 0; i < velocities.size(); ++i) {
            file_ << fixed << setprecision(6) << velocities[i];
            if (i < velocities.size() - 1) file_ << ",";
        }
        file_ << "],";
        file_ << "\"gripper_pos\":" << fixed << setprecision(6) << gripper_pos << ",";
        file_ << "\"gripper_vel\":" << fixed << setprecision(6) << gripper_vel;
        file_ << "}" << endl;
    }

    void close() {
        if (is_open_) {
            file_.close();
            is_open_ = false;
            cout << "轨迹已保存: " << filename_ << endl;
        }
    }

    bool isOpen() const { return is_open_; }
    string getFilename() const { return filename_; }
};

int main(int argc, char** argv)
{
    // 注册信号处理器
    std::signal(SIGINT, signalHandler);

    // 参数区
    bool DO_RECORD = true;
    string REC_FILE = "";  // 空字符串表示自动生成文件名

    try {

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

        // 摩擦补偿参数
        vector<double> Fc = {0.15, 0.12, 0.12, 0.12, 0.07, 0.07};
        vector<double> Fv = {0.05, 0.05, 0.05, 0.025, 0.035, 0.035};
        double vel_threshold = 0.02;

        // 力矩限幅
        vector<double> tau_limit = {15.0, 30.0, 30.0, 15.0, 5.0, 5.0};

        // 零控制参数
        vector<double> zero_pos(motor_count, 0.0);
        vector<double> zero_vel(motor_count, 0.0);
        vector<double> zero_kp(motor_count, 0.0);
        vector<double> zero_kd(motor_count, 0.0);

        // 实例化记录器
        TrajectoryRecorder* rec = nullptr;
        if (DO_RECORD) {
            rec = new TrajectoryRecorder(REC_FILE);
            cout << "\n开始记录轨迹（位置+速度+夹爪）..." << endl;
        }

        cout << "\n单臂重力补偿 + 轨迹记录" << endl;
        cout << string(60, '=') << endl;
        cout << "按 Ctrl+C 停止程序" << endl;
        cout << string(60, '=') << "\n" << endl;

        // 记录轨迹之前先循环发送读取指令
        cout << "初始化关节状态..." << endl;
        for (int i = 0; i < 10; ++i) {
            robot.send_get_motor_state_cmd();
            this_thread::sleep_for(chrono::milliseconds(100));
        }

        auto last_print_time = chrono::steady_clock::now();
        const double print_interval = 0.1;  // 打印间隔（秒）

        while (!exitFlag.load()) {
            // 获取当前状态
            vector<double> positions = robot.getCurrentPos();
            vector<double> velocities = robot.getCurrentVel();

            // 计算重力补偿力矩（使用 Panthera 类的方法）
            vector<double> gravity_torque = robot.getGravity(positions);

            // 添加摩擦补偿（使用 Panthera 类的方法）
            vector<double> friction_torque = robot.getFrictionCompensation(velocities, Fc, Fv, vel_threshold);

            // 总力矩
            vector<double> total_torque(motor_count);
            for (int i = 0; i < motor_count; ++i) {
                total_torque[i] = gravity_torque[i] + friction_torque[i];
            }

            // 力矩限幅（使用 Panthera 类的方法）
            total_torque = robot.clipTorque(total_torque, tau_limit);

            // 零刚度零阻尼控制（纯重力补偿模式，可自由拖动）
            robot.posVelTorqueKpKd(zero_pos, zero_vel, total_torque, zero_kp, zero_kd);

            // 记录关节位置速度 + 夹爪（暂时用0代替）
            if (DO_RECORD && rec && rec->isOpen()) {
                rec->log(positions, velocities, 0.0, 0.0);
            }

            // 打印状态（每0.1秒）
            auto current_time = chrono::steady_clock::now();
            double elapsed = chrono::duration<double>(current_time - last_print_time).count();

            if (elapsed >= print_interval) {
                cout << "\r";
                for (int i = 0; i < motor_count; ++i) {
                    cout << "J" << (i+1) << ": " << fixed << setprecision(3)
                              << setw(6) << positions[i] << "rad "
                              << setw(6) << velocities[i] << "rad/s | ";
                }
                cout << "夹爪: 0.000rad 0.000rad/s   " << flush;
                last_print_time = current_time;
            }

            this_thread::sleep_for(chrono::milliseconds(1));
        }

        // 清理
        if (rec) {
            delete rec;
        }

    } catch (const exception& e) {
        cerr << "\n错误: " << e.what() << endl;
        return 1;
    }

    cout << "\n\n程序结束，所有电机已停止" << endl;
    return 0;
}
