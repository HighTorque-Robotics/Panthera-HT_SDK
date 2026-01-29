#include "panthera/Panthera.hpp"
#include <csignal>
#include <atomic>
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <Eigen/Dense>

using namespace panthera;
using namespace std::chrono_literals;

// 全局原子变量，用于信号处理函数
std::atomic<bool> exitFlag(false);

// 信号处理函数
void signalHandler(int signum)
{
    exitFlag.store(true);
}

/**
 * @brief 正运动学测试程序
 * 实时获取并打印机器人末端位置和姿态
 */

void printMatrix(const Eigen::Matrix3d& matrix, const std::string& title, int precision = 3)
{
    std::cout << "\n" << title << ":" << std::endl;
    for (int i = 0; i < 3; ++i) {
        std::cout << "  [";
        for (int j = 0; j < 3; ++j) {
            std::cout << std::setw(8) << std::setprecision(precision) << std::fixed << matrix(i, j);
            if (j < 2) std::cout << "  ";
        }
        std::cout << "]" << std::endl;
    }
}

void printMatrix(const Eigen::Matrix4d& matrix, const std::string& title, int precision = 4)
{
    std::cout << "\n" << title << ":" << std::endl;
    for (int i = 0; i < 4; ++i) {
        std::cout << "  [";
        for (int j = 0; j < 4; ++j) {
            std::cout << std::setw(8) << std::setprecision(precision) << std::fixed << matrix(i, j);
            if (j < 3) std::cout << "  ";
        }
        std::cout << "]" << std::endl;
    }
}

int main()
{
    // 注册信号处理器
    std::signal(SIGINT, signalHandler);

    Panthera robot;

    try {
        while(!exitFlag.load()) {
            // 获取当前末端位置和姿态
            // 利用运控模式发送控制帧以读取电机反馈状态
            robot.send_get_motor_state_cmd();
            robot.motor_send_cmd();

            std::vector<double> current_angles = robot.getCurrentPos();
            auto fk = robot.forwardKinematics(current_angles);

            std::cout << "\n" << std::string(60, '=');
            std::cout << "\n机械臂正运动学结果";
            std::cout << "\n" << std::string(60, '=');

            // 显示关节角度
            std::cout << "\n关节角度 (度): [";
            for (size_t i = 0; i < fk.joint_angles.size(); ++i) {
                std::cout << std::setw(7) << std::setprecision(2) << std::fixed
                          << fk.joint_angles[i] * 180.0 / M_PI;
                if (i < fk.joint_angles.size() - 1) std::cout << ", ";
            }
            std::cout << "]";

            // 显示末端位置
            const auto& pos = fk.position;
            std::cout << "\n末端位置 (m): x=" << std::setw(8) << std::setprecision(4) << std::fixed << pos[0]
                      << ", y=" << std::setw(8) << pos[1]
                      << ", z=" << std::setw(8) << pos[2];

            // 显示旋转矩阵
            printMatrix(fk.rotation, "旋转矩阵 (R)");

            // 显示欧拉角
            std::vector<double> euler_angles = Panthera::rotationMatrixToEuler(fk.rotation);
            std::cout << "\n欧拉角 (度): Roll=" << std::setw(7) << std::setprecision(2) << euler_angles[0]
                      << ", Pitch=" << std::setw(7) << euler_angles[1]
                      << ", Yaw=" << std::setw(7) << euler_angles[2];

            // 显示4x4变换矩阵
            printMatrix(fk.transform, "4x4变换矩阵 (T)");

            std::this_thread::sleep_for(1s);
        }

    } catch (const std::exception& e) {
        std::cout << "\n错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
