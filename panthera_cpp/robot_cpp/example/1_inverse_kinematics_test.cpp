#include "panthera/Panthera.hpp"
#include <csignal>
#include <atomic>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <Eigen/Dense>

using namespace panthera;
using namespace std::chrono_literals;

/**
 * @brief 逆运动学验证程序
 * 两种验证方法
 * 1. 当前位置正解求取末端位姿，再使用当前末端位姿进行逆解验证（可拖动查看逆解效果）
 * 2. 指定位置和姿态进行逆解验证
 */
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

    Panthera robot;

    try {
        while(!exitFlag.load()) {
            // 方法1：当前位置正解求取末端位姿，再使用当前末端位姿进行逆解验证
            // 利用运控模式发送控制帧以读取电机反馈状态
            robot.send_get_motor_state_cmd();
            std::vector<double> current_angles = robot.getCurrentPos();

            auto fk = robot.forwardKinematics(current_angles);
            if (fk.position.size() == 0) {
                std::this_thread::sleep_for(500ms);
                continue;
            }

            Eigen::Vector3d ik_pos = fk.position;
            Eigen::Matrix3d ik_rot = fk.rotation;

            // 使用当前关节角减0.1作为初始计算角（向量运算）
            std::vector<double> init_q(current_angles.size());
            for (size_t i = 0; i < current_angles.size(); ++i) {
                init_q[i] = current_angles[i] - 0.1;
            }

            // # 使用零位作为初始计算角
            // std::vector<double> init_q(robot.getMotorCount(), 0.0);

            // 使用当前位置和姿态进行逆解
            std::vector<double> solved_angles = robot.inverseKinematics(ik_pos, &ik_rot, init_q);

            if (!solved_angles.empty()) {
                std::cout << "\n当前关节: [";
                for (size_t i = 0; i < current_angles.size(); ++i) {
                    std::cout << std::setprecision(3) << std::fixed << current_angles[i];
                    if (i < current_angles.size() - 1) std::cout << ", ";
                }
                std::cout << "]";

                std::cout << "\n逆解关节: [";
                for (size_t i = 0; i < solved_angles.size(); ++i) {
                    std::cout << std::setprecision(3) << std::fixed << solved_angles[i];
                    if (i < solved_angles.size() - 1) std::cout << ", ";
                }
                std::cout << "]";

                // 计算误差（向量运算）
                double max_error = 0.0;
                for (size_t i = 0; i < current_angles.size() && i < solved_angles.size(); ++i) {
                    double error = std::abs(current_angles[i] - solved_angles[i]);
                    if (error > max_error) {
                        max_error = error;
                    }
                }
                std::cout << "\n最大误差: " << std::setprecision(4) << std::fixed << max_error << " rad";
            }

            std::this_thread::sleep_for(500ms);

            // # 方法2: 指定位置和姿态进行逆解验证
            // Eigen::Vector3d target_pos(0.3, 0.2, 0.2);
            // Eigen::Matrix3d target_rot = Eigen::Matrix3d::Identity();  // 单位矩阵作为示例
            // // 逆解计算
            // std::vector<double> init_q(robot.getMotorCount(), 0.0);
            // std::vector<double> ik_angles = robot.inverseKinematics(target_pos, &target_rot, init_q);
            // if (!ik_angles.empty()) {
            //     // 用逆解结果进行正解验证
            //     auto fk_verify = robot.forwardKinematics(ik_angles);
            //     if (fk_verify.position.size() > 0) {
            //         // 位置误差
            //         double pos_error = (target_pos - fk_verify.position).norm();
            //         // 姿态误差（旋转矩阵差的Frobenius范数）
            //         double rot_error = (target_rot - fk_verify.rotation).norm();
            //
            //         std::cout << "\n目标位置: [" << target_pos[0] << ", " << target_pos[1] << ", " << target_pos[2] << "]";
            //         std::cout << "\n验证位置: [" << fk_verify.position[0] << ", "
            //                   << fk_verify.position[1] << ", " << fk_verify.position[2] << "]";
            //         std::cout << "\n位置误差: " << pos_error << " m";
            //         std::cout << "\n姿态误差: " << rot_error;
            //     }
            // }
            // std::this_thread::sleep_for(500ms);
        }

    } catch (const std::exception& e) {
        std::cout << "\n错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
