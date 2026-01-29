#include "panthera/Panthera.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>

namespace panthera
{

// ==================== 构造函数和析构函数 ====================

Panthera::Panthera() : hightorque_robot::robot("../robot_param/Follower.yaml"), motor_count_(0), gripper_id_(0)
    , model_loaded_(false), tool_offset_(0.14), gripper_limit_lower_(0.0), gripper_limit_upper_(0.0)
{
    // 配置文件已在父类构造函数中加载，这里只需初始化其他成员
    initializeMembers();
}

Panthera::Panthera(const std::string& config_path)
    : hightorque_robot::robot(config_path), motor_count_(0), gripper_id_(0)
    , model_loaded_(false), tool_offset_(0.14), gripper_limit_lower_(0.0), gripper_limit_upper_(0.0)
{
    initialize(config_path);
}

Panthera::~Panthera()
{
}

// ==================== 初始化方法 ====================

void Panthera::initialize(const std::string& config_path)
{
    // 加载配置文件
    loadConfig(config_path);

    // 保存配置文件目录
    size_t last_slash = config_path.find_last_of("/\\");
    if (last_slash != std::string::npos) {
        config_dir_ = config_path.substr(0, last_slash);
    } else {
        config_dir_ = ".";
    }

    // 注意：父类构造函数已经在构造函数初始化列表中调用，会自动调用 init_robot

    // 获取电机数量（不包含夹爪）
    motor_count_ = Motors.size() - 1;
    gripper_id_ = Motors.size();

    std::cout << "初始化机械臂..." << std::endl;
    std::cout << "发现 " << motor_count_ << " 个电机" << std::endl;

    if (motor_count_ == 0) {
        std::cerr << "未发现电机。请检查您的配置和连接。" << std::endl;
        return;
    }

    // 打印电机信息
    for (size_t i = 0; i < Motors.size(); ++i) {
        std::cout << "Motor " << i << ": "
                  << "ID=" << Motors[i]->get_motor_id() << ", "
                  << "Type=" << static_cast<int>(Motors[i]->get_motor_enum_type()) << ", "
                  << "Name=" << Motors[i]->get_motor_name() << std::endl;
    }

    // 尝试加载URDF模型
    loadURDFModel();
}

void Panthera::loadConfig(const std::string& config_path)
{
    try {
        config_ = YAML::LoadFile(config_path);
        std::cout << "配置文件加载成功: " << config_path << std::endl;

        // 读取关节限位
        if (config_["robot"] && config_["robot"]["joint_limits"]) {
            auto limits = config_["robot"]["joint_limits"];
            joint_limits_lower_ = limits["lower"].as<std::vector<double>>();
            joint_limits_upper_ = limits["upper"].as<std::vector<double>>();

            std::cout << "关节限位加载成功: lower=[";
            for (size_t i = 0; i < joint_limits_lower_.size(); ++i) {
                std::cout << joint_limits_lower_[i];
                if (i < joint_limits_lower_.size() - 1) std::cout << ", ";
            }
            std::cout << "], upper=[";
            for (size_t i = 0; i < joint_limits_upper_.size(); ++i) {
                std::cout << joint_limits_upper_[i];
                if (i < joint_limits_upper_.size() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        } else {
            std::cerr << "警告: 配置文件中未找到joint_limits" << std::endl;
        }

        // 读取夹爪限位
        if (config_["robot"] && config_["robot"]["gripper_limits"]) {
            auto gripper_limits = config_["robot"]["gripper_limits"];
            gripper_limit_lower_ = gripper_limits["lower"].as<double>();
            gripper_limit_upper_ = gripper_limits["upper"].as<double>();

            std::cout << "夹爪限位加载成功: lower=" << gripper_limit_lower_
                      << ", upper=" << gripper_limit_upper_ << std::endl;
        } else {
            std::cerr << "警告: 配置文件中未找到gripper_limits" << std::endl;
        }

        // 读取关节名称
        if (config_["kinematics"] && config_["kinematics"]["joint_names"]) {
            joint_names_ = config_["kinematics"]["joint_names"].as<std::vector<std::string>>();
        }

        // 读取最大力矩
        if (config_["robot"] && config_["robot"]["max_torque"]) {
            max_torque_ = config_["robot"]["max_torque"].as<std::vector<double>>();
            std::cout << "最大力矩加载成功: [";
            for (size_t i = 0; i < max_torque_.size(); ++i) {
                std::cout << max_torque_[i];
                if (i < max_torque_.size() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        } else {
            std::cerr << "警告: 配置文件中未找到max_torque，使用默认值[10.0] * " << motor_count_ << std::endl;
            max_torque_.assign(motor_count_, 10.0);
        }

        // 读取速度限幅
        if (config_["robot"] && config_["robot"]["velocity_limits"]) {
            velocity_limits_ = config_["robot"]["velocity_limits"].as<std::vector<double>>();
            std::cout << "速度限幅加载成功: [";
            for (size_t i = 0; i < velocity_limits_.size(); ++i) {
                std::cout << velocity_limits_[i];
                if (i < velocity_limits_.size() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        } else {
            std::cerr << "警告: 配置文件中未找到velocity_limits，使用默认值[1.0] * " << motor_count_ << std::endl;
            velocity_limits_.assign(motor_count_, 1.0);
        }

    } catch (const YAML::Exception& e) {
        std::cerr << "配置文件加载失败: " << e.what() << std::endl;
        throw;
    }
}

void Panthera::initializeMembers()
{
    // 使用默认配置文件路径
    std::string default_config_path = "../robot_param/Follower.yaml";

    // 加载配置文件
    loadConfig(default_config_path);

    // 保存配置文件目录
    size_t last_slash = default_config_path.find_last_of("/\\");
    if (last_slash != std::string::npos) {
        config_dir_ = default_config_path.substr(0, last_slash);
    } else {
        config_dir_ = ".";
    }

    // 获取电机数量（不包含夹爪）
    motor_count_ = Motors.size() - 1;
    gripper_id_ = Motors.size();

    std::cout << "初始化机械臂..." << std::endl;
    std::cout << "发现 " << motor_count_ << " 个电机" << std::endl;

    if (motor_count_ == 0) {
        std::cerr << "未发现电机。请检查您的配置和连接。" << std::endl;
        return;
    }

    // 打印电机信息
    for (size_t i = 0; i < Motors.size(); ++i) {
        std::cout << "Motor " << i << ": "
                  << "ID=" << Motors[i]->get_motor_id() << ", "
                  << "Type=" << static_cast<int>(Motors[i]->get_motor_enum_type()) << ", "
                  << "Name=" << Motors[i]->get_motor_name() << std::endl;
    }

    // 尝试加载URDF模型
    loadURDFModel();
}

bool Panthera::checkJointLimits(const std::vector<double>& pos)
{
    if (joint_limits_lower_.empty() || joint_limits_upper_.empty()) {
        return true; // 如果没有配置限位，直接通过
    }

    if (pos.size() != joint_limits_lower_.size()) {
        std::cerr << "错误: 位置数组大小与关节数不符" << std::endl;
        return false;
    }

    bool all_in_range = true;
    std::vector<int> out_indices;

    for (size_t i = 0; i < pos.size(); ++i) {
        if (pos[i] < joint_limits_lower_[i] || pos[i] > joint_limits_upper_[i]) {
            all_in_range = false;
            out_indices.push_back(i);
        }
    }

    if (!all_in_range) {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "警告：检测到目标位置超出关节限位范围！" << std::endl;
        std::cout << "目标位置: [";
        for (size_t i = 0; i < pos.size(); ++i) {
            std::cout << pos[i];
            if (i < pos.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        std::cout << "限位下限: [";
        for (size_t i = 0; i < joint_limits_lower_.size(); ++i) {
            std::cout << joint_limits_lower_[i];
            if (i < joint_limits_lower_.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        std::cout << "限位上限: [";
        for (size_t i = 0; i < joint_limits_upper_.size(); ++i) {
            std::cout << joint_limits_upper_[i];
            if (i < joint_limits_upper_.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        for (int idx : out_indices) {
            std::cout << "  关节" << (idx + 1) << ": " << pos[idx]
                      << " 不在 [" << joint_limits_lower_[idx]
                      << ", " << joint_limits_upper_[idx] << "] 范围内" << std::endl;
        }
        std::cout << "控制指令已被拒绝，保护机械臂安全" << std::endl;
        std::cout << std::string(60, '=') << "\n" << std::endl;
        return false;
    }

    return true;
}

bool Panthera::checkGripperLimits(double pos)
{
    // 如果夹爪限位未设置（都为0），直接通过
    if (gripper_limit_lower_ == 0.0 && gripper_limit_upper_ == 0.0) {
        return true;
    }

    // 检查夹爪位置是否在限位范围内
    if (pos < gripper_limit_lower_ || pos > gripper_limit_upper_) {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "警告：检测到夹爪目标位置超出限位范围！" << std::endl;
        std::cout << "目标位置: " << pos << std::endl;
        std::cout << "限位下限: " << gripper_limit_lower_ << std::endl;
        std::cout << "限位上限: " << gripper_limit_upper_ << std::endl;
        std::cout << "夹爪位置 " << pos << " 不在 ["
                  << gripper_limit_lower_ << ", " << gripper_limit_upper_ << "] 范围内" << std::endl;
        std::cout << "控制指令已被拒绝，保护夹爪安全" << std::endl;
        std::cout << std::string(60, '=') << "\n" << std::endl;
        return false;
    }

    return true;
}

// ==================== 状态获取接口 ====================

std::vector<motor_back_t*> Panthera::getCurrentState()
{
    std::vector<motor_back_t*> states(motor_count_);
    for (int i = 0; i < motor_count_; ++i) {
        states[i] = Motors[i]->get_current_motor_state();
    }
    return states;
}

std::vector<double> Panthera::getCurrentPos()
{
    std::vector<double> positions(motor_count_);
    for (int i = 0; i < motor_count_; ++i) {
        auto state = Motors[i]->get_current_motor_state();
        positions[i] = state->position;
    }
    return positions;
}

std::vector<double> Panthera::getCurrentVel()
{
    std::vector<double> velocities(motor_count_);
    for (int i = 0; i < motor_count_; ++i) {
        auto state = Motors[i]->get_current_motor_state();
        velocities[i] = state->velocity;
    }
    return velocities;
}

std::vector<double> Panthera::getCurrentTorque()
{
    std::vector<double> torques(motor_count_);
    for (int i = 0; i < motor_count_; ++i) {
        auto state = Motors[i]->get_current_motor_state();
        torques[i] = state->torque;
    }
    return torques;
}

motor_back_t* Panthera::getCurrentStateGripper()
{
    return Motors[gripper_id_ - 1]->get_current_motor_state();
}

double Panthera::getCurrentPosGripper()
{
    auto state = Motors[gripper_id_ - 1]->get_current_motor_state();
    return state->position;
}

double Panthera::getCurrentVelGripper()
{
    auto state = Motors[gripper_id_ - 1]->get_current_motor_state();
    return state->velocity;
}

double Panthera::getCurrentTorqueGripper()
{
    auto state = Motors[gripper_id_ - 1]->get_current_motor_state();
    return state->torque;
}

// ==================== 控制接口 ====================

bool Panthera::posVelMaxTorque(const std::vector<double>& pos,
                                const std::vector<double>& vel,
                                const std::vector<double>& max_torque,
                                bool is_wait,
                                double tolerance,
                                double timeout)
{
    // 确定使用的最大力矩
    std::vector<double> torque_to_use;
    if (max_torque.empty()) {
        torque_to_use = max_torque_;
    } else {
        if (max_torque.size() != motor_count_) {
            std::cerr << "错误: 关节参数长度必须为 " << motor_count_ << std::endl;
            return false;
        }
        torque_to_use = max_torque;
    }

    // 检查参数长度
    if (pos.size() != motor_count_ || vel.size() != motor_count_) {
        std::cerr << "错误: 关节参数长度必须为 " << motor_count_ << std::endl;
        return false;
    }

    // 检查关节限位
    if (!checkJointLimits(pos)) {
        return false;
    }

    // 控制关节（除了夹爪电机）
    for (int i = 0; i < motor_count_; ++i) {
        Motors[i]->pos_vel_MAXtqe(pos[i], vel[i], torque_to_use[i]);
    }
    motor_send_cmd();

    if (is_wait) {
        return waitForPosition(pos, tolerance, timeout);
    }

    return true;
}

bool Panthera::jointVel(const std::vector<double>& vel)
{
    // 参数检查
    if (vel.size() != motor_count_) {
        std::cerr << "错误: 目标速度长度必须为 " << motor_count_ << std::endl;
        return false;
    }

    // 复制速度向量以便可能的限幅处理
    std::vector<double> clipped_vel = vel;

    // 速度限幅检查
    if (!velocity_limits_.empty()) {
        bool out_of_limit = false;
        std::vector<int> out_indices;

        for (size_t i = 0; i < clipped_vel.size(); ++i) {
            double abs_vel = std::abs(clipped_vel[i]);
            if (abs_vel > velocity_limits_[i]) {
                out_of_limit = true;
                out_indices.push_back(i);
                // 限幅处理
                clipped_vel[i] = std::max(-velocity_limits_[i], std::min(velocity_limits_[i], clipped_vel[i]));
            }
        }

        if (out_of_limit) {
            std::cout << "\n" << std::string(60, '=') << std::endl;
            std::cout << "警告：检测到目标速度超出限幅范围！" << std::endl;
            std::cout << "目标速度: [";
            for (size_t i = 0; i < vel.size(); ++i) {
                std::cout << vel[i];
                if (i < vel.size() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
            std::cout << "速度限幅: ±[";
            for (size_t i = 0; i < velocity_limits_.size(); ++i) {
                std::cout << velocity_limits_[i];
                if (i < velocity_limits_.size() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
            for (int idx : out_indices) {
                std::cout << "  关节" << (idx + 1) << ": " << vel[idx]
                          << " rad/s 超出限幅 ±" << velocity_limits_[idx] << " rad/s" << std::endl;
            }
            std::cout << "速度将被限制在安全范围内" << std::endl;
            std::cout << std::string(60, '=') << "\n" << std::endl;
        }
    }

    // 控制关节（除了夹爪电机）
    for (int i = 0; i < motor_count_; ++i) {
        Motors[i]->velocity(clipped_vel[i]);
    }
    motor_send_cmd();
    return true;
}

bool Panthera::jointsSyncArrival(const std::vector<double>& pos,
                                 double duration,
                                 const std::vector<double>& max_torque,
                                 bool is_wait,
                                 double tolerance,
                                 double timeout)
{
    // 参数检查
    if (pos.size() != motor_count_) {
        std::cerr << "错误: 目标位置长度必须为 " << motor_count_ << std::endl;
        return false;
    }
    if (duration <= 0) {
        std::cerr << "错误: 运动时间必须大于0，当前值: " << duration << std::endl;
        return false;
    }

    // 确定使用的最大力矩
    std::vector<double> torque_to_use;
    if (max_torque.empty()) {
        torque_to_use = max_torque_;
    } else {
        if (max_torque.size() != motor_count_) {
            std::cerr << "错误: 最大力矩长度必须为 " << motor_count_ << std::endl;
            return false;
        }
        torque_to_use = max_torque;
    }

    // 检查位置是否在限位范围内
    if (!checkJointLimits(pos)) {
        return false;
    }

    // 获取当前位置
    std::vector<double> current_pos = getCurrentPos();

    // 计算速度: v = (目标位置 - 当前位置) / 时间
    // 这样可以确保所有关节在duration时间内同时到达目标位置
    std::vector<double> vel(motor_count_);
    for (int i = 0; i < motor_count_; ++i) {
        vel[i] = (pos[i] - current_pos[i]) / duration;
    }

    // 调用单关节位置速度控制
    return posVelMaxTorque(pos, vel, torque_to_use, is_wait, tolerance, timeout);
}

bool Panthera::posVelTorqueKpKd(const std::vector<double>& pos,
                                 const std::vector<double>& vel,
                                 const std::vector<double>& torque,
                                 const std::vector<double>& kp,
                                 const std::vector<double>& kd)
{
    // 检查参数长度
    if (pos.size() != motor_count_ || vel.size() != motor_count_ ||
        torque.size() != motor_count_ || kp.size() != motor_count_ ||
        kd.size() != motor_count_) {
        std::cerr << "错误: 关节参数长度必须为 " << motor_count_ << std::endl;
        return false;
    }

    // 检查关节限位
    if (!checkJointLimits(pos)) {
        return false;
    }

    // 控制关节（除了夹爪电机）
    for (int i = 0; i < motor_count_; ++i) {
        Motors[i]->pos_vel_tqe_kp_kd(pos[i], vel[i], torque[i], kp[i], kd[i]);
    }
    motor_send_cmd();

    return true;
}

// ==================== 夹爪控制接口 ====================

bool Panthera::gripperControl(double pos, double vel, double max_torque)
{
    // 检查夹爪限位
    if (!checkGripperLimits(pos)) {
        return false;
    }

    Motors[gripper_id_ - 1]->pos_vel_MAXtqe(pos, vel, max_torque);
    motor_send_cmd();
    return true;
}

bool Panthera::gripperControlMIT(double pos, double vel, double torque,
                                  double kp, double kd)
{
    // 检查夹爪限位
    if (!checkGripperLimits(pos)) {
        return false;
    }

    Motors[gripper_id_ - 1]->pos_vel_tqe_kp_kd(pos, vel, torque, kp, kd);
    motor_send_cmd();
    return true;
}

void Panthera::gripperOpen(double pos, double vel, double max_torque)
{
    gripperControl(pos, vel, max_torque);
}

void Panthera::gripperClose(double pos, double vel, double max_torque)
{
    gripperControl(pos, vel, max_torque);
}

// ==================== 位置检测接口 ====================

bool Panthera::checkPositionReached(const std::vector<double>& target_positions,
                                     double tolerance,
                                     std::vector<double>& position_errors)
{
    bool all_reached = true;
    position_errors.clear();
    position_errors.resize(motor_count_);

    send_get_motor_state_cmd();
    motor_send_cmd();

    // 检查前N个关节（不包含夹爪）
    for (int i = 0; i < motor_count_; ++i) {
        auto state = Motors[i]->get_current_motor_state();
        double error = std::abs(state->position - target_positions[i]);
        position_errors[i] = error;
        if (error > tolerance) {
            all_reached = false;
        }
    }

    return all_reached;
}

bool Panthera::waitForPosition(const std::vector<double>& target_positions,
                                double tolerance,
                                double timeout)
{
    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - start_time).count() / 1000.0;

        if (elapsed >= timeout) {
            return false;
        }

        std::vector<double> errors;
        if (checkPositionReached(target_positions, tolerance, errors)) {
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

// ==================== 摩擦补偿 ====================

std::vector<double> Panthera::getFrictionCompensation(
    const std::vector<double>& vel,
    const std::vector<double>& Fc,
    const std::vector<double>& Fv,
    double vel_threshold)
{
    std::vector<double> current_vel;

    // 如果未提供速度，使用当前速度
    if (vel.empty()) {
        current_vel = getCurrentVel();
    } else {
        current_vel = vel;
    }

    std::vector<double> tau_friction(motor_count_);

    // 向量化计算摩擦力补偿
    for (int i = 0; i < motor_count_; ++i) {
        double v = current_vel[i];
        double fc = Fc[i];
        double fv = Fv[i];

        // 计算完整的摩擦模型（库伦 + 粘性）
        double full_friction = fc * std::copysign(1.0, v) + fv * v;

        // 低速区只使用粘性摩擦
        double low_speed_friction = fv * v;

        // 使用条件选择：|vel| < threshold 时用低速模型，否则用完整模型
        if (std::abs(v) < vel_threshold) {
            tau_friction[i] = low_speed_friction;
        } else {
            tau_friction[i] = full_friction;
        }
    }

    return tau_friction;
}

// ==================== 多项式轨迹插值 ====================

void Panthera::quinticInterpolation(
    const std::vector<double>& start_pos,
    const std::vector<double>& end_pos,
    double duration,
    double current_time,
    std::vector<double>& out_pos,
    std::vector<double>& out_vel,
    std::vector<double>& out_acc)
{
    size_t n = start_pos.size();
    out_pos.resize(n);
    out_vel.resize(n);
    out_acc.resize(n);

    if (current_time <= 0) {
        out_pos = start_pos;
        std::fill(out_vel.begin(), out_vel.end(), 0.0);
        std::fill(out_acc.begin(), out_acc.end(), 0.0);
        return;
    }

    if (current_time >= duration) {
        out_pos = end_pos;
        std::fill(out_vel.begin(), out_vel.end(), 0.0);
        std::fill(out_acc.begin(), out_acc.end(), 0.0);
        return;
    }

    // 归一化时间
    double t = current_time / duration;
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;

    // 五次多项式系数 (位置)
    double a0 = 1.0 - 10.0 * t3 + 15.0 * t4 - 6.0 * t5;
    double a1 = 10.0 * t3 - 15.0 * t4 + 6.0 * t5;

    // 一阶导数系数 (速度)
    double da0 = -30.0 * t2 + 60.0 * t3 - 30.0 * t4;
    double da1 = 30.0 * t2 - 60.0 * t3 + 30.0 * t4;

    // 二阶导数系数 (加速度)
    double dda0 = -60.0 * t + 180.0 * t2 - 120.0 * t3;
    double dda1 = 60.0 * t - 180.0 * t2 + 120.0 * t3;

    // 向量化计算位置、速度、加速度
    for (size_t i = 0; i < n; ++i) {
        out_pos[i] = a0 * start_pos[i] + a1 * end_pos[i];
        out_vel[i] = (da0 * start_pos[i] + da1 * end_pos[i]) / duration;
        out_acc[i] = (dda0 * start_pos[i] + dda1 * end_pos[i]) / (duration * duration);
    }
}

void Panthera::septicInterpolation(
    const std::vector<double>& start_pos,
    const std::vector<double>& end_pos,
    double duration,
    double current_time,
    std::vector<double>& out_pos,
    std::vector<double>& out_vel,
    std::vector<double>& out_acc)
{
    size_t n = start_pos.size();
    out_pos.resize(n);
    out_vel.resize(n);
    out_acc.resize(n);

    if (current_time <= 0) {
        out_pos = start_pos;
        std::fill(out_vel.begin(), out_vel.end(), 0.0);
        std::fill(out_acc.begin(), out_acc.end(), 0.0);
        return;
    }

    if (current_time >= duration) {
        out_pos = end_pos;
        std::fill(out_vel.begin(), out_vel.end(), 0.0);
        std::fill(out_acc.begin(), out_acc.end(), 0.0);
        return;
    }

    // 归一化时间
    double t = current_time / duration;
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;
    double t6 = t5 * t;
    double t7 = t6 * t;

    // 七次多项式系数 (位置)
    double a0 = 1.0 - 35.0 * t4 + 84.0 * t5 - 70.0 * t6 + 20.0 * t7;
    double a1 = 35.0 * t4 - 84.0 * t5 + 70.0 * t6 - 20.0 * t7;

    // 一阶导数系数 (速度)
    double da0 = -140.0 * t3 + 420.0 * t4 - 420.0 * t5 + 140.0 * t6;
    double da1 = 140.0 * t3 - 420.0 * t4 + 420.0 * t5 - 140.0 * t6;

    // 二阶导数系数 (加速度)
    double dda0 = -420.0 * t2 + 1680.0 * t3 - 2100.0 * t4 + 840.0 * t5;
    double dda1 = 420.0 * t2 - 1680.0 * t3 + 2100.0 * t4 - 840.0 * t5;

    // 向量化计算位置、速度、加速度
    for (size_t i = 0; i < n; ++i) {
        out_pos[i] = a0 * start_pos[i] + a1 * end_pos[i];
        out_vel[i] = (da0 * start_pos[i] + da1 * end_pos[i]) / duration;
        out_acc[i] = (dda0 * start_pos[i] + dda1 * end_pos[i]) / (duration * duration);
    }
}

void Panthera::septicInterpolationWithVelocity(
    const std::vector<double>& start_pos,
    const std::vector<double>& end_pos,
    const std::vector<double>& start_vel,
    const std::vector<double>& end_vel,
    double duration,
    double current_time,
    std::vector<double>& out_pos,
    std::vector<double>& out_vel,
    std::vector<double>& out_acc)
{
    size_t n = start_pos.size();
    out_pos.resize(n);
    out_vel.resize(n);
    out_acc.resize(n);

    if (current_time <= 0) {
        out_pos = start_pos;
        out_vel = start_vel;
        std::fill(out_acc.begin(), out_acc.end(), 0.0);
        return;
    }

    if (current_time >= duration) {
        out_pos = end_pos;
        out_vel = end_vel;
        std::fill(out_acc.begin(), out_acc.end(), 0.0);
        return;
    }

    // 归一化时间
    double t = current_time / duration;
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;
    double t6 = t5 * t;
    double t7 = t6 * t;

    // 向量化计算位置、速度、加速度
    for (size_t i = 0; i < n; ++i) {
        double p0 = start_pos[i];
        double p1 = end_pos[i];
        double v0 = start_vel[i] * duration;  // 转换为归一化速度
        double v1 = end_vel[i] * duration;

        // 通过矩阵求解得到的系数
        double a0 = p0;
        double a1 = v0;
        double a2 = 0.0;  // 起始加速度为0
        double a3 = 0.0;  // 起始加加速度为0
        double a4 = 35.0 * (p1 - p0) - 20.0 * v0 - 15.0 * v1;
        double a5 = -84.0 * (p1 - p0) + 45.0 * v0 + 39.0 * v1;
        double a6 = 70.0 * (p1 - p0) - 36.0 * v0 - 34.0 * v1;
        double a7 = -20.0 * (p1 - p0) + 10.0 * v0 + 10.0 * v1;

        // 计算位置
        out_pos[i] = a0 + a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5 + a6 * t6 + a7 * t7;

        // 计算速度（一阶导数）
        out_vel[i] = (a1 + 2.0 * a2 * t + 3.0 * a3 * t2 + 4.0 * a4 * t3 +
                      5.0 * a5 * t4 + 6.0 * a6 * t5 + 7.0 * a7 * t6) / duration;

        // 计算加速度（二阶导数）
        out_acc[i] = (2.0 * a2 + 6.0 * a3 * t + 12.0 * a4 * t2 +
                      20.0 * a5 * t3 + 30.0 * a6 * t4 + 42.0 * a7 * t5) / (duration * duration);
    }
}

// ==================== 工具方法 ====================

void Panthera::getJointLimits(std::vector<double>& lower, std::vector<double>& upper) const
{
    lower = joint_limits_lower_;
    upper = joint_limits_upper_;
}

// ==================== 运动学和动力学实现 ====================

void Panthera::loadURDFModel()
{
    try {
        // 从配置文件获取URDF路径
        if (!config_["urdf"] || !config_["urdf"]["file_path"]) {
            std::cerr << "警告: 配置文件中未找到URDF路径" << std::endl;
            model_loaded_ = false;
            return;
        }

        std::string urdf_relative = config_["urdf"]["file_path"].as<std::string>();
        std::string urdf_path = config_dir_ + "/" + urdf_relative;

        // 使用Pinocchio加载URDF
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);

        // 获取关节信息
        if (config_["kinematics"] && config_["kinematics"]["joint_names"]) {
            joint_names_ = config_["kinematics"]["joint_names"].as<std::vector<std::string>>();

            // 获取关节ID（跳过universe joint）
            joint_ids_.clear();
            for (const auto& name : joint_names_) {
                if (model_.existJointName(name)) {
                    int joint_id = model_.getJointId(name);
                    joint_ids_.push_back(joint_id);
                } else {
                    std::cerr << "警告: 关节 " << name << " 未在模型中找到" << std::endl;
                }
            }
        }

        model_loaded_ = true;
        std::cout << "URDF加载成功: " << urdf_path << std::endl;
        std::cout << "模型包含 " << model_.njoints - 1 << " 个关节（不含base）" << std::endl;
        std::cout << "配置关节数: " << joint_ids_.size() << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "URDF加载失败: " << e.what() << std::endl;
        model_loaded_ = false;
    }
}

Eigen::VectorXd Panthera::jointAnglesToPinocchioQ(const std::vector<double>& joint_angles)
{
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);

    for (size_t i = 0; i < joint_ids_.size() && i < joint_angles.size(); ++i) {
        int joint_id = joint_ids_[i];
        int idx = model_.joints[joint_id].idx_q();
        q[idx] = joint_angles[i];
    }

    return q;
}

std::vector<double> Panthera::pinocchioQToJointAngles(const Eigen::VectorXd& q)
{
    std::vector<double> joint_angles(joint_ids_.size());

    for (size_t i = 0; i < joint_ids_.size(); ++i) {
        int joint_id = joint_ids_[i];
        int idx = model_.joints[joint_id].idx_q();
        joint_angles[i] = q[idx];
    }

    return joint_angles;
}

Panthera::ForwardKinematicsResult Panthera::forwardKinematics(const std::vector<double>& joint_angles)
{
    if (!model_loaded_) {
        std::cerr << "错误: URDF模型未加载" << std::endl;
        return ForwardKinematicsResult();
    }

    // 如果未提供关节角度，获取当前角度
    std::vector<double> q;
    if (joint_angles.empty()) {
        q = getCurrentPos();
    } else {
        q = joint_angles;
    }

    // 转换为Pinocchio配置向量
    Eigen::VectorXd q_pinocchio = jointAnglesToPinocchioQ(q);

    // 计算正运动学
    pinocchio::forwardKinematics(model_, data_, q_pinocchio);
    pinocchio::updateFramePlacements(model_, data_);

    // 获取最后一个活动关节的变换矩阵
    int last_joint_id = joint_ids_.back();
    pinocchio::SE3 last_joint_transform = data_.oMi[last_joint_id];

    // 添加工具坐标系偏移：相对于最后一个关节在X轴方向偏移
    Eigen::Vector3d tool_offset(tool_offset_, 0.0, 0.0);

    // 计算工具坐标系的位置和旋转
    Eigen::Vector3d position = last_joint_transform.translation() +
                             last_joint_transform.rotation() * tool_offset;
    Eigen::Matrix3d rotation = last_joint_transform.rotation();

    // 构建4x4变换矩阵
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = rotation;
    transform.block<3, 1>(0, 3) = position;

    ForwardKinematicsResult result;
    result.position = position;
    result.rotation = rotation;
    result.transform = transform;
    result.joint_angles = q;

    return result;
}

std::vector<double> Panthera::inverseKinematics(
    const Eigen::Vector3d& target_position,
    const Eigen::Matrix3d* target_rotation,
    const std::vector<double>& init_q,
    int max_iter,
    double eps)
{
    if (!model_loaded_) {
        std::cerr << "错误: URDF模型未加载" << std::endl;
        return std::vector<double>();
    }

    // 目标姿态（默认单位矩阵）
    Eigen::Matrix3d target_rot;
    if (target_rotation != nullptr) {
        target_rot = *target_rotation;
    } else {
        target_rot = Eigen::Matrix3d::Identity();
    }

    // 计算最后关节的目标位置（减去工具偏移）
    Eigen::Vector3d tool_offset(tool_offset_, 0.0, 0.0);
    Eigen::Vector3d last_joint_target_pos = target_position - target_rot * tool_offset;

    pinocchio::SE3 oMdes(target_rot, last_joint_target_pos);

    // 初始关节角度
    std::vector<double> current_q;
    if (init_q.empty()) {
        current_q = getCurrentPos();
    } else {
        current_q = init_q;
    }

    // 转换为Pinocchio配置向量
    Eigen::VectorXd q = jointAnglesToPinocchioQ(current_q);

    // 获取最后一个活动关节ID
    int joint_id = joint_ids_.back();

    // 迭代求解
    double dt = 0.1;
    double damp = 1e-12;

    for (int iter = 0; iter < max_iter; ++iter) {
        // 计算误差
        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::SE3 iMd = data_.oMi[joint_id].actInv(oMdes);
        Eigen::Matrix<double, 6, 1> err = pinocchio::log6(iMd);

        if (err.norm() < eps) {
            // 提取关节角度
            return pinocchioQToJointAngles(q);
        }

        // 计算雅可比矩阵
        Eigen::MatrixXd J(6, model_.nv);
        J.setZero();
        pinocchio::computeJointJacobian(model_, data_, q, joint_id, J);

        Eigen::Matrix<double, 6, 6> Jlog = pinocchio::Jlog6(iMd.inverse());
        J = -Jlog * J;

        // 阻尼最小二乘法
        Eigen::MatrixXd JJT = J * J.transpose();
        Eigen::MatrixXd damped = JJT + damp * Eigen::Matrix<double, 6, 6>::Identity();
        Eigen::Matrix<double, 6, 1> v = -J.transpose() * damped.ldlt().solve(err);

        // 更新关节角度（直接积分，简化版本）
        // 只更新前6个关节（主要运动关节）
        for (int i = 0; i < std::min(6, model_.nv); ++i) {
            q[i] += v[i] * dt;
        }
    }

    std::cerr << "IK求解失败：未在 " << max_iter << " 次迭代内收敛" << std::endl;
    return std::vector<double>();
}

std::vector<double> Panthera::getGravity(const std::vector<double>& q)
{
    if (!model_loaded_) {
        std::cerr << "错误: URDF模型未加载" << std::endl;
        return std::vector<double>();
    }

    // 使用当前角度（如果未提供）
    std::vector<double> q_use;
    if (q.empty()) {
        q_use = getCurrentPos();
    } else {
        q_use = q;
    }

    // 转换为Pinocchio配置向量
    Eigen::VectorXd q_pinocchio = jointAnglesToPinocchioQ(q_use);

    // 计算重力补偿
    Eigen::VectorXd G = pinocchio::computeGeneralizedGravity(model_, data_, q_pinocchio);

    // 转换回std::vector
    std::vector<double> gravity_torque(joint_ids_.size());
    for (size_t i = 0; i < joint_ids_.size(); ++i) {
        gravity_torque[i] = G[i];
    }

    return gravity_torque;
}

Eigen::MatrixXd Panthera::getCoriolis(const std::vector<double>& q, const std::vector<double>& v)
{
    if (!model_loaded_) {
        std::cerr << "错误: URDF模型未加载" << std::endl;
        return Eigen::MatrixXd();
    }

    // 使用当前角度和速度（如果未提供）
    std::vector<double> q_use, v_use;
    if (q.empty()) {
        q_use = getCurrentPos();
    } else {
        q_use = q;
    }

    if (v.empty()) {
        v_use = getCurrentVel();
    } else {
        v_use = v;
    }

    // 转换为Pinocchio配置向量
    Eigen::VectorXd q_pinocchio = jointAnglesToPinocchioQ(q_use);
    Eigen::VectorXd v_pinocchio = jointAnglesToPinocchioQ(v_use);

    // 计算科氏力矩阵
    Eigen::MatrixXd C = pinocchio::computeCoriolisMatrix(model_, data_, q_pinocchio, v_pinocchio);

    // 只返回实际使用的关节数量
    return C.block(0, 0, joint_ids_.size(), joint_ids_.size());
}

std::vector<double> Panthera::getCoriolisVector(const std::vector<double>& q, const std::vector<double>& v)
{
    Eigen::MatrixXd C = getCoriolis(q, v);
    std::vector<double> v_use = v.empty() ? getCurrentVel() : v;

    // 计算 C * v
    Eigen::VectorXd v_eigen(v_use.size());
    for (size_t i = 0; i < v_use.size(); ++i) {
        v_eigen[i] = v_use[i];
    }

    Eigen::VectorXd coriolis_vector = C * v_eigen;

    // 转换回std::vector
    std::vector<double> result(joint_ids_.size());
    for (size_t i = 0; i < joint_ids_.size(); ++i) {
        result[i] = coriolis_vector[i];
    }

    return result;
}

Eigen::MatrixXd Panthera::getMassMatrix(const std::vector<double>& q)
{
    if (!model_loaded_) {
        std::cerr << "错误: URDF模型未加载" << std::endl;
        return Eigen::MatrixXd();
    }

    // 使用当前角度（如果未提供）
    std::vector<double> q_use;
    if (q.empty()) {
        q_use = getCurrentPos();
    } else {
        q_use = q;
    }

    // 转换为Pinocchio配置向量
    Eigen::VectorXd q_pinocchio = jointAnglesToPinocchioQ(q_use);

    // 计算质量矩阵（CRBA算法）
    Eigen::MatrixXd M = pinocchio::crba(model_, data_, q_pinocchio);

    // 只返回实际使用的关节数量
    return M.block(0, 0, joint_ids_.size(), joint_ids_.size());
}

std::vector<double> Panthera::getInertiaTerms(const std::vector<double>& q, const std::vector<double>& a)
{
    if (!model_loaded_) {
        std::cerr << "错误: URDF模型未加载" << std::endl;
        return std::vector<double>();
    }

    // 使用当前角度和加速度（如果未提供）
    std::vector<double> q_use, a_use;
    if (q.empty()) {
        q_use = getCurrentPos();
    } else {
        q_use = q;
    }

    if (a.empty()) {
        a_use = std::vector<double>(motor_count_, 0.0);
    } else {
        a_use = a;
    }

    // 计算质量矩阵
    Eigen::MatrixXd M = getMassMatrix(q_use);

    // 转换加速度为Eigen向量
    Eigen::VectorXd a_eigen(a_use.size());
    for (size_t i = 0; i < a_use.size(); ++i) {
        a_eigen[i] = a_use[i];
    }

    // 计算惯性力矩 M*a
    Eigen::VectorXd inertia_torque = M * a_eigen;

    // 转换回std::vector
    std::vector<double> result(joint_ids_.size());
    for (size_t i = 0; i < joint_ids_.size(); ++i) {
        result[i] = inertia_torque[i];
    }

    return result;
}

std::vector<double> Panthera::getDynamics(const std::vector<double>& q, const std::vector<double>& v, const std::vector<double>& a)
{
    if (!model_loaded_) {
        std::cerr << "错误: URDF模型未加载" << std::endl;
        return std::vector<double>();
    }

    // 使用当前角度、速度和加速度（如果未提供）
    std::vector<double> q_use, v_use, a_use;
    if (q.empty()) {
        q_use = getCurrentPos();
    } else {
        q_use = q;
    }

    if (v.empty()) {
        v_use = getCurrentVel();
    } else {
        v_use = v;
    }

    if (a.empty()) {
        a_use = std::vector<double>(motor_count_, 0.0);
    } else {
        a_use = a;
    }

    // 转换为Pinocchio配置向量
    Eigen::VectorXd q_pinocchio = jointAnglesToPinocchioQ(q_use);
    Eigen::VectorXd v_pinocchio = jointAnglesToPinocchioQ(v_use);
    Eigen::VectorXd a_pinocchio = jointAnglesToPinocchioQ(a_use);

    // 使用RNEA算法计算完整动力学
    Eigen::VectorXd tau = pinocchio::rnea(model_, data_, q_pinocchio, v_pinocchio, a_pinocchio);

    // 转换回std::vector
    std::vector<double> result(joint_ids_.size());
    for (size_t i = 0; i < joint_ids_.size(); ++i) {
        result[i] = tau[i];
    }

    return result;
}

std::vector<double> Panthera::rotationMatrixToEuler(const Eigen::Matrix3d& rotation)
{
    std::vector<double> euler(3);

    double sy = std::sqrt(rotation(0, 0) * rotation(0, 0) + rotation(1, 0) * rotation(1, 0));
    bool singular = sy < 1e-6;

    if (!singular) {
        euler[0] = std::atan2(rotation(2, 1), rotation(2, 2));  // Roll
        euler[1] = std::atan2(-rotation(2, 0), sy);           // Pitch
        euler[2] = std::atan2(rotation(1, 0), rotation(0, 0));  // Yaw
    } else {
        euler[0] = std::atan2(-rotation(1, 2), rotation(1, 1));
        euler[1] = std::atan2(-rotation(2, 0), sy);
        euler[2] = 0;
    }

    // 转换为度
    euler[0] = euler[0] * 180.0 / M_PI;
    euler[1] = euler[1] * 180.0 / M_PI;
    euler[2] = euler[2] * 180.0 / M_PI;

    return euler;
}

std::vector<double> Panthera::clipTorque(const std::vector<double>& torque, const std::vector<double>& max_torque)
{
    std::vector<double> clipped(torque.size());
    for (size_t i = 0; i < torque.size(); ++i) {
        clipped[i] = std::max(-max_torque[i], std::min(max_torque[i], torque[i]));
    }
    return clipped;
}

} // namespace panthera
