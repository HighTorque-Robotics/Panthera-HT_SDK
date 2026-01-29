#ifndef _PANTHERA_HPP_
#define _PANTHERA_HPP_

#include "hardware/robot.hpp"
#include "serial_struct.hpp"
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <memory>
#include <cmath>

// Pinocchio 头文件
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <Eigen/Dense>

namespace panthera
{

/**
 * @brief Panthera 机械臂控制类
 *
 * 继承自 hightorque_robot::robot，添加高级功能如：
 * - 重力补偿
 * - 摩擦补偿
 * - 多项式轨迹插值
 * - 关节限位检查
 * - 夹爪控制
 */
class Panthera : public hightorque_robot::robot
{
public:
    /**
     * @brief 构造函数，使用默认配置文件
     */
    Panthera();

    /**
     * @brief 构造函数，使用指定的配置文件
     * @param config_path 配置文件路径
     */
    explicit Panthera(const std::string& config_path);

    /**
     * @brief 析构函数
     */
    ~Panthera();

    // ==================== 状态获取接口 ====================

    /**
     * @brief 获取当前关节状态
     * @return 关节状态列表
     */
    std::vector<motor_back_t*> getCurrentState();

    /**
     * @brief 获取当前关节位置
     * @return 关节位置数组 (rad)
     */
    std::vector<double> getCurrentPos();

    /**
     * @brief 获取当前关节速度
     * @return 关节速度��组 (rad/s)
     */
    std::vector<double> getCurrentVel();

    /**
     * @brief 获取当前关节力矩
     * @return 关节力矩数组 (Nm)
     */
    std::vector<double> getCurrentTorque();

    /**
     * @brief 获取夹爪当前状态
     * @return 夹爪状态
     */
    motor_back_t* getCurrentStateGripper();

    /**
     * @brief 获取夹爪当前位置
     * @return 夹爪位置 (rad)
     */
    double getCurrentPosGripper();

    /**
     * @brief 获取夹爪当前速度
     * @return 夹爪速度 (rad/s)
     */
    double getCurrentVelGripper();

    /**
     * @brief 获取夹爪当前力矩
     * @return 夹爪力矩 (Nm)
     */
    double getCurrentTorqueGripper();

    // ==================== 控制接口 ====================

    /**
     * @brief 关节位置、速度、最大力矩控制
     * @param pos 目标位置数组 (rad)
     * @param vel 目标速度数组 (rad/s)
     * @param max_torque 最大力矩数组 (Nm)，如果为空则使用配置文件中的默认值
     * @param is_wait 是否等待到达目标位置
     * @param tolerance 位置容差 (rad)
     * @param timeout 超时时间 (s)
     * @return 是否成功
     */
    bool posVelMaxTorque(const std::vector<double>& pos,
                         const std::vector<double>& vel,
                         const std::vector<double>& max_torque = {},
                         bool is_wait = false,
                         double tolerance = 0.1,
                         double timeout = 15.0);

    /**
     * @brief 关节五参数MIT模式控制（位置、速度、力矩、Kp、Kd）
     * @param pos 目标位置数组 (rad)
     * @param vel 目标速度数组 (rad/s)
     * @param torque 前馈力矩数组 (Nm)
     * @param kp 比例增益数组
     * @param kd 微分增益数组
     * @return 是否成功
     */
    bool posVelTorqueKpKd(const std::vector<double>& pos,
                          const std::vector<double>& vel,
                          const std::vector<double>& torque,
                          const std::vector<double>& kp,
                          const std::vector<double>& kd);

    /**
     * @brief 关节速度控制
     * @param vel 目标速度数组 (rad/s)
     * @return 是否成功
     *
     * 说明:
     *      直接控制关节速度，不进行位置限位检查
     *      适用于需要精确速度控制的场景
     *      速度将被限制在配置文件设定的范围内
     */
    bool jointVel(const std::vector<double>& vel);

    /**
     * @brief 多关节同步到达控制（所有关节在指定时间内同时到达目标位置）
     * @param pos 目标位置列表/数组 [joint1, joint2, ..., jointN]
     * @param duration 运动时间（秒），所有关节将在该时间内同时到达目标位置
     * @param max_torque 最大力矩列表/数组，如果为None则使用配置文件中的默认值
     * @param is_wait 是否等待运动完成
     * @param tolerance 位置容差（弧度）
     * @param timeout 等待超时时间（秒）
     * @return 是否成功
     *
     * 说明:
     *      该函数通过 (目标位置 - 当前位置) / duration 计算每个关节的平均速度，
     *      确保所有关节在指定的时间内同时到达目标位置，适用于需要协调运动的场景
     */
    bool jointsSyncArrival(const std::vector<double>& pos,
                           double duration,
                           const std::vector<double>& max_torque = {},
                           bool is_wait = false,
                           double tolerance = 0.1,
                           double timeout = 15.0);

    // ==================== 夹爪控制接口 ====================

    /**
     * @brief 夹爪控制（位置、速度、最大力矩模式）
     * @param pos 目标位置 (rad)
     * @param vel 目标速度 (rad/s)
     * @param max_torque 最大力矩 (Nm)
     * @return 是否成功
     */
    bool gripperControl(double pos, double vel, double max_torque);

    /**
     * @brief 夹爪MIT模式控制
     * @param pos 目标位置 (rad)
     * @param vel 目标速度 (rad/s)
     * @param torque 前馈力矩 (Nm)
     * @param kp 比例增益
     * @param kd 微分增益
     * @return 是否成功
     */
    bool gripperControlMIT(double pos, double vel, double torque,
                           double kp, double kd);

    /**
     * @brief 打开夹爪
     * @param pos 目标位置 (rad)
     * @param vel 目标速度 (rad/s)
     * @param max_torque 最大力矩 (Nm)
     */
    void gripperOpen(double pos = 1.6, double vel = 0.5, double max_torque = 0.5);

    /**
     * @brief 关闭夹爪
     * @param pos 目标位置 (rad)
     * @param vel 目标速度 (rad/s)
     * @param max_torque 最大力矩 (Nm)
     */
    void gripperClose(double pos = 0.0, double vel = 0.5, double max_torque = 0.5);

    // ==================== 位置检测接口 ====================

    /**
     * @brief 检查关节位置是否到达
     * @param target_positions 目标位置数组
     * @param tolerance 位置容差 (rad)
     * @param position_errors 输出位置误差数组
     * @return 是否全部到达
     */
    bool checkPositionReached(const std::vector<double>& target_positions,
                              double tolerance,
                              std::vector<double>& position_errors);

    /**
     * @brief 等待位置到达
     * @param target_positions 目标位置数组
     * @param tolerance 位置容差 (rad)
     * @param timeout 超时时间 (s)
     * @return 是否成功到达
     */
    bool waitForPosition(const std::vector<double>& target_positions,
                         double tolerance = 0.01,
                         double timeout = 15.0);

    // ==================== 摩擦补偿 ====================

    /**
     * @brief 计算摩擦力补偿力矩（库伦摩擦 + 粘性摩擦模型）
     * @param vel 关节速度数组 (rad/s)，如果为空则使用当前速度
     * @param Fc 库伦摩擦系数数组 (Nm)
     * @param Fv 粘性摩擦系数数组 (Nm·s/rad)
     * @param vel_threshold 速度阈值 (rad/s)，低于此值使用特殊处理避免抖动
     * @return 摩擦力补偿力矩数组 (Nm)
     */
    std::vector<double> getFrictionCompensation(
        const std::vector<double>& vel,
        const std::vector<double>& Fc,
        const std::vector<double>& Fv,
        double vel_threshold = 0.01);

    // ==================== 多项式轨迹插值 ====================

    /**
     * @brief 五次多项式插值轨迹生成（速度、加速度连续）
     * @param start_pos 起始位置
     * @param end_pos 目标位置
     * @param duration 运动时间 (s)
     * @param current_time 当前时间 (s)
     * @param out_pos 输出位置
     * @param out_vel 输出速度
     * @param out_acc 输出加速度
     */
    static void quinticInterpolation(
        const std::vector<double>& start_pos,
        const std::vector<double>& end_pos,
        double duration,
        double current_time,
        std::vector<double>& out_pos,
        std::vector<double>& out_vel,
        std::vector<double>& out_acc);

    /**
     * @brief 七次多项式插值轨迹生成（速度、加速度、加加速度连续）
     * @param start_pos 起始位置
     * @param end_pos 目标位置
     * @param duration 运动时间 (s)
     * @param current_time 当前时间 (s)
     * @param out_pos 输出位置
     * @param out_vel 输出速度
     * @param out_acc 输出加速度
     */
    static void septicInterpolation(
        const std::vector<double>& start_pos,
        const std::vector<double>& end_pos,
        double duration,
        double current_time,
        std::vector<double>& out_pos,
        std::vector<double>& out_vel,
        std::vector<double>& out_acc);

    /**
     * @brief 七次多项式插值轨迹生成（指定起始和终止速度）
     * @param start_pos 起始位置
     * @param end_pos 目标位置
     * @param start_vel 起始速度
     * @param end_vel 终止速度
     * @param duration 运动时间 (s)
     * @param current_time 当前时间 (s)
     * @param out_pos 输出位置
     * @param out_vel 输出速度
     * @param out_acc 输出加速度
     */
    static void septicInterpolationWithVelocity(
        const std::vector<double>& start_pos,
        const std::vector<double>& end_pos,
        const std::vector<double>& start_vel,
        const std::vector<double>& end_vel,
        double duration,
        double current_time,
        std::vector<double>& out_pos,
        std::vector<double>& out_vel,
        std::vector<double>& out_acc);

    // ==================== 工具方法 ====================

    /**
     * @brief 获取关节限位
     * @param lower 输出下限数组
     * @param upper 输出上限数组
     */
    void getJointLimits(std::vector<double>& lower, std::vector<double>& upper) const;

    /**
     * @brief 获取电机数量（不含夹爪）
     * @return 电机数量
     */
    int getMotorCount() const { return motor_count_; }

    /**
     * @brief 获取夹爪电机索引
     * @return 夹爪电机索引
     */
    int getGripperId() const { return gripper_id_; }

    // ==================== 运动学和动力学接口 ====================

    /**
     * @brief 正运动学结构体
     */
    struct ForwardKinematicsResult {
        Eigen::Vector3d position;           ///< 末端位置 (m)
        Eigen::Matrix3d rotation;          ///< 旋转矩阵
        Eigen::Matrix4d transform;         ///< 4x4齐次变换矩阵
        std::vector<double> joint_angles;  ///< 关节角度 (rad)

        ForwardKinematicsResult() : position(Eigen::Vector3d::Zero()),
                                  rotation(Eigen::Matrix3d::Identity()),
                                  transform(Eigen::Matrix4d::Identity()) {}
    };

    /**
     * @brief 计算正运动学（考虑工具偏移）
     * @param joint_angles 关节角度数组，如果为空则使用当前角度
     * @return 正运动学结果（位置、旋转、变换矩阵）
     */
    ForwardKinematicsResult forwardKinematics(const std::vector<double>& joint_angles = {});

    /**
     * @brief 计算逆运动学（考虑工具偏移）
     * @param target_position 目标位置 (m)
     * @param target_rotation 目标旋转矩阵，如果为空则使用单位矩阵
     * @param init_q 初始关节角度，如果为空则使用当前角度
     * @param max_iter 最大迭代次数
     * @param eps 收敛阈值
     * @return 关节角度数组，如果未收敛则返回空数组
     */
    std::vector<double> inverseKinematics(
        const Eigen::Vector3d& target_position,
        const Eigen::Matrix3d* target_rotation = nullptr,
        const std::vector<double>& init_q = {},
        int max_iter = 1000,
        double eps = 1e-4);

    /**
     * @brief 获取重力补偿力矩 G(q)
     * @param q 关节角度数组，如果为空则使用当前角度
     * @return 重力力矩数组 (Nm)
     */
    std::vector<double> getGravity(const std::vector<double>& q = {});

    /**
     * @brief 获取科氏力矩阵 C(q,v)
     * @param q 关节角度数组，如果为空则使用当前角度
     * @param v 关节速度数组，如果为空则使用当前速度
     * @return 科氏力矩阵
     */
    Eigen::MatrixXd getCoriolis(const std::vector<double>& q = {},
                                 const std::vector<double>& v = {});

    /**
     * @brief 获取科氏力向量 C(q,v)*v
     * @param q 关节角度数组，如果为空则使用当前角度
     * @param v 关节速度数组，如果为空则使用当前速度
     * @return 科氏力向量 (Nm)
     */
    std::vector<double> getCoriolisVector(const std::vector<double>& q = {},
                                         const std::vector<double>& v = {});

    /**
     * @brief 获取质量矩阵 M(q)
     * @param q 关节角度数组，如果为空则使用当前角度
     * @return 质量矩阵
     */
    Eigen::MatrixXd getMassMatrix(const std::vector<double>& q = {});

    /**
     * @brief 获取惯性力矩 M(q)*a
     * @param q 关节角度数组，如果为空则使用当前角度
     * @param a 关节加速度数组，如果为空则使用零加速度
     * @return 惯性力矩数组 (Nm)
     */
    std::vector<double> getInertiaTerms(const std::vector<double>& q = {},
                                       const std::vector<double>& a = {});

    /**
     * @brief 获取完整动力学 tau = M(q)*a + C(q,v)*v + G(q)
     * @param q 关节角度数组，如果为空则使用当前角度
     * @param v 关节速度数组，如果为空则使用当前速度
     * @param a 关节加速度数组，如果为空则使用零加速度
     * @return 动力学力矩数组 (Nm)
     */
    std::vector<double> getDynamics(const std::vector<double>& q = {},
                                   const std::vector<double>& v = {},
                                   const std::vector<double>& a = {});

    /**
     * @brief 将旋转矩阵转换为欧拉角（ZYX顺序，单位：度）
     * @param rotation 旋转矩阵
     * @return 欧拉角数组 [roll, pitch, yaw] (度)
     */
    static std::vector<double> rotationMatrixToEuler(const Eigen::Matrix3d& rotation);

    /**
     * @brief 力矩限幅
     * @param torque 输入力矩数组
     * @param max_torque 最大力矩数组
     * @return 限幅后的力矩数组
     */
    static std::vector<double> clipTorque(const std::vector<double>& torque,
                                         const std::vector<double>& max_torque);

    /**
     * @brief 检查Pinocchio模型是否已加载
     * @return 是否已加载
     */
    bool isModelLoaded() const { return model_loaded_; }

private:
    /**
     * @brief 初始化机械臂
     * @param config_path 配置文件路径
     */
    void initialize(const std::string& config_path);

    /**
     * @brief 初始化成员变量（配置文件已由父类加载）
     */
    void initializeMembers();

    /**
     * @brief 加载配置文件
     * @param config_path 配置文件路径
     */
    void loadConfig(const std::string& config_path);

    /**
     * @brief 检查关节限位
     * @param pos 目标位置数组
     * @return 是否在限位范围内
     */
    bool checkJointLimits(const std::vector<double>& pos);

    /**
     * @brief 检查夹爪限位
     * @param pos 目标位置
     * @return 是否在限位范围内
     */
    bool checkGripperLimits(double pos);

    /**
     * @brief 加载URDF模型
     */
    void loadURDFModel();

    /**
     * @brief 将关节角度转换为Pinocchio配置向量
     * @param joint_angles 关节角度数组
     * @return Pinocchio配置向量
     */
    Eigen::VectorXd jointAnglesToPinocchioQ(const std::vector<double>& joint_angles);

    /**
     * @brief 从Pinocchio配置向量提取关节角度
     * @param q Pinocchio配置向量
     * @return 关节角度数组
     */
    std::vector<double> pinocchioQToJointAngles(const Eigen::VectorXd& q);

    // 成员变量
    int motor_count_;                          // 电机数量（不含夹爪）
    int gripper_id_;                           // 夹爪电机索引
    YAML::Node config_;                        // 配置文件内容
    std::string config_dir_;                   // 配置文件目录
    std::vector<double> joint_limits_lower_;   // 关节下限
    std::vector<double> joint_limits_upper_;   // 关节上限
    double gripper_limit_lower_;               // 夹爪下限
    double gripper_limit_upper_;               // 夹爪上限
    std::vector<double> velocity_limits_;      // 关节速度限制
    std::vector<double> max_torque_;           // 关节最大力矩
    std::vector<std::string> joint_names_;     // 关节名称

    bool model_loaded_;                        // URDF模型是否已加载
    pinocchio::Model model_;                   // Pinocchio模型
    pinocchio::Data data_;                     // Pinocchio数据
    std::vector<int> joint_ids_;               // Pinocchio关节ID列表
    double tool_offset_;                       // 工具偏移 (m)
};

} // namespace panthera

#endif // _PANTHERA_HPP_
