# Panthera 机械臂控制 C++ SDK

## 功能特性

- **多种控制模式**:
  - 位置速度控制（含最大力矩限制）
  - 五参数 MIT 模式控制（位置+速度+力矩+Kp+Kd）
  - 夹爪独立控制
- **运动学计算**:
  - 正运动学（FK）
  - 逆运动学（IK）
- **动力学计算**:
  - 重力补偿
  - 科氏力补偿
  - 质量矩阵
  - 摩擦力补偿
  - 完整动力学模型
- **轨迹规划**:
  - 五次多项式插值（速度、加速度连续）
  - 七次多项式插值（速度、加速度、加加速度连续）
  - 带速度约束的七次多项式插值
- **安全功能**:
  - 关节位置限制
  - 力矩限制
  - 超时检测
  - 位置到达检测
- **主从协同控制**：支持双臂协同操作和轨迹记录回放
- **基于 YAML 的配置系统**
- **高性能**：编译型语言，适合实时控制系统

## 环境安装

### 系统要求
- Ubuntu 20.04 / 22.04 / 24.04
- CMake >= 3.10
- C++17 或更高版本

### 安装步骤

1. **安装依赖**

```bash
# 安装基础依赖
sudo apt update
sudo apt install -y build-essential cmake git
sudo apt install -y libyaml-cpp-dev liblcm-dev

# 安装 Pinocchio（用于动力学计算）
# 方式一：使用二进制包
sudo apt install pinocchio

# 方式二：从源码编译
# 详细教程: https://github.com/stack-of-tasks/pinocchio


```

2. **编译安装**

```bash
cd ~/Panthera-HT_git/Panthera-HT_SDK/panthera_cpp/robot_cpp
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

编译成功后，库文件会安装到系统路径，可以在任何位置使用。

## 快速开始

### 赋串口权限
设备正常连接可以看到七个设备
```bash
ls /dev/ttyACM*
```
赋予权限：
```bash
sudo chmod -R 777 /dev/ttyACM*
```

### 测试示例
a. 首次打开先在 `build` 目录下运行 `0_robot_get_state` 程序，查看各关节状态
```bash
cd build
./0_robot_get_state
```
启动成功后会显示端口号、电机ID、和初始化电机的信息，随后循环打印各关节位置、速度等状态。

b. 运行位置速度控制程序
```bash
./1_PosVel_control
```
机械臂会按指定速度运动到不同位置，并控制夹爪开合。若机械臂依次完成动作，则说明工作正常。

### 实现记录与播放轨迹
运行 `5_record_trajectory` 程序，进行主从协同同时记录机械臂运动轨迹。记录完成后按 Ctrl+C 退出程序，轨迹文件会自动保存在当前目录下（命名如 trajectory_20251211_160537.jsonl）。
```bash
./5_record_trajectory
```
随后在 `5_replay_trajectory` 程序内修改 trajectory_file 变量，将路径改成刚刚保存好的文件路径，然后重新编译运行该程序，主臂会自动运行记录好的轨迹。
```bash
./5_replay_trajectory
```
若想使用从臂运行轨迹，则在程序中将 config_path 变量中的 Leader.yaml 路径改成 Follower.yaml。

### 主从遥操
将从臂连接至can口1,主臂连接至can口2
启动程序：
```bash
./5_teleop_control
```
即可看到遥操效果

## 使用示例

### 基础控制示例
参考0_robot_get_state.cpp、0_robot_set_zero.cpp、1_PosVel_control.cpp等等

## API 参考

### Robot 类

Robot 类提供了机械臂级别的控制接口。

#### 初始化
- `Robot(config_path = "")` - 创建机械臂实例
  - `config_path`: 配置文件路径，默认使用 Leader.yaml

#### 状态获取
- `get_current_state()` - 获取所有关节状态列表
- `get_current_pos()` - 获取当前关节角度 (std::vector<double> [6])
- `get_current_vel()` - 获取当前关节速度 (std::vector<double> [6])
- `get_current_torque()` - 获取当前关节力矩 (std::vector<double> [6])
- `get_current_state_gripper()` - 获取夹爪状态
- `get_current_pos_gripper()` - 获取夹爪位置
- `get_current_vel_gripper()` - 获取夹爪速度
- `get_current_torque_gripper()` - 获取夹爪力矩

#### 控制命令
- `pos_vel_MAXtqe(pos, vel, max_tqu, iswait = false, tolerance = 0.1, timeout = 15.0)`
  - 位置速度最大力矩控制
  - `pos`: 目标位置数组 [6] (rad)
  - `vel`: 目标速度数组 [6] (rad/s)
  - `max_tqu`: 最大力矩数组 [6] (Nm)
  - `iswait`: 是否等待到达目标
  - `tolerance`: 位置容差 (rad)
  - `timeout`: 超时时间 (s)

- `pos_vel_tqe_kp_kd(pos, vel, tqe, kp, kd)`
  - 五参数MIT控制模式
  - `pos`: 目标位置 [6] (rad)
  - `vel`: 目标速度 [6] (rad/s)
  - `tqe`: 前馈力矩 [6] (Nm)
  - `kp`: 位置增益 [6]
  - `kd`: 速度增益 [6]

- `gripper_control(pos, vel, max_tqu)` - 夹爪位置速度控制
- `gripper_control_MIT(pos, vel, tqe, kp, kd)` - 夹爪MIT控制
- `gripper_open(vel = 0.5, max_tqu = 0.5)` - 打开夹爪
- `gripper_close(pos = 0.0, vel = 0.5, max_tqu = 0.5)` - 关闭夹爪

#### 位置检测
- `check_position_reached(target_positions, tolerance = 0.1)` - 检查是否到达目标
- `wait_for_position(target_positions, tolerance = 0.01, timeout = 15.0)` - 等待到达目标

#### 运动学
- `forward_kinematics(joint_angles = {})` - 正运动学
  - 返回: `{'position': [x,y,z], 'rotation': R, 'transform': T, 'joint_angles': q}`

- `inverse_kinematics(target_position, target_rotation = {}, init_q = {}, max_iter = 1000, eps = 1e-4)` - 逆运动学
  - `target_position`: 目标位置 [x, y, z] (m)
  - `target_rotation`: 目标旋转矩阵 3x3 (可选)
  - `init_q`: 初始关节角度 (可选)
  - 返回: 关节角度解 [6] 或空向量

#### 动力学
- `get_Gravity(q = {})` - 重力补偿力矩 G(q)
- `get_Coriolis(q = {}, v = {})` - 科氏力矩阵 C(q,v)
- `get_Coriolis_vector(q = {}, v = {})` - 科氏力向量 C(q,v)*v
- `get_Mass_Matrix(q = {})` - 质量矩阵 M(q)
- `get_Inertia_Terms(q = {}, a = {})` - 惯性力矩 M(q)*a
- `get_Dynamics(q = {}, v = {}, a = {})` - 完整动力学 τ = M(q)a + C(q,v)v + G(q)
- `get_friction_compensation(vel = {}, Fc = {}, Fv = {}, vel_threshold = 0.01)` - 摩擦力补偿
  - 摩擦模型: τ_friction = Fc * sign(vel) + Fv * vel

#### 轨迹规划
- `quintic_interpolation(start_pos, end_pos, duration, current_time)`
  - 五次多项式插值（速度、加速度连续）
  - 返回: (位置, 速度, 加速度)

- `septic_interpolation(start_pos, end_pos, duration, current_time)`
  - 七次多项式插值（速度、加速度、加加速度连续）
  - 返回: (位置, 速度, 加速度)

- `septic_interpolation_with_velocity(start_pos, end_pos, start_vel, end_vel, duration, current_time)`
  - 带速度约束的七次多项式插值
  - 返回: (位置, 速度, 加速度)

#### 继承的基础方法
- `motor_send_cmd()` - 发送控制命令到电机
- `send_get_motor_state_cmd()` - 请求电机状态
- `set_stop()` - 停止所有电机
- `set_reset()` - 重启所有电机
- `set_timeout(timeout_ms)` - 设置超时时间

### 电机参数配置 (robot_param/motor_param/*.yaml)
包含电机ID、CAN总线配置、电机型号等底层参数。

## 项目结构

```
panthera_cpp/robot_cpp/
├── include/                   # 头文件
│   └── panthera/
│       ├── Panthera.hpp       # Panthera类定义
│       └── SignalHandler.hpp  # 信号处理器
│
├── src/                       # 源文件
│   └── panthera/
│       └── Panthera.cpp       # Panthera类实现
│
├── example/                   # 控制示例
│   ├── 0_robot_get_state.cpp          # 状态查看
│   ├── 0_robot_set_zero.cpp           # 设置零位
│   │
│   ├── 1_PosVel_control.cpp           # 位置速度控制
│   ├── 1_PD_control.cpp               # PD控制
│   │
│   ├── 2_inv_PosVel_control.cpp       # 基于逆运动学的位置控制
│   ├── 2_gravity_compensation_control.cpp              # 重力补偿控制
│   ├── 2_gravity_friction_compensation_control.cpp    # 重力摩擦力补偿控制
│   ├── 2_joint_impedance_control.cpp                  # 关节阻抗控制(重力+PD)
│   ├── 2_joint_impedance_control_with_friction.cpp   # 关节阻抗控制(重力+摩擦力+PD)
│   │
│   ├── 3_interpolation_control_zeroVel.cpp            # 插值轨迹控制(零速度)
│   ├── 3_interpolation_control_nozeroVel.cpp          # 插值轨迹控制(非零速度)
│   ├── 3_sin_trajectory_control.cpp                   # 正弦轨迹控制
│   ├── 3_gravity_compensation_with_fk.cpp             # 重力补偿+正运动学
│   │
│   ├── 4_impedance_trajectory_control_with_gra_pd.cpp # 基于轨迹的阻抗控制
│   │
│   ├── 5_teleop_control.cpp            # 主从臂遥操作
│   ├── 5_record_trajectory.cpp         # 轨迹记录
│   └── 5_replay_trajectory.cpp         # 轨迹回放
│
├── robot_param/                # 机械臂配置文件
│   ├── Leader.yaml             # 主臂配置
│   ├── Follower.yaml           # 从臂配置
│   └── motor_param/            # 电机参数
│       ├── 6dof_Panthera_params_leader.yaml
│       └── 6dof_Panthera_params_follower.yaml
│
├── Panthera-HT_description/   # 机械臂URDF模型和描述文件
│   ├── urdf/                   # URDF文件
│   │   ├── Panthera-HT_description_leader.urdf
│   │   └── Panthera-HT_description_follower.urdf
│   ├── meshes/                 # 3D网格文件
│   ├── launch/                 # 启动文件
│   └── config/                 # 配置文件
│
├── CMakeLists.txt              # CMake构建配置
└── README.md                   # 本文档
```

## 故障排除

### 编译错误
- 确保所有依赖库已正确安装（libyaml-cpp-dev, liblcm-dev, pinocchio）
- 检查 CMake 版本是否满足要求（>= 3.10）
- 确认 C++17 或更高版本已启用

### URDF加载失败
检查配置文件中的URDF路径是否正确，确保相对路径从配置文件所在目录计算。

### 逆运动学不收敛
检查目标位置是否在机械臂工作空间内，可以调整 `max_iter` 和 `eps` 参数。

### 电机无法连接
检查板子开关情况，电机电源按钮亮绿灯则为上电
若依旧无法连接请检查电机之间连接线情况

## 小技巧

自动赋值串口权限
a. 修改udev规则文件：
```bash
sudo gedit /etc/udev/rules.d/99-tegra-devices.rules
```
b. 添加如下内容，并保存：
```bash
KERNEL=="ttyACM*", MODE="0777"
```
c. 重新加载udev规则：
```bash
sudo udevadm control --reload-rules
```
重新连接电脑和主板即可生效。

## 与 Python SDK 对比

### C++ SDK 优势
1. **性能更高**: 编译型语言，执行效率更高，适合实时控制
2. **类型安全**: 编译时类型检查，减少运行时错误
3. **内存管理**: 更精细的内存控制，适合嵌入式系统
4. **实时性**: 更好的实时性保证，适合高频控制

### 功能一致性
- 核心控制功能与 Python SDK 完全一致
- API 调用方式对应，命名风格符合 C++ 规范
- 配置文件格式完全相同

## 许可证

MIT License

## 贡献

欢迎提交 Issue 和 Pull Request!

## 联系方式

如有问题，请提交 Issue 或联系维护者。
