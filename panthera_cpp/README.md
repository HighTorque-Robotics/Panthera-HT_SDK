# Panthera-HT C++ SDK

Panthera 机械臂 C++ SDK，提供完整的机械臂控制解决方案。

## 概述

Panthera C++ SDK 分为两个层级：

- **motor_cpp**: 电机底层驱动 SDK - 提供单个电机的基础控制功能
- **robot_cpp**: 机械臂高级控制 SDK - 基于 motor_cpp 构建的机械臂级别控制库

### SDK 架构关系

```
┌─────────────────────────────────────────┐
│   robot_cpp (panthera_robot 库)         │
│   机械臂高级控制                          │
│   - 运动学/动力学计算                      │
│   - 轨迹规划                             │
│   - 多关节协调控制                        │
└──────────────┬──────────────────────────┘
               │ depends on
┌──────────────▼──────────────────────────┐
│   motor_cpp (hightorque_motor 库)       │
│   电机底层驱动                            │
│   - CAN 通信                             │
│   - 电机控制命令                          │
│   - 状态反馈读取                          │
└──────────────┬──────────────────────────┘
               │ depends on
┌──────────────▼──────────────────────────┐
│   硬件层                                  │
│   - CAN 总线                             │
│   - 串口通信                             │
│   - 电机驱动器                            │
└─────────────────────────────────────────┘
```

## 选择合适的 SDK

### 使用 motor_cpp
适合以下场景：
- 需要直接控制单个电机
- 开发自定义电机控制算法
- 需要底层硬件访问
- 测试和调试电机硬件

**📖 详细文档**: [motor_cpp/README.md](./motor_cpp/README.md)

### 使用 robot_cpp （推荐）
适合以下场景：
- 控制完整的机械臂系统
- 需要运动学/动力学计算
- 需要轨迹规划功能
- 实现复杂的控制策略
- 主从协同控制

**📖 详细文档**: [robot_cpp/README.md](./robot_cpp/README.md)

## 快速开始

### 系统要求
- Ubuntu 20.04 / 22.04 / 24.04
- CMake >= 3.10
- C++17 或更高版本

### 安装依赖

```bash
# 安装基础依赖
sudo apt update
sudo apt install -y build-essential cmake git
sudo apt install -y libyaml-cpp-dev liblcm-dev

# 安装 Pinocchio（robot_cpp 需要用于动力学计算）
# 方式一：使用二进制包
sudo apt install pinocchio

# 方式二：从源码编译
# 详细教程: https://github.com/stack-of-tasks/pinocchio
```

### 编译

#### 编译 motor_cpp（电机底层 SDK）

```bash
cd motor_cpp
mkdir build && cd build
cmake ..
make -j$(nproc)
```

#### 编译 robot_cpp（机械臂高级 SDK）

```bash
cd robot_cpp
mkdir build && cd build
cmake ..
make -j$(nproc)
```

> **注意**: robot_cpp 会自动链接 motor_cpp，无需单独编译 motor_cpp。

## 项目结构

```
panthera_cpp/
├── motor_cpp/                  # 电机底层驱动 SDK
│   ├── include/                # 头文件
│   ├── src/                   # 源文件
│   ├── msg/                   # LCM 消息定义
│   ├── third_part/            # 第三方库
│   ├── example/               # 电机控制示例
│   ├── robot_param/           # 电机参数配置
│   │
│   ├── CMakeLists.txt         # motor_cpp 构建配置
│   └── README.md              # motor_cpp 详细文档
│
└── robot_cpp/                 # 机械臂控制 SDK
    ├── include/               # 头文件
    ├── src/                   # 源文件
    ├── example/               # 机械臂控制示例
    ├── robot_param/           # 机械臂配置文件
    ├── Panthera-HT_description/  # 机械臂描述文件
    │
    ├── CMakeLists.txt         # robot_cpp 构建配置
    └── README.md              # robot_cpp 详细文档
```

## 库和命名空间

### motor_cpp 库
- **库名称**: `hightorque_motor`
- **命名空间**: `hightorque_robot`
- **主要类**:
  - `hightorque_robot::motor` - 单个电机控制
  - `hightorque_robot::canport` - CAN 端口管理
  - `hightorque_robot::canboard` - CAN 板控制
  - `hightorque_robot::robot` - 底层机器人控制

### robot_cpp 库
- **库名称**: `panthera_robot`
- **命名空间**: `panthera`
- **主要类**:
  - `panthera::Panthera` - 机械臂高级控制类（继承自 `hightorque_robot::robot`）
  - `panthera::SignalHandler` - 信号处理辅助类

## 功能对比

| 功能 | motor_cpp | robot_cpp |
|------|-----------|-----------|
| 电机控制（位置/速度/力矩） | ✅ | ✅ |
| MIT 模式控制 | ✅ | ✅ |
| CAN 通信 | ✅ | ✅ |
| 状态反馈读取 | ✅ | ✅ |
| 运动学计算 | ❌ | ✅ |
| 动力学计算 | ❌ | ✅ |
| 轨迹规划 | ❌ | ✅ |
| 重力补偿 | ❌ | ✅ |
| 摩擦补偿 | ❌ | ✅ |
| 夹爪控制 | ❌ | ✅ |
| 主从协同 | ❌ | ✅ |

## 下一步

- 📖 **电机 SDK 详细文档**: [motor_cpp/README.md](./motor_cpp/README.md)
- 📖 **机械臂 SDK 详细文档**: [robot_cpp/README.md](./robot_cpp/README.md)

## 许可证

MIT License

## 贡献

欢迎提交 Issue 和 Pull Request!

## 联系方式

如有问题，请提交 Issue 或联系维护者。
