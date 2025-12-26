# CLAUDE.md

本文件为Claude Code (claude.ai/code)在此代码库中工作时提供指导。

## 构建系统和命令

### 构建命令
```bash
# 标准构建流程
mkdir build && cd build
cmake ..
make -j8

# 安装到自定义路径（默认：build/install）
cmake -DCMAKE_INSTALL_PREFIX=/custom/path ..
make install
```

### 依赖项
- **libserialport**: 串口通信必需 (`sudo apt-get install libserialport-dev`)
- **yaml-cpp**: 配置文件解析 (`sudo apt-get install libyaml-cpp-dev`)
- **LCM**: 轻量级通信和编组库 (`find_package(lcm REQUIRED)`)
- **serial**: 串口库依赖 (`find_package(serial REQUIRED)`)

### 构建的可执行文件
- `motor_run`: 主要的电机控制演示程序
- `motor_msg_subscriber`: 电机数据的LCM消息订阅器
- `parse_demo`: 配置解析演示程序

## 架构概述

### 核心组件层次结构
```
robot (顶层控制器)
├── CANboards (通信硬件抽象)
│   └── CANports (独立的CAN总线端口)
│       └── Motors (单个电机实例)
└── serial_driver (底层串口通信)
```

### 关键类及其作用

**`hightorque_robot::robot`** (`include/hardware/robot.hpp`, `src/hardware/robot.cpp`)
- 管理多个CAN板和电机的顶层机器人控制器
- 处理电机状态的LCM消息发布
- 管理错误检查和超时检测
- 提供高级电机控制接口
- SDK版本：4.4.7

**`canboard`** (`include/hardware/canboard.hpp`, `src/hardware/canboard.cpp`) 
- 代表物理CAN通信板硬件
- 管理每个板上的多个CAN端口
- 处理板级操作，如引导加载器模式和FDCAN复位

**`canport`** (`include/hardware/canport.hpp`, `src/hardware/canport.cpp`)
- CAN板上的独立CAN总线端口
- 将连接到同一CAN总线的电机分组
- 管理端口特定的通信

**`motor`** (`include/hardware/motor.hpp`, `src/hardware/motor.cpp`)
- 单个电机控制和状态管理
- 支持多种控制模式：位置、速度、力矩和混合控制
- 支持的电机类型：4438、5046、5047、6056系列
- 位置和力矩限制功能

### 配置系统

**机器人配置** (`robot_param/robot_config.yaml`)
- 主要的机器人配置文件
- 引用不同DOF配置的特定参数文件

**参数文件** (`robot_param/*dof_STM32H730_model_test_Orin_params.yaml`)
- 详细的电机配置，包括：
  - 电机类型和ID
  - 位置和力矩限制
  - CAN板和端口分配
  - 串口通信参数

### 通信架构

**串口通信**
- 通过USB虚拟串口与CANFD协议转换板通信
- 默认设备模式：`/dev/ttyACM*`
- 波特率：4,000,000（可配置）
- 支持多个串口用于多个CAN板

**LCM消息系统**
- 在`msg/motor_msg/motor_msg.hpp`中自动生成的消息类型
- 实时电机状态发布
- 电机状态数组支持4个CAN总线上最多40个电机

**CRC保护**
- 用于数据完整性的CRC8和CRC16实现
- 位于`src/crc/`和`include/crc/`

### 开发模式

**电机控制流程**
1. 使用配置文件初始化机器人
2. 使用`rb.lcm_enable()`启用LCM发布
3. 使用电机控制方法设置电机命令（例如`pos_vel_MAXtqe()`）
4. 调用`rb.motor_send_cmd()`传输命令
5. 使用`get_current_motor_state()`读取电机状态

**配置加载**
- 从`robot_param/`中的YAML文件加载机器人参数
- `parse_robot_params.cpp`中的解析函数处理配置解析
- 默认配置指向1DOF设置

**错误处理**
- 内置错误检查线程用于电机超时
- 位置和力矩限制监控
- 初始化期间的电机连接验证