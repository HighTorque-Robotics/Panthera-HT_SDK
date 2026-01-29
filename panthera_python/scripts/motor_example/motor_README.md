# 高扭矩机器人电机控制 Python 接口

这是一个为高扭矩机器人电机控制C++库提供的Python接口。通过pybind11实现C++到Python的绑定。
需要通过电机底层开发可以使用这个，使用前先熟悉代码。

## 功能特性

- 支持多种高扭矩电机型号 (4438、5046、5047、6056等系列)
- 多种控制模式:
  - 位置控制
  - 速度控制
  - 力矩控制
  - 电压控制
  - 电流控制
  - 混合控制模式 (位置+速度+力矩+PID参数)
- 支持多CAN总线、多电机控制
- 安全功能: 位置限制、力矩限制、超时检测
- 基于YAML的配置系统

## 依赖项

### 系统依赖
```bash
sudo apt-get install -y \
    cmake \
    python3-dev \
    python3-pip \
    liblcm-dev \
    libyaml-cpp-dev \
    libserialport-dev
```

### Python依赖
```bash
pip3 install pybind11 numpy
```

### C++项目
需要先编译C++项目:
```bash
cd path/to/hightorque_robot
mkdir -p build && cd build
cmake ..
make
```

## 安装

### 方式1: 使用pip安装 (开发模式)
```bash
cd path/to/hightorque_robot_python
pip3 install -e .
```

### 方式2: 使用CMake构建
```bash
cd path/to/hightorque_robot_python
mkdir -p build && cd build
cmake ..
make
```

## 使用示例

### 基础示例
```python
import hightorque_robot as htr

# 使用配置文件创建电机对象
robot = htr.Robot("/path/to/robot_config.yaml")

# 启用LCM消息发布
robot.lcm_enable()

# 获取所有电机
motors = robot.get_motors()
print(f"电机数量: {len(motors)}")

# 控制第一个电机
motor = motors[0]
motor.position(0.5)  # 位置控制
# motor.velocity(1.0)  # 速度控制
# motor.torque(5.0)    # 力矩控制

# 发送控制命令
robot.motor_send_cmd()

# 获取电机状态
state = motor.get_current_motor_state()
print(f"电机 {state.ID}: 位置={state.position}, 速度={state.velocity}, 力矩={state.torque}")

# 停止所有电机
robot.set_stop()
```

### 高级控制示例
```python
import hightorque_robot as htr
import time

robot = htr.Robot("/path/to/robot_config.yaml")
motors = robot.get_motors()

# 使用混合控制模式
for motor in motors:
    # 位置 + 速度 + 最大力矩
    motor.pos_vel_MAXtqe(
        position=1.0,
        velocity=0.5,
        torque_max=10.0
    )

# 发送命令
robot.motor_send_cmd()

# 循环控制
while True:
    # 更新控制命令
    for i, motor in enumerate(motors):
        motor.pos_vel_MAXtqe(1.0 if i % 2 == 0 else -1.0, 0.1, 10.0)

    robot.motor_send_cmd()

    # 读取状态
    for motor in motors:
        state = motor.get_current_motor_state()
        print(f"电机 {state.ID}: pos={state.position:.3f}, vel={state.velocity:.3f}")

    time.sleep(0.001)  # 1ms控制周期
```

### 配置文件使用
```python
import hightorque_robot as htr

# 使用YAML配置文件初始化
robot = htr.Robot("path/to/hightorque_robot/robot_param/robot_config.yaml")

# 查看机器人名称
print(f"机器人名称: {robot.robot_params.robot_name}")

# 查看电机数量
print(f"电机总数: {len(robot.get_motors())}")

# 设置超时
robot.set_timeout(100)  # 100ms

# 重置所有电机
robot.set_reset()
```

## API 参考

### Robot类
- `Robot()` / `Robot(config_path)` - 构造函数
- `get_motors()` - 获取所有电机列表
- `motor_send_cmd()` - 发送电机控制命令
- `set_stop()` - 停止所有电机
- `set_reset()` - 重启所有电机
- `set_reset_zero()` / `set_reset_zero(motor_id)` - 重置电机零点
- `send_get_motor_state_cmd()` - 请求电机状态
- `set_timeout(timeout_ms)` - 设置超时时间
- `lcm_enable()` - 启用LCM消息发布

### Motor类
- `position(pos)` - 位置控制
- `velocity(vel)` - 速度控制
- `torque(tqe)` - 力矩控制
- `voltage(vol)` - 电压控制
- `current(cur)` - 电流控制
- `pos_vel_MAXtqe(pos, vel, tqe_max)` - 位置+速度+最大力矩
- `pos_vel_tqe_kp_kd(pos, vel, tqe, kp, kd)` - 五参数控制
- `pos_vel_kp_kd(pos, vel, kp, kd)` - 位置+速度+PID
- `pos_vel_acc(pos, vel, acc)` - 位置+速度+加速度
- `get_current_motor_state()` - 获取电机状态
- `get_motor_id()` - 获取电机ID
- `stop()` - 停止电机
- `brake()` - 刹车
- `reset()` - 重启电机

### MotorState类
电机状态数据结构:
- `ID` - 电机ID
- `mode` - 运行模式
- `fault` - 故障码
- `position` - 位置 (弧度)
- `velocity` - 速度 (弧度/秒)
- `torque` - 力矩 (N·m)
- `Kp` - PID Kp参数
- `Kd` - PID Kd参数

## 故障排除

### 找不到C++库
确保C++项目已编译:
```bash
cd path/to/hightorque_robot/build
cmake ..
make
```

### 找不到pybind11
```bash
pip3 install pybind11
```

### 权限问题 (串口访问)
```bash
sudo usermod -a -G dialout $USER
# 注销后重新登录
```

### 导入错误
确保Python能找到编译的模块:
```bash
export PYTHONPATH=path/to/hightorque_robot_python:$PYTHONPATH
```

## 项目结构

```
├── motor_example/
    └── motor_control.py            # 示例
    └── 01_motor_get_status.py      # 读取电机状态
    └── 02_position_control.py      # 位置控制
    └── 03_velocity_control.py      # 速度控制
    └── 04_torque_control.py        # 力矩控制
    └── 05_voltage_control.py       # 扭矩电压控制
    └── 06_current_control.py       # 扭矩电流控制
    └── 07_pos_vel_maxtorque.py     # 位置-速度-最大扭矩控制
    └── 08_pos_vel_torque_kp_kd.py  # 示例程序
```

## 许可证

MIT License

## 贡献

欢迎提交Issue和Pull Request!

## 联系方式

如有问题,请提交Issue或联系维护者。
