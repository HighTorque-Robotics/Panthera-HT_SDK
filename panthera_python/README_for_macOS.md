# Panthera-HT SDK — macOS (Apple Silicon) 安装指南

本文档记录在 macOS Apple Silicon (M系列芯片) 上从源码编译并运行 Panthera-HT SDK 的完整步骤。

---

## 环境要求

- macOS 12+ (Apple Silicon, arm64)
- Homebrew
- Miniconda / Anaconda
- Xcode Command Line Tools (`xcode-select --install`)

---

## 第一步：安装系统依赖

```bash
brew install cmake yaml-cpp libserialport pybind11
```

---

## 第二步：创建 conda 环境

```bash
conda create -n panthera python=3.10
conda activate panthera
pip install pybind11
```

---

## 第三步：编译 C++ 电机 SDK

```bash
cd /path/to/Panthera-HT_SDK/panthera_cpp/motor_cpp
mkdir -p build && cd build
cmake ..
make -j$(sysctl -n hw.logicalcpu)
```

编译产物会安装到 `~/.local/lib/`（包含 `libhightorque_motor.4.dylib` 等）。

---

## 第四步：编译 Python 绑定

```bash
cd /path/to/Panthera-HT_SDK/panthera_python
mkdir -p build && cd build
cmake ..
make -j$(sysctl -n hw.logicalcpu)
```

编译成功后会在 `hightorque_robot/` 目录下生成 `_hightorque_robot.cpython-310-darwin.so`。

---

## 第五步：安装 Python 依赖

```bash
cd /path/to/Panthera-HT_SDK/panthera_python
pip install -r requirements.txt
```

> `pin` 包（pinocchio 动力学库）体积较大（~5MB），网络不稳定时可能需要多次重试。

---

## 第六步：配置 conda 环境变量

让每次 `conda activate panthera` 自动设置所需的环境变量：

```bash
mkdir -p /opt/miniconda3/envs/panthera/etc/conda/activate.d
mkdir -p /opt/miniconda3/envs/panthera/etc/conda/deactivate.d
```

创建激活脚本 `/opt/miniconda3/envs/panthera/etc/conda/activate.d/panthera_env.sh`：

```bash
export DYLD_LIBRARY_PATH=$HOME/.local/lib:/opt/homebrew/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}
export PYTHONPATH=/path/to/Panthera-HT_SDK/panthera_python${PYTHONPATH:+:$PYTHONPATH}
```

创建停用脚本 `/opt/miniconda3/envs/panthera/etc/conda/deactivate.d/panthera_env.sh`：

```bash
unset DYLD_LIBRARY_PATH
unset PYTHONPATH
```

之后重新激活环境即可生效：

```bash
conda deactivate && conda activate panthera
```

---

## 第七步：修改配置文件（macOS 串口适配）

macOS 上串口设备名称与 Linux 不同，需修改电机参数文件：

文件：`robot_param/motor_param/6dof_Panthera_params_follower.yaml`（或 leader 版本）

```yaml
# 将 Linux 的设备名
Serial_Type: "/dev/ttyACM"

# 改为 macOS 的设备名
Serial_Type: "/dev/tty.usbmodem"
```

---

## 第八步：运行脚本

```bash
conda activate panthera
cd /path/to/Panthera-HT_SDK/panthera_python/scripts
python3 0_robot_get_state.py
```

启动成功后会显示串口列表、CANboard 版本，以及各关节的位置、速度、力矩状态。

---

## 常见问题

### `No module named 'hightorque_robot'`

`PYTHONPATH` 未设置。确认第六步的 conda 激活脚本已创建，并重新执行 `conda deactivate && conda activate panthera`。

### `Library not loaded: @rpath/libhightorque_motor.4.dylib`

`DYLD_LIBRARY_PATH` 未设置。同上，确认激活脚本中包含：

```bash
export DYLD_LIBRARY_PATH=$HOME/.local/lib:/opt/homebrew/lib
```

### `RuntimeError: invalid node`（YAML 解析错误）

检查 YAML 文件格式，常见问题：
- 注释后面不能跟其他字段，必须另起一行
- `Follower.yaml` 需要包含 `robot_name` 字段：
  ```yaml
  robot:
    name: "Panthera-HT"
    robot_name: "Panthera-HT"
  ```

### 串口列表为空

确认 `Serial_Type` 已改为 `/dev/tty.usbmodem`（见第七步）。

可用以下命令确认设备已连接：

```bash
ls /dev/tty.usbmodem*
```

正常连接后应看到 7 个设备（`/dev/tty.usbmodem2024051701*`）。

### 所有电机显示 `v0.0.0` / 关节位置为 `999.000`

这是电机未响应时的占位值，属于硬件问题：
- 确认机械臂主电源（DC电源）已打开（电机电源指示灯亮绿灯）
- 检查 CANboard 到电机的 CAN 线连接是否牢固
- CANboard 本身通过 USB 供电，电机需要独立的高压电源
- 端口可能读反，尝试将电机通信线束从CAN1口换成CAN7口

---

## macOS 与 Linux 的主要差异

| 项目 | Linux | macOS |
|------|-------|-------|
| 串口设备名 | `/dev/ttyACM*` | `/dev/tty.usbmodem*` |
| 动态库路径变量 | `LD_LIBRARY_PATH` | `DYLD_LIBRARY_PATH` |
| 串口权限 | 需要 `sudo chmod 777` | 无需额外授权 |
| VID/PID 过滤 | 正常工作 | `sp_get_port_usb_vid_pid` 返回无效值，需在 C++ 中用 `#ifdef __APPLE__` 跳过 |
| 预编译 whl | 可直接使用 `motor_whl/` 中的包 | 需从源码编译 |
