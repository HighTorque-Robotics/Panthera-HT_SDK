# Panthera-HT SDK 🚀

[![en](https://img.shields.io/badge/lang-English-blue.svg)](#english)[![中文](https://img.shields.io/badge/lang-简体中文-brown.svg)](#中文) 

<div id="中文"></div>

## 中文

**Panthera-HT SDK** 是 Panthera-HT 六轴机械臂的官方软件开发工具包，提供完整的 C++ 和 Python 接口，用于电机控制、运动学、动力学和轨迹规划。

### ✨ 主要特性

- **多语言支持**：完整的 C++ 和 Python API
- **丰富的控制模式**：位置、速度、力矩和混合控制
- **先进的运动学**：正/逆运动学及完整雅可比矩阵支持
- **动力学建模**：重力补偿、摩擦力补偿和完整动力学模型
- **轨迹规划**：五次和七次多项式插值
- **主从控制**：双臂遥操作和轨迹记录/回放
- **硬件支持**：最多 7 路 CAN 总线，每条总线支持 30 个电机

---

## 📁 仓库结构

```
Panthera-HT_SDK/
├── panthera_cpp/              # C++ SDK
├── panthera_python/           # Python SDK
├── LICENSE                    # MIT 许可证
└── README.md                  # 本文件
```

---

## 📦 安装

详细的安装说明请参考各语言目录下的 README：

- **C++ SDK**：参见 [panthera_cpp/README.md](panthera_cpp/README.md) 了解 C++ 依赖、构建说明和示例
- **Python SDK**：参见 [panthera_python/README.md](panthera_python/README.md) 了解 Python 依赖、安装和示例

---

## 🔗 相关开发资源

| 仓库 | 许可证 | 说明 |
|------|--------|------|
| **[Panthera-HT_Host](https://github.com/HighTorque-Robotics/Panthera-HT_Host)** | [MIT](LICENSE) | 基于 SDK 仓库构建的 Web 上位机，支持个别例程可视化控制，并集成多种 SDK 使用示例。 |
| **[Panthera-HT_ROS2](https://github.com/HighTorque-Robotics/Panthera-HT-ROS2)** | [MIT](LICENSE) | ROS2 开发包，提供机械臂的驱动、控制与仿真支持。 |
| **[Panthera-HT_lerobot](https://github.com/HighTorque-Robotics/Panthera-HT_lerobot)** | [MIT](LICENSE) | LeRobot 集成包，支持模仿学习和机器人学习算法。 |
| **[Panthera-HT_Model](https://github.com/HighTorque-Robotics/Panthera-HT_Model)** | [MIT](LICENSE) | SolidWorks原始设计文件、钣金图、3D打印文件和物料清单（BOM）。 |

---

## 📖 文档

- [C++ SDK 文档](panthera_cpp/README.md)
- [Python SDK 文档](panthera_python/README.md)
- [机器人参数](https://github.com/HighTorque-Robotics/Panthera-HT_Main/blob/main/images/parameters.jpg)

---

## 🎬 视频

- [主从遥操作打乒乓球](https://www.bilibili.com/video/BV1KprhBPE26/)
- [移植 LeRobot 数据集用于模仿学习](https://www.bilibili.com/video/BV1GLi1BqETz/)

---

## ⚠️ 免责声明

> [!WARNING]
> 如果您基于此仓库构建或开发 Panthera-HT，您将对由此给您自己或他人造成的所有身体和精神损害负全部责任。

> [!CAUTION]
> **掉电请扶好机械臂，防止其跌落。**

---

## 📝 许可证

本项目采用 [MIT 许可证](LICENSE)。

---

## 🤝 贡献

欢迎贡献！请随时提交问题和拉取请求。

---

## 📌 2026年10月更新说明

本版本的更新主要集中在 Python SDK 的机器人模型、参数配置和示例脚本：

- **支持可选夹爪**：新增 `gripper_installed` 配置，可在安装或未安装夹爪的单臂模式下使用；未安装夹爪时，SDK 会自动使用不包含夹爪分支的机器人模型。
- **更新 URDF 模型**：新增/完善夹爪自由度、末端执行器连接关系及 D405 相机坐标系，并同步调整相关惯性参数，为运动学、动力学和重力补偿提供更准确的模型。
- **完善参数配置**：更新 `Follower.yaml`、`Leader.yaml` 以及对应的电机参数文件，统一末端链接、夹爪和 URDF 配置项。
- **改进重力补偿与控制示例**：更新 `Panthera.py`、`2_gravity_compensation_control.py` 及相关控制脚本，使其能够根据夹爪配置加载对应模型并进行重力补偿。
- **新增测试和工具**：增加可选夹爪测试、MuJoCo 模型/可视化支持，以及 VR 控制启动和相关示例。

如需使用无夹爪模式，请检查对应机械臂的参数文件，将 `gripper_installed` 设置为 `false`；如需进行包含夹爪或相机负载的动力学计算，应同步维护 URDF 中相应 link 的质量、质心和惯性参数。

---
[![中文](https://img.shields.io/badge/lang-简体中文-brown.svg)](#中文) [![en](https://img.shields.io/badge/lang-English-blue.svg)](#english)
<div id="english"></div>

## English

**Panthera-HT SDK** is the official software development kit for the Panthera-HT six-axis robotic arm, providing both C++ and Python interfaces for motor control, kinematics, dynamics, and trajectory planning.

### ✨ Key Features

- **Multi-language Support**: Complete C++ and Python APIs
- **Rich Control Modes**: Position, velocity, torque, and hybrid control
- **Advanced Kinematics**: Forward and inverse kinematics with full Jacobian support
- **Dynamics Modeling**: Gravity compensation, friction compensation, and full dynamics
- **Trajectory Planning**: Quintic and septic polynomial interpolation
- **Master-Slave Control**: Bilateral teleoperation and trajectory recording/playback
- **Hardware Support**: Up to 7 CAN buses, 30 motors per bus

---

## 📁 Repository Structure

```
Panthera-HT_SDK/
├── panthera_cpp/              # C++ SDK
├── panthera_python/           # Python SDK
├── LICENSE                    # MIT License
└── README.md                  # This file
```

---

## 📦 Installation

For detailed installation instructions, please refer to the README in each language-specific directory:

- **C++ SDK**: See [panthera_cpp/README.md](panthera_cpp/README.md) for C++ dependencies, build instructions, and examples
- **Python SDK**: See [panthera_python/README.md](panthera_python/README.md) for Python dependencies, installation, and examples

---

## 🔗 Related Repositories

| Repository | License | Description |
|------------|---------|-------------|
| **[Panthera-HT_Host](https://github.com/HighTorque-Robotics/Panthera-HT_Host)** | [MIT](LICENSE) | The web-based host computer, built on the SDK repository, supports visual control of individual routines and integrates various SDK usage examples. |
| **[Panthera-HT_ROS2](https://github.com/HighTorque-Robotics/Panthera-HT-ROS2)** | [MIT](LICENSE) | ROS2 development package providing driver, control, and simulation support for the robotic arm. |
| **[Panthera-HT_lerobot](https://github.com/HighTorque-Robotics/Panthera-HT_lerobot)** | [MIT](LICENSE) | LeRobot integration package supporting imitation learning and robot learning algorithms. |
| **[Panthera-HT_Model](https://github.com/HighTorque-Robotics/Panthera-HT_Model)** | [MIT](LICENSE) | SolidWorks original design files, sheet metal drawings, 3D printing files, and Bill of Materials (BOM). |

---

## 📖 Documentation

- [C++ SDK Documentation](panthera_cpp/README.md)
- [Python SDK Documentation](panthera_python/README.md)
- [Robot Parameters](https://github.com/HighTorque-Robotics/Panthera-HT_Main/blob/main/images/parameters.jpg)

---

## 🎬 Videos

- [Master-Slave Teleoperation Playing Table Tennis](https://www.bilibili.com/video/BV1KprhBPE26/)
- [LeRobot Dataset Imitation Learning](https://www.bilibili.com/video/BV1GLi1BqETz/)

---

## ⚠️ Disclaimer

> [!WARNING]
> If you build or develop Panthera-HT based on this repository, you will be fully responsible for all physical and mental damages caused to you or others.

> [!CAUTION]
> **When power is lost, please support the robotic arm to prevent it from falling.**

---

## 📝 License

This project is licensed under the [MIT License](LICENSE).

---

## 🤝 Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.

---

<div align="center">
  <strong>Built with ❤️ by HighTorque Robotics</strong>
</div>

---

## 📌 October 2026 Update

This release mainly updates the Python SDK robot model, configuration files, and example scripts:

- **Optional gripper support**: Added the `gripper_installed` setting for single-arm operation with or without a gripper. When disabled, the SDK loads a robot model without the gripper branch.
- **URDF model updates**: Added or refined gripper degrees of freedom, end-effector links, and the D405 camera frame, together with corresponding inertial parameters for more accurate kinematics, dynamics, and gravity compensation.
- **Configuration updates**: Updated `Follower.yaml`, `Leader.yaml`, and related motor parameter files to align end-effector, gripper, and URDF settings.
- **Gravity-compensation and control examples**: Updated `Panthera.py`, `2_gravity_compensation_control.py`, and related scripts to load the model selected by the gripper configuration.
- **New tests and tools**: Added optional-gripper tests, MuJoCo model/visualization support, and VR-control launch examples.

To use gripper-free mode, set `gripper_installed` to `false` in the corresponding arm configuration. For dynamics calculations that include a gripper or camera payload, keep the mass, center of mass, and inertia of the relevant URDF links up to date.
