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

## 🔗 相关仓库

| 仓库 | 许可证 | 说明 |
|------|--------|------|
| **[Panthera-HT_Main](https://github.com/HighTorque-Robotics/Panthera-HT_Main)** | [MIT](LICENSE) | 主项目仓库，包含项目介绍、仓库链接和功能请求。 |
| **[Panthera-HT_Model](https://github.com/HighTorque-Robotics/Panthera-HT_Model)** | [MIT](LICENSE) | SolidWorks原始设计文件、钣金图、3D打印文件和物料清单（BOM）。 |
| **[Panthera-HT_SDK](https://github.com/HighTorque-Robotics/Panthera-HT_SDK)** | [MIT](LICENSE) | Python SDK 开发包，提供快速上手的示例代码与开发工具链。 |
| **[Panthera-HT_ROS2](https://github.com/HighTorque-Robotics/Panthera-HT-ROS2)** | [MIT](LICENSE) | ROS2 开发包，提供机械臂的驱动、控制与仿真支持。 |
| **[Panthera-HT_lerobot](https://github.com/HighTorque-Robotics/Panthera-HT_lerobot)** | [MIT](LICENSE) | LeRobot 集成包，支持模仿学习和机器人学习算法。 |

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
| **[Panthera-HT_Main](https://github.com/HighTorque-Robotics/Panthera-HT_Main)** | [MIT](LICENSE) | Main project repository with project introduction, repository links, and feature requests. |
| **[Panthera-HT_Model](https://github.com/HighTorque-Robotics/Panthera-HT_Model)** | [MIT](LICENSE) | SolidWorks original design files, sheet metal drawings, 3D printing files, and Bill of Materials (BOM). |
| **[Panthera-HT_SDK](https://github.com/HighTorque-Robotics/Panthera-HT_SDK)** | [MIT](LICENSE) | Python SDK development package with quick-start example code and development toolchain. |
| **[Panthera-HT_ROS2](https://github.com/HighTorque-Robotics/Panthera-HT-ROS2)** | [MIT](LICENSE) | ROS2 development package providing driver, control, and simulation support for the robotic arm. |
| **[Panthera-HT_lerobot](https://github.com/HighTorque-Robotics/Panthera-HT_lerobot)** | [MIT](LICENSE) | LeRobot integration package supporting imitation learning and robot learning algorithms. |

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
