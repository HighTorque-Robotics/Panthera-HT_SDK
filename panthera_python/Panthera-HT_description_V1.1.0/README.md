# Panthera-HT 机器人模型 V1.1.0

本目录提供 Panthera-HT 六自由度机械臂的 URDF、外观与碰撞网格，以及 MuJoCo 可视化脚本，可用于机器人模型展示、仿真和 RoboTwin 中的仿真数据采集。

## 使用说明入口

- [URDF 配置说明](urdf/README.md)：手动切换固定夹爪与可动夹爪、配置右夹爪 mimic 跟随，以及启用 D405 相机坐标系。
- [MuJoCo 使用说明](mujoco/README.md)：安装 Python 依赖、启动可视化、鼠标拖动施力、模型验证与截图。

## 外观渲染与仿真

我们使用 **GLB 网格进行外观渲染**，文件位于 `meshes/visual/`，用于呈现机械臂和夹爪的几何外形及材质配色。URDF 的 `<visual>` 引用这些 GLB 文件；`<collision>` 则引用 `meshes/stl/` 中的 STL 网格进行碰撞描述。

该模型可用于 RoboTwin 的仿真数据采集。接入时，根据采集任务选择是否启用夹爪自由度和 D405 相机坐标系，并在 RoboTwin 侧配置机器人加载路径、关节控制、相机和采集任务。请保留模型目录结构，使 URDF 中的 `package://Panthera-HT_description_V1.1.0/...` 网格路径能够被正确解析。本目录提供机器人模型资源，具体数据采集流程由 RoboTwin 侧配置。

MuJoCo 使用配套脚本将源网格转换为 OBJ，再生成仿真场景。可视化效果如下：

![Panthera-HT MuJoCo 可视化效果](mujoco/Panthera-HT_mujoco.png)

## URDF 内容与配置

源模型分为 [Follower URDF](urdf/Panthera-HT_description_V1.1.0_follower.urdf) 和 [Leader URDF](urdf/Panthera-HT_description_V1.1.0_leader.urdf)，均包含连杆惯性、视觉与碰撞模型、关节连接关系、运动轴和限位。MuJoCo 可视化脚本默认使用 Follower URDF。

| 部分 | 内容与配置方式 |
| --- | --- |
| 机械臂 | `joint1` 至 `joint6`，共六个旋转关节 |
| 夹爪基体 | 通过 `gripper_center_joint` 固定连接到 `link6` |
| 左右夹爪 | 默认使用 `fixed`；需要开合运动时，将左右关节同时切换为 `prismatic` |
| 夹爪跟随 | 启用右侧的 `mimic` 配置后，`gripper_R_joint` 以 `-1` 倍位移跟随 `gripper_L_joint` |
| D405 相机 | 默认注释；取消 `camera_link` 与 `camera_joint` 整块注释后，启用相机模型及相对 `link6` 的固定坐标关系 |

夹爪与相机选项可独立组合。具体代码修改位置和截图见 [URDF 配置说明](urdf/README.md)。启用相机连杆仅增加模型及坐标关系，相机图像采集仍需在使用端配置。

`urdf/` 中还保留了 CSV 参数文件。修改运行模型时，请以源 URDF 为准；MuJoCo 脚本从源 URDF 生成模型，不读取 CSV。`mujoco/robot.urdf` 和 `mujoco/scene.xml` 为生成文件，重新运行脚本会覆盖，不应作为参数修改入口。

## 目录结构

```text
Panthera-HT_description_V1.1.0/
├── README.md                # 模型总览与使用说明入口
├── urdf/                    # 源 URDF、CSV 参数及配置说明
├── meshes/
│   ├── visual/              # GLB 外观网格
│   └── stl/                 # STL 碰撞网格
├── mujoco/                  # MuJoCo 脚本、使用说明、生成模型和网格
├── image/                   # URDF 配置说明截图
├── config/                  # 关节名称配置
├── launch/                  # ROS 启动文件
├── CMakeLists.txt           # ROS 包构建配置
└── package.xml              # ROS 包描述
```
