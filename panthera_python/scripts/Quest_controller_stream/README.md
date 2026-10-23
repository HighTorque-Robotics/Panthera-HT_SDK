# Quest Controller Stream

在 **Meta Quest 2 / Quest 3** 上通过 **WebXR** 读取手柄位姿、按钮和摇杆数据，并通过 **WebSocket -> UDP** 转发到电脑上的程序。本目录只负责数据转发，不解算运动学，也不向机械臂发送控制指令。

推荐优先使用桌面 UI 启动；命令行方式保留为备用方案。

本工程已包含在 Panthera-HT SDK 的 `panthera_python/scripts/Quest_controller_stream/` 中，无需单独克隆。以下启动命令均在本目录执行；从 SDK 仓库根目录进入：

```bash
cd panthera_python/scripts/Quest_controller_stream
```

机械臂端的安装和 VR 控制步骤见 [Panthera Python SDK 使用说明](../../README.md#quest-vr-遥操)。启动机械臂控制前，请停止本工程的 UDP 验证接收器（UI 的 Verify 或 `udp_receiver_test.py`），保持数据转发服务运行。

如果希望通过 USB 同时启动数据转发和遥操接收，推荐从 `panthera_python/scripts` 运行快速启动脚本 `7_vr_usb_start_quickly.py`。该脚本已经包含本目录的转发服务，使用时不要再运行 `quest_stream_ui.py` 或 `start_stream_server.py`：

```bash
cd panthera_python/scripts
./7_vr_usb_start_quickly.py                   # USB + 单臂
./7_vr_usb_start_quickly.py --controller dual # USB + 双臂
```

`7_vr_usb_start_quickly.py` 只支持 USB 有线连接。它会先结束 Quest Browser 中残留的旧 WebVR 会话，再打开带 `ws://localhost` 参数的页面，因此现有浏览器标签页会被关闭。启动后仍需在 Quest 页面点击 `Start XR Stream`。按 `Ctrl+C` 时，脚本会自动关闭 Quest Browser、停止本服务的转发输出，同时等待遥操脚本完成机械臂回零；重复按键不会跳过安全退出流程。使用 `--no-open-quest` 时，脚本不会打开或关闭浏览器。WiFi 连接请按本文后面的分步方式单独启动转发服务和遥操脚本。

---

# 目录

```text
quest_controller_stream/
├── quest_stream_ui.py       # 推荐入口：桌面 UI
├── start_stream_server.py   # 命令行入口：启动页面服务 + WS -> UDP 中继
├── start_http_server.sh     # 对命令行入口的 shell 包装
├── index.html               # Quest Browser 打开的 WebXR 页面
└── udp_receiver_test.py     # 本机 UDP 调试接收器
```

---

# 1. 启动前先安装环境

## 系统要求

* Meta Quest 2 / Quest 3
* Python 3.10+
* `adb`
* `openssl`

说明：

* `adb` 用于 USB 模式自动配置 `adb reverse`
* `openssl` 用于无线模式自动生成本地 HTTPS/WSS 证书

## Python 依赖

```bash
pip install websockets PyQt5
```

如果你只打算用命令行，不用 UI，`PyQt5` 不是必须。

## 检查 adb

```bash
adb devices
```

如果 USB 模式准备正常，你应该能看到 Quest 设备。

---

# 2. 推荐方式：用 UI 启动

启动 UI：

```bash
python3 quest_stream_ui.py
```

UI 支持：

* 选择 `USB` / `WiFi`
* 修改 `UDP`、`HTTP`、`WS` 参数
* 启动 / 停止服务
* 验证本机 UDP 接收
* 实时查看左右手：
  * `position`
  * `quaternion`
  * `thumbstick`
  * `trigger`
  * `squeeze`
  * 按钮状态
* 自动保存参数到：

```text
.quest_stream_ui.json
```

## 各项参数

* `Mode`
  * `USB`：有线模式，自动尝试 `adb reverse`
  * `WiFi`：无线模式，自动走 `HTTPS/WSS`
* `UDP IP`
  * 数据最终要转发到的目标 IP
  * 本机调试通常用 `127.0.0.1`
* `UDP Port`
  * 数据最终要转发到的目标端口
  * 默认 `5005`
* `HTTP Host`
  * 网页服务监听地址
  * 默认 `0.0.0.0`
* `HTTP Port`
  * Quest 浏览器打开页面时使用的端口
  * 默认 `8080`
* `WS Host`
  * WebSocket 服务监听地址
  * 默认 `0.0.0.0`
* `WS Port`
  * 页面把手柄数据发回电脑时使用的端口
  * 默认 `8765`
* `Force HTTPS/WSS`
  * 强制启用安全上下文
  * `WiFi` 模式下默认会自动开启
* `Cert File`
  * HTTPS 证书文件路径
  * 默认 `.certs/quest-controller.crt`
* `Key File`
  * HTTPS 私钥文件路径
  * 默认 `.certs/quest-controller.key`
* `Verify Port`
  * UI 本地验证 UDP 接收时监听的端口
  * 通常和 `UDP Port` 保持一致

## UI 里最常见的两种用法

### USB 模式

1. 选择 `Mode = USB`
2. `UDP IP = 127.0.0.1`
3. `UDP Port = 5005`
4. 点击 `Start Server`
5. 在 Quest Browser 打开 UI 里显示的 `USB URL`
6. 点击网页里的 `Start XR Stream`
7. 如需验证，点击 `Start Verify`

### WiFi 模式

1. 选择 `Mode = WiFi`
2. 确保 Quest 和电脑在同一个局域网
3. `UDP IP = 127.0.0.1` 或你自己的接收端 IP
4. 点击 `Start Server`
5. 在 Quest Browser 打开 UI 里显示的 `WiFi URL`
6. 第一次如果有证书警告，先接受，再重新打开
7. 点击网页里的 `Start XR Stream`
8. 如需验证，点击 `Start Verify`

---

# 3. 命令行启动方式

如果你不想开 UI，可以直接用命令行。

## 最简单启动

```bash
python3 start_stream_server.py
```

或：

```bash
./start_http_server.sh
```

它会同时：

* 启动 `index.html` 页面服务
* 启动 `WebSocket -> UDP` 中继
* USB 模式下自动尝试执行 `adb reverse`
* 打印 USB 和 WiFi 地址

默认 UDP 目标：

```text
127.0.0.1:5005
```

如需本机验证：

```bash
python3 udp_receiver_test.py
```

---

# 4. 命令行参数怎么改

## 最常改的参数

* `--udp-ip`
  * UDP 转发目标 IP
  * 默认 `127.0.0.1`
* `--udp-port`
  * UDP 转发目标端口
  * 默认 `5005`
* `--http-port`
  * 网页服务端口
  * 默认 `8080`
* `--ws-port`
  * WebSocket 服务端口
  * 默认 `8765`
* `--adb auto`
  * 默认模式
  * 自动尝试 `adb reverse`
* `--adb never`
  * 无线模式
  * 不尝试 `adb`
  * 会默认自动启用 `HTTPS/WSS`
* `--adb always`
  * 强制要求 `adb reverse` 成功，否则退出
* `--https`
  * 显式启用 `HTTPS/WSS`
* `--cert-file`
  * HTTPS 证书路径
  * 默认 `.certs/quest-controller.crt`
* `--key-file`
  * HTTPS 私钥路径
  * 默认 `.certs/quest-controller.key`

## 常用命令示例

### USB 模式

```bash
python3 start_stream_server.py
```

### WiFi 模式

```bash
python3 start_stream_server.py --adb never
```

### 改 UDP 目标

```bash
python3 start_stream_server.py --udp-ip 192.168.1.50 --udp-port 5005
```

### 端口冲突时改端口

```bash
python3 start_stream_server.py --http-port 8081 --ws-port 8766
```

### USB 模式强制要求 adb 成功

```bash
python3 start_stream_server.py --adb always
```

---

# 5. USB 和 WiFi 分别怎么工作

## USB 模式

Quest 访问：

```text
http://localhost:8080/index.html
```

电脑端通过 `adb reverse` 把 Quest 里的 `localhost` 反向映射到本机服务。

推荐条件：

1. Quest 开启开发者模式
2. USB 连接 Quest 和电脑
3. 头显内允许 `USB debugging`
4. `adb devices` 能看到设备

## WiFi 模式

Quest 访问：

```text
https://电脑局域网IP:8080/index.html?ws=wss://电脑局域网IP:8765
```

无线模式默认使用 `HTTPS/WSS`，因为 WebXR 往往需要安全上下文。

---

# 6. 证书文件是什么

无线模式会用到两个文件：

* `.certs/quest-controller.crt`
* `.certs/quest-controller.key`

含义：

* `.crt`：证书文件
* `.key`：私钥文件

作用：

* 让页面从 `http://` 变成 `https://`
* 让 WebSocket 从 `ws://` 变成 `wss://`

如果文件不存在，脚本会自动生成自签名证书。通常不需要手动改；只有你已经有自己的证书时，才需要改路径。

注意：

* 自签名证书第一次访问时，Quest Browser 可能会提示证书警告
* 需要你先手动接受一次，再重新打开页面

---

# 7. 启动后应该打开哪个地址

启动后会打印这些地址：

* `USB access`
* `USB access with ws param`
* `WiFi access`
* `WiFi access with ws param`

建议：

* USB 模式：优先打开 `USB access with ws param`
* WiFi 模式：优先打开 `WiFi access with ws param`

带 `ws` 参数的地址会覆盖网页上次保存在浏览器里的连接地址。从 WiFi
切换到 USB 时，应确认网页中的 WebSocket 地址为
`ws://localhost:8765`，而不是上次使用的 `wss://电脑局域网IP:8765`。

---

# 8. 本机验证接收

命令行验证：

```bash
python3 udp_receiver_test.py
```

它会打印：

* 左右手位置
* 左右手四元数
* 欧拉角
* 摇杆
* trigger / squeeze
* 按钮状态

如果使用 UI，也可以直接点击 `Start Verify` 在界面里看最新数据。

---

# 9. 常见问题

## `navigator.xr 不存在`

常见原因：

* 不是在 Quest Browser 里打开
* 没有进入安全上下文
* 无线模式地址不对

处理：

* 先试 USB 模式
* 或无线用：

```bash
python3 start_stream_server.py --adb never
```

## `Address already in use`

端口被占用了。

处理：

* 关掉上一次启动的服务
* 或换端口：

```bash
python3 start_stream_server.py --http-port 8081 --ws-port 8766
```

## `Missing dependency: websockets`

安装：

```bash
pip install websockets
```

## `ModuleNotFoundError: No module named 'PyQt5'`

安装：

```bash
pip install PyQt5
```
