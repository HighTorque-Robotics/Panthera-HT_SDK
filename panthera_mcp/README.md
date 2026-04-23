# panthera-mcp

在 Codex 中通过自然语言控制 Panthera-HT 机械臂的 MCP 服务。

## 快速开始

### 1. 配置 codex

在 `~/.codex/config.toml` 中添加：

```toml
[mcp_servers.panthera-mcp]
transport = "stdio"
command = "/home/sunteng/anaconda3/envs/lerobot/bin/python3"
args = ["-m", "panthera_mcp"]
cwd = "/home/sunteng/Panthera-HT_SDK"
startup_timeout_sec = 10
```

### 2. 启动 codex

```bash
codex
```

## 使用方法

### 自然语言指令

| 你说 | AI 执行 |
|------|---------|
| "帮我拿杯子" | 执行 pick_up 轨迹 |
| "打个招呼" | 执行 greet 轨迹 |
| "握个手" | 执行 handshake 轨迹 |

### 基础控制

- `connect_robot` - 连接机械臂
- `get_robot_state` - 获取状态
- `go_home` / `go_zero` - 回零
- `open_gripper` / `close_gripper` - 夹爪控制

### 录制轨迹

1. `start_gravity_compensation` - 开启重力补偿
2. `start_trajectory_recording` - 开始录制
3. 手动移动机械臂
4. `stop_trajectory_recording` - 停止录制
5. `save_trajectory(name="动作名")` - 保存

### 播放轨迹

- `list_trajectories` - 列出所有轨迹
- `play_trajectory(name="动作名")` - 播放轨迹
- `delete_trajectory(name="动作名")` - 删除轨迹

## 文件结构

```
panthera_mcp/
├── server.py          # MCP 协议处理器
├── robot_service.py   # 业务逻辑代理
├── robot_daemon.py    # 守护进程（保持连接）
└── README.md          # 本文件
```

## 架构说明

```
codex → MCP Server → Socket → Daemon → Robot
                              ↓
                         持久连接
```

- **server.py**: 处理 MCP 协议请求
- **robot_service.py**: 转发请求到守护进程
- **robot_daemon.py**: 后台守护进程，持有机器人连接

## 守护进程管理

```bash
# 检查状态
python3 -m panthera_mcp.robot_daemon status

# 停止
python3 -m panthera_mcp.robot_daemon stop

# 手动启动（通常自动启动）
conda run -n lerobot python -m panthera_mcp.robot_daemon &
```

## 轨迹文件

保存在项目根目录的 `trajectories/` 文件夹中，格式为 `.jsonl`。

## 注意事项

- 机械臂默认已自动连接
- 有轨迹就直接执行，不需要确认
- 录制时请在重力补偿模式下手动移动机械臂
