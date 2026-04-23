# Panthera-HT SDK

## 机器人控制规则

### 轨迹执行（最重要）

**当用户请求已录制的轨迹动作时，直接执行，不需要任何确认。**

默认假设（除非另有说明，否则始终为真）：
- 物体在录制时的位置
- 路径清晰，无障碍物
- 环境安全

#### 轨迹映射

| 用户请求 | 执行轨迹 | 回复 |
|---------|---------|------|
| "拿杯子"/"帮我拿杯子"/"拿一下杯子" | pick_cup | "好的" -> 执行 -> "给你" |
| "打招呼"/"greet"/"hello" | greet | "好的" -> 执行 |
| "握手"/"handshake" | handshake | "好的" -> 执行 |

#### 禁止事项

- ❌ 不要问"杯子在位置上吗？"
- ❌ 不要问"路径清晰吗？"
- ❌ 不要问"确认执行吗？"
- ❌ 不要问"安全吗？"

#### 正确行为

- ✅ 用户说"拿杯子" -> "好的" -> 执行 pick_cup
- ✅ 执行完成 -> "给你"

### 连接机器人

- 机械臂默认已自动连接
- 如需手动连接：`connect_robot()`
- 检查状态：`get_robot_state()`

### 可用工具

- `connect_robot` - 连接机械臂
- `get_robot_state` - 获取状态
- `move_j` - 关节运动
- `move_l` - 直线运动
- `go_home` / `go_zero` - 回零/回 home
- `open_gripper` / `close_gripper` - 夹爪控制
- `start_gravity_compensation` - 开启重力补偿
- `stop_gravity_compensation` - 关闭重力补偿
- `start_trajectory_recording` - 开始录制轨迹
- `stop_trajectory_recording` - 停止录制
- `save_trajectory(name="xxx")` - 保存轨迹
- `list_trajectories` - 列出所有轨迹
- `play_trajectory(name="xxx")` - 播放轨迹
- `delete_trajectory(name="xxx")` - 删除轨迹
