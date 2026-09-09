#!/usr/bin/env python3
"""
机械臂笛卡尔空间速度控制程序（基于雅可比矩阵）

功能：
    1. 启动时机械臂缓慢移动到安全位置
    2. 使用键盘控制末端效应器速度（按住时持续运动）
    3. 使用阻尼伪逆避免雅可比奇异性
    4. 实时显示当前末端位置和速度

键盘控制：
    W/S: 工具坐标系X轴线速度 (+/-)
    A/D: 工具坐标系Y轴线速度 (+/-)
    Q/E: 工具坐标系Z轴线速度 (+/-)
    1/2: 绕工具坐标系X轴角速度 (+/-)
    3/4: 绕工具坐标系Y轴角速度 (+/-)
    5/6: 绕工具坐标系Z轴角速度 (+/-)
    ESC: 退出程序

注意事项：
    - 按住按键时末端持续运动，松开后停止
    - 使用阻尼伪逆避免奇异性，运动更平滑
    - 接近工作空间边界时会自动减速
"""

import glob
import os
import sys
import threading
import time
import numpy as np
from Panthera_lib import Panthera
from Panthera_lib.keyboard_listener import create_keyboard_listener as create_platform_keyboard_listener

# 全局变量
end_velocity = np.zeros(6)  # 目标速度 [vx, vy, vz, ωx, ωy, ωz]
actual_velocity = np.zeros(6)  # 实际速度（平滑后）
linear_speed = 0.3  # 线速度 m/s
angular_speed = 2.0  # 角速度 rad/s
running = True
keys_pressed = set()  # 记录当前按下的键
keys_lock = threading.Lock()

# hightorque_robot 1.2.0 的 C++ 析构函数会先关串口再等待接收线程，
# 导致退出时可能因 EBADF 而 abort。主程序直接运行时保持对象存活，
# 完成显式停止/刹车后由操作系统回收资源，避免触发有缺陷的析构路径。
robot_keepalive = None
safe_shutdown_complete = False

# 加减速参数
acceleration_factor = 0.02  # 加速因子（0-1，值越大加速越快）
                            # 0.15 约需 0.2-0.3 秒达到目标速度

CONTROL_KEYS = {'w', 's', 'a', 'd', 'q', 'e', '1', '2', '3', '4', '5', '6'}


def update_end_velocity():
    """根据当前按键集合重新计算目标速度。

    重新计算可以正确处理同一轴上两个方向键同时按下的情况。
    """
    global end_velocity

    end_velocity[0] = linear_speed * (('w' in keys_pressed) - ('s' in keys_pressed))
    end_velocity[1] = linear_speed * (('a' in keys_pressed) - ('d' in keys_pressed))
    end_velocity[2] = linear_speed * (('q' in keys_pressed) - ('e' in keys_pressed))
    end_velocity[3] = angular_speed * (('1' in keys_pressed) - ('2' in keys_pressed))
    end_velocity[4] = angular_speed * (('3' in keys_pressed) - ('4' in keys_pressed))
    end_velocity[5] = angular_speed * (('5' in keys_pressed) - ('6' in keys_pressed))


def on_press(key_name):
    """键盘按下事件处理"""
    if key_name in CONTROL_KEYS:
        with keys_lock:
            keys_pressed.add(key_name)
            update_end_velocity()


def on_release(key_name):
    """键盘释放事件处理"""
    global end_velocity, running, keys_pressed

    if key_name == 'esc':
        print("\n检测到ESC键，准备退出...")
        running = False
        return False  # 停止监听

    if key_name in CONTROL_KEYS:
        with keys_lock:
            keys_pressed.discard(key_name)
            update_end_velocity()

    return True


def keyboard_listener_failed(exc):
    """键盘监听中断时立即让控制循环进入停止流程。"""
    global running, end_velocity

    with keys_lock:
        keys_pressed.clear()
        end_velocity[:] = 0.0
    running = False
    print(f"\n键盘监听中断: {exc}")


class EvdevKeyboardListener:
    """使用 Linux evdev 监听键盘，适用于 Wayland。"""

    KEY_MAP = {
        'KEY_W': 'w', 'KEY_S': 's', 'KEY_A': 'a', 'KEY_D': 'd',
        'KEY_Q': 'q', 'KEY_E': 'e',
        'KEY_1': '1', 'KEY_2': '2', 'KEY_3': '3',
        'KEY_4': '4', 'KEY_5': '5', 'KEY_6': '6',
        'KEY_ESC': 'esc',
    }

    def __init__(self, device_path=None):
        try:
            import evdev
        except ImportError as exc:
            raise RuntimeError(
                "Wayland 键盘监听需要 evdev，请执行: pip install evdev"
            ) from exc

        self.evdev = evdev
        self.device = self._open_keyboard(device_path)
        self.stop_event = threading.Event()
        self.thread = None

    def _open_keyboard(self, device_path):
        evdev = self.evdev
        required_keys = {
            evdev.ecodes.KEY_W, evdev.ecodes.KEY_A,
            evdev.ecodes.KEY_S, evdev.ecodes.KEY_D,
            evdev.ecodes.KEY_ESC,
        }

        if device_path:
            paths = [device_path]
        else:
            # evdev.list_devices() 会隐藏无权限的设备，直接枚举才能给出明确的权限提示。
            paths = sorted(glob.glob('/dev/input/event*'))

        permission_denied = []
        candidates = []
        for path in paths:
            try:
                device = evdev.InputDevice(path)
                key_codes = set(device.capabilities().get(evdev.ecodes.EV_KEY, []))
                if required_keys.issubset(key_codes):
                    candidates.append(device)
                else:
                    device.close()
            except PermissionError:
                permission_denied.append(path)
            except OSError:
                continue

        if candidates:
            # 多个键盘时可用环境变量显式选择。
            selected = candidates[0]
            for device in candidates[1:]:
                device.close()
            return selected

        if permission_denied:
            raise RuntimeError(
                "无权读取键盘输入设备 " + ", ".join(permission_denied) + "。\n"
                "请将当前用户加入 input 组后注销并重新登录：\n"
                "  sudo usermod -aG input $USER"
            )

        if device_path:
            raise RuntimeError(f"指定的设备不是可用键盘: {device_path}")
        raise RuntimeError(
            "未找到可读取的键盘 /dev/input/event*。\n"
            "可设置 PANTHERA_KEYBOARD_DEVICE=/dev/input/eventX 指定键盘。"
        )

    @property
    def description(self):
        return f"evdev ({self.device.path}, {self.device.name})"

    def start(self):
        self.thread = threading.Thread(
            target=self._run,
            name="panthera-evdev-keyboard",
            daemon=True,
        )
        self.thread.start()

    def _run(self):
        try:
            for event in self.device.read_loop():
                if self.stop_event.is_set():
                    break
                if event.type != self.evdev.ecodes.EV_KEY:
                    continue

                key_code = self.evdev.ecodes.KEY.get(event.code)
                # 某些 evdev 版本可能返回别名列表，优先使用第一个。
                if isinstance(key_code, list):
                    key_code = key_code[0]
                key_name = self.KEY_MAP.get(key_code)
                if key_name is None:
                    continue

                if event.value == 1:      # key down
                    on_press(key_name)
                elif event.value == 0:    # key up
                    if on_release(key_name) is False:
                        break
                # event.value == 2 是键盘自动重复，按下状态已经记录，无需处理。
        except OSError as exc:
            if not self.stop_event.is_set():
                keyboard_listener_failed(exc)
        except Exception as exc:
            keyboard_listener_failed(exc)

    def is_alive(self):
        return self.thread is not None and self.thread.is_alive()

    def stop(self):
        self.stop_event.set()
        self.device.close()
        if self.thread is not None and self.thread is not threading.current_thread():
            self.thread.join(timeout=0.5)


class PynputKeyboardListener:
    """使用 pynput 监听键盘，作为 X11 环境的兼容后端。"""

    def __init__(self):
        try:
            from pynput import keyboard
        except ImportError as exc:
            raise RuntimeError(
                "X11 键盘监听需要 pynput，请执行: pip install pynput"
            ) from exc

        self.keyboard = keyboard
        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )

    @staticmethod
    def _character(key):
        char = getattr(key, 'char', None)
        return char.lower() if char else None

    def _on_press(self, key):
        key_name = self._character(key)
        if key_name:
            on_press(key_name)

    def _on_release(self, key):
        if key == self.keyboard.Key.esc:
            return on_release('esc')
        key_name = self._character(key)
        if key_name:
            return on_release(key_name)
        return True

    @property
    def description(self):
        return "pynput (X11)"

    def start(self):
        self.listener.start()
        self.listener.wait()

    def is_alive(self):
        return self.listener.is_alive()

    def stop(self):
        self.listener.stop()


def create_keyboard_listener():
    """通过共享后端监听所有键盘，速度控制不处理自动重复事件。"""
    return create_platform_keyboard_listener(
        on_press,
        on_release,
        keyboard_listener_failed,
        handle_repeats=False,
    )


def brake_robot(robot):
    """显式发送刹车命令，确保绕过 SDK 析构函数前电机已进入安全状态。"""
    for motor in robot.Motors:
        motor.brake()
    for _ in range(3):
        robot.motor_send_cmd()
        time.sleep(0.01)


def move_to_safe_position(robot):
    """启动时缓慢移动到安全位置"""
    print("\n" + "="*60)
    print("正在移动到安全位置...")
    print("="*60)

    # 定义安全位置（关节空间）
    safe_joint_pos = [0.0, 0.5, 0.6, 0.0, 0.0, 0.0]

    # 先刷新状态
    robot.send_get_motor_state_cmd()
    robot.motor_send_cmd()
    time.sleep(0.3)

    # 缓慢移动到安全位置（3秒）
    print("移动中...")
    success = robot.Joint_Pos_Vel(safe_joint_pos, [0.5]*robot.motor_count, iswait=True)

    if success:
        print("✓ 已到达安全位置")
    else:
        print("✗ 移动到安全位置失败")
        return False

    time.sleep(0.5)
    return True

def main():
    global end_velocity, running, robot_keepalive, safe_shutdown_complete

    print("="*60)
    print("机械臂笛卡尔空间速度控制程序（基于雅可比矩阵）")
    print("="*60)

    # 在机械臂运动前先检查键盘后端和权限，避免无法键盘停止。
    listener = create_keyboard_listener()
    print(f"\n键盘后端: {listener.description}")

    # 初始化机械臂
    print("\n初始化机械臂...")
    robot = Panthera()
    if __name__ == "__main__":
        robot_keepalive = robot

    # 检查是否有 Pinocchio 模型
    if robot.model is None:
        print("错误：未找到 Pinocchio 模型，无法计算雅可比矩阵")
        return

    # 移动到安全位置
    if not move_to_safe_position(robot):
        print("初始化失败，退出程序")
        return

    # 获取当前位姿
    current_fk = robot.forward_kinematics()
    current_pos = current_fk['position']

    print(f"\n初始位置: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}] m")
    print("\n" + "="*60)
    print("键盘控制说明（按住时持续运动）：")
    print("  W/S: 工具坐标系X轴线速度 (+/-)")
    print("  A/D: 工具坐标系Y轴线速度 (+/-)")
    print("  Q/E: 工具坐标系Z轴线速度 (+/-)")
    print("  1/2: 绕工具坐标系X轴角速度 (+/-)")
    print("  3/4: 绕工具坐标系Y轴角速度 (+/-)")
    print("  5/6: 绕工具坐标系Z轴角速度 (+/-)")
    print("  ESC: 退出程序")
    print("="*60)
    print("\n开始控制，请小心操作！\n")

    # 启动键盘监听
    listener.start()

    # 控制参数
    control_rate = 0.01  # 100Hz 控制频率
    damping_base = 0.01  # 基础阻尼系数
    damping_adaptive = True  # 自适应阻尼

    # MIT 控制参数
    kp = [0.0] * robot.motor_count  # 位置增益为0（纯速度控制）
    kd = [10.0, 15.0, 15.0, 10.0, 5.0, 5.0]  # 阻尼增益

    # 关节速度限制
    max_joint_vel = np.array([2.0, 2.0, 2.0, 2.0, 3.0, 3.0])  # rad/s

    try:
        while running:
            if not listener.is_alive():
                keyboard_listener_failed("监听线程已退出")
                break

            # 获取当前关节角度
            q = robot.get_current_pos()

            # 避免在键盘线程更新六个分量时读到中间状态。
            with keys_lock:
                commanded_end_velocity = end_velocity.copy()

            # 计算雅可比矩阵
            try:
                J = robot.get_jacobian(q)
            except Exception as e:
                print(f"\r雅可比计算失败: {e}                    ", end='')
                time.sleep(control_rate)
                continue

            # 计算可操作度
            manipulability = robot.get_manipulability(q)

            # 自适应阻尼：接近奇异时增大阻尼
            if damping_adaptive:
                # 可操作度阈值
                manip_threshold = 0.01
                if manipulability < manip_threshold:
                    # 接近奇异，增大阻尼并减速
                    damping = damping_base * (1.0 + (manip_threshold - manipulability) * 100)
                    speed_scale = manipulability / manip_threshold
                    end_velocity_scaled = commanded_end_velocity * speed_scale
                else:
                    damping = damping_base
                    end_velocity_scaled = commanded_end_velocity
            else:
                damping = damping_base
                end_velocity_scaled = commanded_end_velocity

            # 计算阻尼伪逆
            J_damp = Panthera.compute_damped_pseudoinverse(J, damping)

            # === 速度平滑（加减速） ===
            # 使用指数平滑让实际速度逐渐接近目标速度
            global actual_velocity
            actual_velocity = actual_velocity + (end_velocity_scaled - actual_velocity) * acceleration_factor

            # 使用平滑后的速度
            end_velocity_to_use = actual_velocity

            # 获取当前末端位置和姿态
            current_fk = robot.forward_kinematics()
            current_pos = current_fk['position']
            R_tool = current_fk['rotation']  # 工具坐标系的旋转矩阵

            # 将线速度和角速度从工具坐标系转换到世界坐标系
            # v_world = R_tool @ v_tool
            # ω_world = R_tool @ ω_tool
            end_velocity_world = np.zeros(6)
            end_velocity_world[0:3] = R_tool @ end_velocity_to_use[0:3]  # 线速度转换
            end_velocity_world[3:6] = R_tool @ end_velocity_to_use[3:6]  # 角速度转换

            # 计算关节速度
            q_dot = J_damp @ end_velocity_world

            # 限制关节速度
            q_dot = np.clip(q_dot, -max_joint_vel, max_joint_vel)

            robot.Joint_Vel(q_dot)

            # 显示信息
            vel_norm = np.linalg.norm(actual_velocity[:3])  # 使用实际速度
            print(f"\r位置: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}] | "
                  f"速度: {vel_norm:.3f} m/s | 可操作度: {manipulability:.4f} | 阻尼: {damping:.4f}", end='')

            time.sleep(control_rate)

    except KeyboardInterrupt:
        print("\n\n程序被中断")
    finally:
        listener.stop()
        # 停止运动
        print("\n\n停止运动...")
        q = robot.get_current_pos()
        gra = robot.get_Gravity()
        robot.pos_vel_tqe_kp_kd(q, [0.0]*robot.motor_count, gra, kp, kd)
        time.sleep(0.5)

        print("返回零位...")
        zero_pos = [0.0] * robot.motor_count
        robot.Joint_Pos_Vel(zero_pos, [0.5]*robot.motor_count, iswait=True)
        print("所有电机已停止")

        # 在绕过 SDK 1.2.0 有缺陷的 C++ 析构函数前，显式让所有电机进入刹车模式。
        brake_robot(robot)
        print("所有电机已刹车")
        safe_shutdown_complete = True

if __name__ == "__main__":
    exit_code = 0
    try:
        main()
    except Exception as e:
        exit_code = 1
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if safe_shutdown_complete:
            # os._exit 不执行 Python/C++ 析构函数；只在显式停止和刹车都完成后使用。
            sys.stdout.flush()
            sys.stderr.flush()
            os._exit(exit_code)
