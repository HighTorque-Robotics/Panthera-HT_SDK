"""Linux 键盘监听后端：Wayland 使用 evdev，X11 使用 pynput。"""

import glob
import os
import threading


KEY_MAP = {
    'KEY_W': 'w', 'KEY_S': 's', 'KEY_A': 'a', 'KEY_D': 'd',
    'KEY_Q': 'q', 'KEY_E': 'e',
    'KEY_1': '1', 'KEY_2': '2', 'KEY_3': '3',
    'KEY_4': '4', 'KEY_5': '5', 'KEY_6': '6',
    'KEY_ESC': 'esc',
}


class EvdevKeyboardListener:
    """直接读取 Linux input event，不依赖 Wayland 全局快捷键协议。"""

    def __init__(self, on_press, on_release, on_failure, device_path=None,
                 handle_repeats=False):
        try:
            import evdev
        except ImportError as exc:
            raise RuntimeError(
                "Wayland 键盘监听需要 evdev，请执行: pip install evdev"
            ) from exc

        self.evdev = evdev
        self.on_press = on_press
        self.on_release = on_release
        self.on_failure = on_failure
        self.handle_repeats = handle_repeats
        self.debug = os.environ.get('PANTHERA_KEYBOARD_DEBUG') == '1'
        self.devices = self._open_keyboards(device_path)
        self.stop_event = threading.Event()
        self.threads = []

    def _open_keyboards(self, device_path):
        evdev = self.evdev
        required_keys = {
            evdev.ecodes.KEY_W, evdev.ecodes.KEY_A,
            evdev.ecodes.KEY_S, evdev.ecodes.KEY_D,
            evdev.ecodes.KEY_ESC,
        }
        paths = ([device_path] if device_path else
                 sorted(glob.glob('/dev/input/event*')))
        permission_denied = []
        candidates = []

        for path in paths:
            try:
                device = evdev.InputDevice(path)
                key_codes = set(
                    device.capabilities(absinfo=False).get(evdev.ecodes.EV_KEY, [])
                )
                if required_keys.issubset(key_codes):
                    candidates.append(device)
                else:
                    device.close()
            except PermissionError:
                permission_denied.append(path)
            except OSError:
                continue

        if candidates:
            # 同时监听所有完整键盘。系统可能同时有内置键盘、USB
            # 键盘和无线接收器，只选第一个会造成按键无响应。
            return candidates

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
        devices = ", ".join(
            f"{device.path}: {device.name}" for device in self.devices
        )
        return f"evdev ({devices})"

    def start(self):
        for index, device in enumerate(self.devices):
            thread = threading.Thread(
                target=self._run,
                args=(device,),
                name=f"panthera-evdev-keyboard-{index}",
                daemon=True,
            )
            self.threads.append(thread)
            thread.start()

    def _run(self, device):
        try:
            for event in device.read_loop():
                if self.stop_event.is_set():
                    break
                if event.type != self.evdev.ecodes.EV_KEY:
                    continue

                key_code = self.evdev.ecodes.KEY.get(event.code)
                if isinstance(key_code, list):
                    key_code = key_code[0]
                key_name = KEY_MAP.get(key_code)
                if key_name is None:
                    continue

                if self.debug:
                    event_names = {0: 'release', 1: 'press', 2: 'repeat'}
                    print(
                        f"\n键盘事件 [{device.path}]: "
                        f"{key_name} {event_names.get(event.value, event.value)}",
                        flush=True,
                    )

                if event.value == 1 or (event.value == 2 and self.handle_repeats):
                    self.on_press(key_name)
                elif event.value == 0:
                    if self.on_release(key_name) is False:
                        break
        except OSError as exc:
            if not self.stop_event.is_set():
                self.on_failure(exc)
        except Exception as exc:
            self.on_failure(exc)

    def is_alive(self):
        return bool(self.threads) and any(thread.is_alive() for thread in self.threads)

    def stop(self):
        self.stop_event.set()
        for device in self.devices:
            device.close()
        current_thread = threading.current_thread()
        for thread in self.threads:
            if thread is not current_thread:
                thread.join(timeout=0.5)


class PynputKeyboardListener:
    """X11 下的 pynput 兼容后端。"""

    def __init__(self, on_press, on_release):
        try:
            from pynput import keyboard
        except ImportError as exc:
            raise RuntimeError(
                "X11 键盘监听需要 pynput，请执行: pip install pynput"
            ) from exc

        self.keyboard = keyboard
        self.on_press = on_press
        self.on_release = on_release
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
            self.on_press(key_name)

    def _on_release(self, key):
        if key == self.keyboard.Key.esc:
            return self.on_release('esc')
        key_name = self._character(key)
        if key_name:
            return self.on_release(key_name)
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


def create_keyboard_listener(on_press, on_release, on_failure,
                             handle_repeats=False):
    """根据当前图形会话选择键盘后端。"""
    backend = os.environ.get('PANTHERA_KEYBOARD_BACKEND', '').strip().lower()
    session_type = os.environ.get('XDG_SESSION_TYPE', '').strip().lower()
    device_path = os.environ.get('PANTHERA_KEYBOARD_DEVICE')

    if backend and backend not in {'evdev', 'pynput'}:
        raise RuntimeError(
            "PANTHERA_KEYBOARD_BACKEND 只能设为 evdev 或 pynput"
        )
    if backend == 'evdev' or (not backend and session_type == 'wayland'):
        return EvdevKeyboardListener(
            on_press, on_release, on_failure, device_path, handle_repeats
        )
    return PynputKeyboardListener(on_press, on_release)
