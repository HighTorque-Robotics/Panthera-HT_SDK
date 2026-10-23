#!/usr/bin/env python3
"""PyQt5 轨迹记录/循环回放界面。

安装界面依赖：python -m pip install PyQt5
运行：python scripts/5_trajectory_gui.py
机械臂依赖与原记录/回放脚本相同；仅打开界面不会连接机械臂。
"""
import json
import bisect
import math
import queue
import sys
import tempfile
import threading
import time
from datetime import datetime
from functools import lru_cache
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets

SCRIPT_DIR = Path(__file__).resolve().parent
TRAJECTORY_DIR = SCRIPT_DIR / "trajectory"
CONFIG_PATH = SCRIPT_DIR.parent / "robot_param" / "Follower.yaml"
KP = [30.0, 40.0, 55.0, 15.0, 7.0, 5.0]
KD = [3.0, 4.0, 5.5, 1.5, 0.7, 0.5]
TAU_LIMIT = [15.0, 30.0, 30.0, 15.0, 5.0, 5.0]
CONTROL_DT = 0.002
PLAYBACK_RESAMPLE_DT = 0.01
PLAYBACK_SMOOTH_WINDOW = 7
TRANSITION_LIMIT_SCALE = 1.1


@lru_cache(maxsize=4)
def load_transition_limits(path, modified_ns):
    """按配置版本缓存，避免重复读取 YAML；错误配置不回退到静默默认值。"""
    import yaml
    with Path(path).open(encoding="utf-8") as stream:
        robot = yaml.safe_load(stream)["robot"]
    limits = []
    for key in ("velocity_limits", "acceleration_limits"):
        values = [float(value) for value in robot[key]]
        if len(values) != 6 or any(not math.isfinite(v) or v <= 0 for v in values):
            raise ValueError(f"YAML robot.{key} 必须是 6 个有限正数")
        limits.append(tuple(v * TRANSITION_LIMIT_SCALE for v in values))
    return tuple(limits)


def prepare_playback_frames(frames, stages=None):
    """离线平滑位置，再生成形状保持的三次曲线切线；不使用原始测量速度。"""
    import numpy as np
    if len(frames) == 1:
        result = dict(frames[0], vel=[0.0]*6)
        if "gripper_pos" in result:
            result["gripper_vel"] = 0.0
        return [result]
    times = np.array([f["t"] for f in frames])
    values = np.array([f["pos"] + ([f["gripper_pos"]] if "gripper_pos" in f else []) for f in frames])
    # 保留稀疏停留段的端点，避免把长间隔误当成运动。
    gaps = np.flatnonzero(np.diff(times) > 3*PLAYBACK_RESAMPLE_DT)
    boundaries = [s[key] for s in (stages or []) for key in ("start", "end")
                  if times[0] <= s[key] <= times[-1]]
    anchors = np.unique(np.r_[0.0, times[gaps], times[gaps+1], times[-1], boundaries])
    grid = np.concatenate([np.linspace(start, end, max(1, math.ceil((end-start)/PLAYBACK_RESAMPLE_DT))+1)[:-1]
                           for start, end in zip(anchors[:-1], anchors[1:])] + [anchors[-1:]])
    raw = np.column_stack([np.interp(grid, times, column) for column in values.T])
    window = min(PLAYBACK_SMOOTH_WINDOW, len(grid) if len(grid) % 2 else len(grid)-1)
    pad = window // 2
    padded = np.pad(raw, ((pad, pad), (0, 0)), mode="edge")
    smoothed = np.column_stack([np.convolve(column, np.ones(window)/window, mode="valid")
                                for column in padded.T])
    smoothed[0], smoothed[-1] = raw[0], raw[-1]
    boundary_mask = np.isin(grid, boundaries)
    # 回零和其他模块边界必须精确到达，不能被平滑窗口跨段抹去。
    smoothed[boundary_mask] = raw[boundary_mask]
    for index in gaps:
        if np.all(values[index] == values[index+1]):
            smoothed[(grid >= times[index]) & (grid <= times[index+1])] = values[index]
    dt = np.diff(grid)
    slopes = np.diff(smoothed, axis=0) / dt[:, None]
    velocities = np.zeros_like(smoothed)
    if len(grid) > 2:
        same_sign = slopes[:-1]*slopes[1:] > 0
        w1, w2 = (2*dt[1:]+dt[:-1])[:, None], (dt[1:]+2*dt[:-1])[:, None]
        with np.errstate(divide="ignore", invalid="ignore"):
            harmonic = (w1+w2) / (w1/slopes[:-1]+w2/slopes[1:])
        velocities[1:-1] = np.where(same_sign, harmonic, 0.0)
    velocities[boundary_mask] = 0.0
    result = []
    for t, position, velocity in zip(grid, smoothed, velocities):
        frame = {"t": float(t), "pos": position[:6].tolist(), "vel": velocity[:6].tolist()}
        if values.shape[1] == 7:
            frame.update(gripper_pos=float(position[6]), gripper_vel=float(velocity[6]))
        result.append(frame)
    return result


def transition_duration(first, last, minimum=1.0):
    velocity, acceleration = load_transition_limits(str(CONFIG_PATH), CONFIG_PATH.stat().st_mtime_ns)
    distances = [abs(b - a) for a, b in zip(first["pos"], last["pos"])]
    # 五次平滑插值的归一化峰值：速度 15/8，加速度 10/sqrt(3)。
    duration = max([minimum] + [max(1.875*d/v, math.sqrt((10/math.sqrt(3))*d/a))
                               for d, v, a in zip(distances, velocity, acceleration)])
    if "gripper_pos" in first and "gripper_pos" in last:
        # YAML 未提供夹爪运动限制，夹爪保留原规划约束。
        distance = abs(last["gripper_pos"] - first["gripper_pos"])
        duration = max(duration, 3.75*distance, math.sqrt(12*distance))
    return duration


def transition_frame(first, last, elapsed, duration):
    """五次插值，端点速度和加速度为零。"""
    u = min(1.0, max(0.0, elapsed / duration)) if duration else 1.0
    blend = 10*u**3 - 15*u**4 + 6*u**5
    rate = (30*u**2 - 60*u**3 + 30*u**4) / duration if duration else 0.0
    delta = [b - a for a, b in zip(first["pos"], last["pos"])]
    frame = {"pos": [a + blend*d for a, d in zip(first["pos"], delta)],
             "vel": [rate*d for d in delta]}
    if "gripper_pos" in first and "gripper_pos" in last:
        difference = last["gripper_pos"] - first["gripper_pos"]
        frame.update(gripper_pos=first["gripper_pos"] + blend*difference,
                     gripper_vel=rate*difference)
    return frame


def load_frames(path):
    """在发送运动指令前检查完整文件，保留原有 JSONL 格式。"""
    frames = []
    with Path(path).open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                frame = json.loads(line)
                for key in ("t", "pos"):
                    if key not in frame:
                        raise ValueError(f"缺少 {key}")
                for key in ("pos", "vel"):
                    if key in frame:
                        if len(frame[key]) != 6:
                            raise ValueError(f"{key} 必须包含 6 个关节")
                        frame[key] = [float(v) for v in frame[key]]
                for key in ("t", "gripper_pos", "gripper_vel"):
                    if key in frame:
                        frame[key] = float(frame[key])
                values = [frame["t"], *frame["pos"], *frame.get("vel", [])]
                values += [frame[k] for k in ("gripper_pos", "gripper_vel") if k in frame]
                if not all(math.isfinite(v) for v in values):
                    raise ValueError("存在非有限数值")
                if frames and frame["t"] <= frames[-1]["t"]:
                    raise ValueError("时间戳必须严格递增")
                if frames and ("gripper_pos" in frame) != ("gripper_pos" in frames[0]):
                    raise ValueError("夹爪位置字段不一致")
                frames.append(frame)
            except (ValueError, TypeError, KeyError) as exc:
                raise ValueError(f"第 {line_number} 行轨迹无效：{exc}") from exc
    if not frames:
        raise ValueError("轨迹文件为空")
    start = frames[0]["t"]
    for frame in frames:
        frame["t"] -= start
    return frames


def concatenate_trajectories(segments, with_stages=False):
    """segments: (文件路径, 段后停留秒数, 到下一段的最短过渡秒数)。"""
    if len(segments) < 2:
        raise ValueError("请至少添加两段轨迹（可以重复添加同一文件）")
    loaded = []
    for path, gap, transition in segments:
        if not all(math.isfinite(v) and v >= 0 for v in (gap, transition)):
            raise ValueError("间隔和过渡时间必须为非负有限数值")
        loaded.append(load_frames(path))
    has_gripper = "gripper_pos" in loaded[0][0]
    if any(("gripper_pos" in frames[0]) != has_gripper for frames in loaded):
        raise ValueError("所选轨迹的夹爪数据不一致，请使用全部含夹爪或全部不含夹爪的轨迹")

    def copy_frame(frame, timestamp, stopped=False):
        result = dict(frame, t=timestamp, pos=list(frame["pos"]),
                      vel=[0.0] * 6 if stopped else list(frame.get("vel", [0.0] * 6)))
        if has_gripper:
            result["gripper_vel"] = 0.0 if stopped else frame.get("gripper_vel", 0.0)
        return result

    output = [copy_frame(f, f["t"]) for f in loaded[0]]
    stages = [{"start": 0.0, "end": output[-1]["t"], "kind": "trajectory",
               "label": f"1 · {Path(segments[0][0]).stem}"}]
    for index, incoming in enumerate(loaded[1:]):
        gap, minimum_transition = segments[index][1:]
        last = output[-1] = copy_frame(output[-1], output[-1]["t"], stopped=True)
        if gap > 0:
            output.append(copy_frame(last, last["t"] + gap, stopped=True))
            stages.append({"start": last["t"], "end": output[-1]["t"],
                           "kind": "gap", "label": f"间隔 {index + 1}"})
        first = incoming[0]
        # 五次插值：两端速度、加速度为零；较大位移自动延长过渡。
        duration = transition_duration(last, first, minimum_transition)
        start = output[-1]["t"]
        if duration > 0:
            steps = max(1, math.ceil(duration / 0.01))
            for step in range(1, steps + 1):
                elapsed = duration * step / steps
                frame = dict(transition_frame(last, first, elapsed, duration), t=start + elapsed)
                output.append(frame)
            stages.append({"start": start, "end": output[-1]["t"],
                           "kind": "transition", "label": f"过渡 {index + 1}→{index + 2}"})
        offset = output[-1]["t"]
        output.extend(copy_frame(f, offset + f["t"]) for f in incoming[1:])
        stages.append({"start": offset, "end": output[-1]["t"], "kind": "trajectory",
                       "label": f"{index + 2} · {Path(segments[index + 1][0]).stem}"})
    return (output, stages) if with_stages else output


def validate_duration(value, minimum=0.0):
    value = float(value)
    if not math.isfinite(value) or not minimum <= value <= 3600:
        raise ValueError(f"时间必须在 {minimum:g} 到 3600 秒之间")
    return value


def recipe_modules(data):
    """旧方案转为显式阶段，起始过渡只保存参数，不固化当前机械臂姿态。"""
    if data.get("version") == 1:
        modules = [{"kind": "start", "duration": 2.0}]
        for index, segment in enumerate(data["segments"]):
            modules.append({"kind": "trajectory", "file": segment["file"]})
            if index < len(data["segments"])-1:
                if segment["gap"]:
                    modules.append({"kind": "gap", "duration": segment["gap"]})
                modules.append({"kind": "transition", "duration": segment["transition"]})
    elif data.get("version") == 2:
        modules = [dict(module) for module in data["modules"]]
    else:
        raise ValueError("不支持的方案版本")
    if len(modules) < 2 or modules[0]["kind"] != "start" or modules[1]["kind"] != "trajectory":
        raise ValueError("方案必须以起始过渡、轨迹开始")
    previous = None
    for index, module in enumerate(modules):
        kind = module["kind"]
        if kind == "trajectory":
            filename = module["file"]
            if Path(filename).name != filename or not filename.endswith(".jsonl"):
                raise ValueError("方案中的轨迹文件名无效")
            if previous not in ("start", "transition"):
                raise ValueError("轨迹前必须有起始过渡或段间过渡")
        elif kind in ("start", "transition", "zero", "gap"):
            module["duration"] = validate_duration(module["duration"], .1 if kind in ("start", "zero") else 0)
            if kind == "start" and index != 0:
                raise ValueError("起始过渡只能位于第一行")
            if kind == "transition" and (index+1 >= len(modules) or modules[index+1]["kind"] != "trajectory"):
                raise ValueError("段间过渡必须紧邻后面的轨迹")
            if kind in ("zero", "gap", "transition") and previous not in ("trajectory", "zero", "gap"):
                raise ValueError("回零、间隔和段间过渡的位置无效")
        else:
            raise ValueError(f"未知阶段：{kind}")
        previous = kind
    return modules


def compile_sequence(modules):
    modules = recipe_modules({"version": 2, "modules": modules})
    sources = {m["file"]: load_frames(TRAJECTORY_DIR / m["file"])
               for m in modules if m["kind"] == "trajectory"}
    with_gripper = {"gripper_pos" in f[0] for f in sources.values()}
    if len(with_gripper) != 1:
        raise ValueError("所选轨迹的夹爪数据不一致")
    output, stages = [], []
    number = 0
    for index, module in enumerate(modules[1:], 1):
        kind = module["kind"]
        start = output[-1]["t"] if output else 0.0
        if kind == "trajectory":
            number += 1
            incoming = sources[module["file"]]
            output.extend(dict(f, t=start+f["t"], pos=list(f["pos"]))
                          for f in (incoming[1:] if output else incoming))
            label = f"{number} · {Path(module['file']).stem}"
        else:
            last = dict(output[-1], vel=[0.0]*6)
            if "gripper_pos" in last:
                last["gripper_vel"] = 0.0
            output[-1] = last
            if kind == "gap":
                duration = module["duration"]
                if duration <= 0:
                    continue
                output.append(dict(last, t=start+duration))
                label = "停留间隔"
            else:
                if kind == "zero":
                    target = dict(last, pos=[0.0]*6)
                    label = "回零位"
                else:
                    target = sources[modules[index+1]["file"]][0]
                    label = f"过渡 → {number+1}"
                duration = transition_duration(last, target, module["duration"])
                steps = max(1, math.ceil(duration/.01))
                if duration > 0:
                    for step in range(1, steps+1):
                        elapsed = duration*step/steps
                        output.append(dict(transition_frame(last, target, elapsed, duration), t=start+elapsed))
        stages.append({"start": start, "end": output[-1]["t"], "kind": kind, "label": label})
    return output, stages


def initial_stage_preview(frames, stages, minimum_duration, current=None):
    duration = transition_duration(current, frames[0], minimum_duration) if current else minimum_duration
    prefix = []
    if current:
        steps = max(1, math.ceil(duration/.01))
        prefix = [dict(transition_frame(current, frames[0], duration*i/steps, duration), t=duration*i/steps)
                  for i in range(steps)]
    shifted = [dict(f, t=f["t"]+duration) for f in frames]
    labels = [{"start": 0.0, "end": duration, "kind": "start",
               "label": "起始过渡" if current else "起始过渡（当前位置待定）"}]
    labels.extend(dict(s, start=s["start"]+duration, end=s["end"]+duration) for s in stages)
    return prefix+shifted, labels


class ConcatenateDialog(QtWidgets.QDialog):
    def __init__(self, parent):
        super().__init__(parent)
        self.output_path = self.recipe_path = None
        self.actions = []
        self.initial_duration = 2.0
        self.row_bindings = []
        self.setWindowTitle("编辑拼接方案")
        self.resize(1100, 820)
        self.preview_timer = QtCore.QTimer(self)
        self.preview_timer.setSingleShot(True)
        self.preview_timer.setInterval(200)
        self.preview_timer.timeout.connect(self.update_preview)
        layout = QtWidgets.QVBoxLayout(self)
        recipe_row = QtWidgets.QHBoxLayout()
        self.recipes = QtWidgets.QComboBox()
        for path in sorted(TRAJECTORY_DIR.glob("*.sequence.json")):
            self.recipes.addItem(path.name.removesuffix(".sequence.json"), str(path))
        for widget in (self.recipes,):
            recipe_row.addWidget(widget, 1)
        for label, callback in (("打开已有方案", self.open_recipe), ("新建方案", self.new_recipe)):
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(callback)
            recipe_row.addWidget(button)
        layout.addLayout(recipe_row)
        source_row = QtWidgets.QHBoxLayout()
        self.source = QtWidgets.QComboBox()
        for path in sorted(TRAJECTORY_DIR.glob("*.jsonl")):
            if path.is_file():
                self.source.addItem(path.name, str(path))
        source_row.addWidget(self.source, 1)
        add = QtWidgets.QPushButton("添加轨迹到末尾")
        add.clicked.connect(self.add_segment)
        source_row.addWidget(add)
        layout.addLayout(source_row)
        self.table = QtWidgets.QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(["阶段 / 轨迹", "过渡至少（秒）"])
        self.table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        for column in (1,):
            self.table.horizontalHeader().setSectionResizeMode(column, QtWidgets.QHeaderView.ResizeToContents)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        layout.addWidget(self.table)
        controls = QtWidgets.QHBoxLayout()
        for label, callback in (("上移", lambda: self.move_segment(-1)),
                                ("下移", lambda: self.move_segment(1)),
                                ("替换为上方轨迹", self.replace_segment),
                                ("在选中轨迹后插入回零位", self.add_zero),
                                ("插入停留模块", self.add_gap),
                                ("移除选中模块", self.remove_segment)):
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(callback)
            controls.addWidget(button)
        layout.addLayout(controls)
        self.plot = TrajectoryPlot()
        layout.addWidget(self.plot, 1)
        self.preview_info = QtWidgets.QLabel()
        self.preview_info.setWordWrap(True)
        layout.addWidget(self.preview_info)
        note = QtWidgets.QLabel("过渡行可设置最短过渡时间，位移较大时自动延长；轨迹按原始时间轴和 2 ms 控制周期完整回放。\n"
                               "回零位：六个关节插值到 0 rad，夹爪保持进入该模块时的位置。停留模块按列表顺序执行，保持上一模块的结束姿态。")
        note.setWordWrap(True)
        layout.addWidget(note)
        self.name = QtWidgets.QLineEdit()
        form = QtWidgets.QFormLayout()
        form.addRow("方案名称（修改名称可另存）：", self.name)
        layout.addLayout(form)
        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Save | QtWidgets.QDialogButtonBox.Cancel)
        buttons.button(QtWidgets.QDialogButtonBox.Save).setText("保存方案并更新轨迹")
        buttons.button(QtWidgets.QDialogButtonBox.Cancel).setText("取消")
        buttons.accepted.connect(self.save)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)
        self.new_recipe()
        if parent is not None and parent.files.currentData():
            recipe = Path(parent.files.currentData()).with_suffix(".sequence.json")
            if recipe.is_file():
                self.load_recipe(recipe)

    def new_recipe(self):
        self.recipe_path = None
        self.initial_duration = 2.0
        self.actions = []
        self.name.setText(datetime.now().strftime("combined_%Y%m%d_%H%M%S_%f"))
        self.render_rows()

    def open_recipe(self):
        if self.recipes.currentData():
            self.load_recipe(Path(self.recipes.currentData()))

    def load_recipe(self, path):
        try:
            modules = recipe_modules(json.loads(path.read_text(encoding="utf-8")))
            actions = []
            for module in modules[1:]:
                kind = module["kind"]
                if kind in ("trajectory", "zero", "gap"):
                    actions.append(dict(module, transition=1.0))
                elif kind == "transition":
                    actions[-1]["transition"] = module["duration"]
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "打开方案失败", str(exc))
            return
        self.initial_duration = modules[0]["duration"]
        self.actions = actions
        self.recipe_path = path
        self.name.setText(path.name.removesuffix(".sequence.json"))
        self.recipes.setCurrentIndex(self.recipes.findData(str(path)))
        self.render_rows()

    def modules(self):
        result = [{"kind": "start", "duration": self.initial_duration}]
        for index, action in enumerate(self.actions):
            if index and action["kind"] == "trajectory":
                result.append({"kind": "transition", "duration": self.actions[index-1]["transition"]})
            if action["kind"] == "trajectory":
                result.append({"kind": "trajectory", "file": action["file"]})
            else:
                result.append({"kind": action["kind"], "duration": action["duration"]})
        return result

    def render_rows(self, selected=None):
        self.table.setRowCount(0)
        self.row_bindings = []
        def row(label, binding, duration=None):
            index = self.table.rowCount()
            self.table.insertRow(index)
            self.row_bindings.append(binding)
            kind, action_index = binding
            color = {"start": "#ede9fe", "transition": "#ffedd5", "zero": "#dcfce7", "gap": "#f1f5f9"}.get(kind, "#eff6ff")
            for column in range(2):
                item = QtWidgets.QTableWidgetItem(label if column == 0 else "—")
                item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
                item.setBackground(QtGui.QColor(color))
                self.table.setItem(index, column, item)
            for column, value in ((1, duration),):
                if value is None:
                    continue
                spin = QtWidgets.QDoubleSpinBox()
                spin.setDecimals(2)
                spin.setRange(.1 if kind in ("start", "zero") else 0, 3600)
                spin.setValue(value)
                if kind != "gap":
                    spin.setToolTip("仅设置本过渡阶段的最短时间；位移较大时自动延长，不改变录制轨迹的时长和控制周期。")
                spin.valueChanged.connect(lambda value, b=binding, c=column: self.change_time(b, c, value))
                if kind == "gap":
                    container = QtWidgets.QWidget()
                    layout = QtWidgets.QHBoxLayout(container)
                    layout.setContentsMargins(6, 0, 6, 0)
                    layout.addWidget(QtWidgets.QLabel(label))
                    spin.setSuffix(" 秒")
                    spin.setMaximumWidth(140)
                    layout.addWidget(spin)
                    layout.addStretch()
                    spin.setObjectName("gap_duration")
                    spin.valueChanged.connect(lambda _, r=index: self.table.selectRow(r))
                    self.table.setCellWidget(index, 0, container)
                else:
                    self.table.setCellWidget(index, column, spin)
            if selected == action_index and kind in ("trajectory", "zero", "gap"):
                self.table.selectRow(index)
        row("起始过渡：当前位置 → 第一段起点", ("start", -1), duration=self.initial_duration)
        for index, action in enumerate(self.actions):
            if index and action["kind"] == "trajectory":
                row("段间过渡：上一模块末尾 → 下一轨迹起点", ("transition", index-1),
                    duration=self.actions[index-1]["transition"])
            if action["kind"] == "trajectory":
                row(action["file"], ("trajectory", index))
            elif action["kind"] == "zero":
                row("回零位：J1–J6 → 0 rad（夹爪保持）", ("zero", index), duration=action["duration"])
            else:
                row("停留：保持上一模块结束姿态", ("gap", index), duration=action["duration"])
        self.preview_timer.start()

    def change_time(self, binding, column, value):
        kind, index = binding
        if kind == "start":
            self.initial_duration = value
        elif kind == "transition":
            self.actions[index]["transition"] = value
        else:
            self.actions[index]["duration"] = value
        self.preview_timer.start()

    def selected_action(self):
        row = self.table.currentRow()
        if row < 0:
            return None
        kind, index = self.row_bindings[row]
        return index if kind in ("trajectory", "zero", "gap") else None

    def insert_segment(self, index, path, gap=0.0, transition=1.0):
        self.actions.insert(index, {"kind": "trajectory", "file": Path(path).name,
                                    "transition": transition})
        if gap > 0:
            self.actions.insert(index+1, {"kind": "gap", "duration": gap, "transition": transition})
        self.render_rows(index)

    def add_segment(self):
        if self.source.currentData():
            self.insert_segment(len(self.actions), self.source.currentData())

    def add_zero(self):
        index = self.selected_action()
        if index is None:
            QtWidgets.QMessageBox.information(self, "选择轨迹", "请先选中要在其后插入回零位的轨迹。")
            return
        self.actions.insert(index+1, {"kind": "zero", "duration": 2.0, "transition": 1.0})
        self.render_rows(index+1)

    def add_gap(self):
        index = self.selected_action()
        if index is None:
            QtWidgets.QMessageBox.information(self, "选择模块", "请先选中轨迹、回零位或停留模块，在其后插入停留。")
            return
        transition = self.actions[index]["transition"]
        self.actions.insert(index+1, {"kind": "gap", "duration": 1.0, "transition": transition})
        self.render_rows(index+1)

    def move_segment(self, direction):
        index = self.selected_action()
        if index is None:
            return
        target = index+direction
        if 0 <= target < len(self.actions):
            new = list(self.actions)
            new[index], new[target] = new[target], new[index]
            if new[0]["kind"] != "trajectory":
                return
            self.actions = new
            self.render_rows(target)

    def remove_segment(self):
        index = self.selected_action()
        if index is not None:
            new = self.actions[:index]+self.actions[index+1:]
            if new and new[0]["kind"] != "trajectory":
                QtWidgets.QMessageBox.information(self, "无法移除", "请先移除或移动第一段后的模块，第一段必须为轨迹。")
                return
            self.actions = new
            self.render_rows(min(index, len(new)-1))

    def replace_segment(self):
        index = self.selected_action()
        if index is not None and self.actions[index]["kind"] == "trajectory" and self.source.currentData():
            self.actions[index]["file"] = Path(self.source.currentData()).name
            self.render_rows(index)

    def update_preview(self):
        try:
            if not self.actions:
                self.plot.set_frames([])
                self.preview_info.setText("添加轨迹后预览，起始过渡在回放时自动生成。")
                return
            frames, stages = compile_sequence(self.modules())
            timing = []
            transition_stages = iter(s for s in stages if s["kind"] in ("transition", "zero"))
            for row, (kind, index) in enumerate(self.row_bindings):
                if kind not in ("transition", "zero"):
                    continue
                stage = next(transition_stages)
                actual = stage["end"]-stage["start"]
                requested = self.actions[index]["transition" if kind == "transition" else "duration"]
                detail = f"{stage['label']}：设置 {requested:.2f} 秒，实际 {actual:.2f} 秒"
                if actual > requested+1e-6:
                    detail += "（按位移自动延长）"
                self.table.item(row, 0).setText(detail)
                self.table.cellWidget(row, 1).setToolTip(detail)
                timing.append(detail)
            current = getattr(self.parent(), "last_robot_frame", None)
            preview, stages = initial_stage_preview(frames, stages, self.initial_duration, current)
            self.plot.set_frames(preview, stages)
            qualifier = "预计" if current else "至少"
            self.preview_info.setText(f"含起始过渡{qualifier} {preview[-1]['t']:.2f} 秒 · "
                "紫色：起始过渡 / 蓝色：轨迹 / 灰色：停留 / 橙色：段间过渡 / 绿色：回零位\n"
                "起始过渡按回放时实际位置生成；当前位置未知时，预览只显示该阶段区域。"
                + ("\n" + "；".join(timing) if timing else ""))
        except Exception as exc:
            self.plot.set_frames([])
            self.preview_info.setText(f"无法预览：{exc}")

    def save(self):
        name = self.name.text().strip()
        if name.endswith(".jsonl"):
            name = name[:-6]
        if not name.strip() or name in (".", "..") or any(c in name for c in '/\\\0\n\r'):
            QtWidgets.QMessageBox.warning(self, "无法保存", "请输入有效的文件名。")
            return
        path = TRAJECTORY_DIR / (name+".jsonl")
        recipe = TRAJECTORY_DIR / (name+".sequence.json")
        temporary = []
        try:
            if self.recipe_path != recipe and (path.exists() or recipe.exists()):
                raise ValueError("名称已存在，请打开对应方案编辑，或使用新名称另存。")
            modules = self.modules()
            if any(m.get("file") == path.name for m in modules):
                raise ValueError("方案不能把自身输出作为输入")
            frames, stages = compile_sequence(modules)
            data = {"version": 2, "modules": modules, "stages": stages}
            for destination in (path, recipe):
                with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=TRAJECTORY_DIR,
                        prefix=".sequence_", suffix=".tmp", delete=False) as stream:
                    temporary.append((Path(stream.name), destination))
                    if destination == path:
                        for frame in frames:
                            stream.write(json.dumps(frame, ensure_ascii=False)+"\n")
                    else:
                        json.dump(data, stream, ensure_ascii=False, indent=2)
            for source, destination in temporary:
                source.replace(destination)
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "拼接失败", str(exc))
            return
        finally:
            for source, _ in temporary:
                source.unlink(missing_ok=True)
        self.recipe_path, self.output_path = recipe, path
        self.accept()


class Cancelled(Exception):
    pass


class PlaybackCancelled(Exception):
    pass


class RobotWorker(QtCore.QThread):
    status = QtCore.pyqtSignal(str)
    progress = QtCore.pyqtSignal(int, int)
    sample = QtCore.pyqtSignal(float, list)
    error = QtCore.pyqtSignal(str)
    done = QtCore.pyqtSignal()
    gravity_ready = QtCore.pyqtSignal()
    recording_changed = QtCore.pyqtSignal(bool, str)
    playback_finished = QtCore.pyqtSignal()
    state_snapshot = QtCore.pyqtSignal(object)
    playback_plan = QtCore.pyqtSignal(object, object)

    def __init__(self):
        super().__init__()
        self.tasks = queue.Queue()
        self.record_commands = queue.Queue()
        self.cancel = threading.Event()
        self.shutdown_requested = threading.Event()
        self.brake_applied = False
        self.play_cancel = threading.Event()
        self.playing = False
        self.robot = None
        self.plan_frames = []
        self.plan_stages = []

    def submit(self, task):
        self.brake_applied = False
        self.cancel.clear()
        self.play_cancel.clear()
        self.record_commands = queue.Queue()
        self.tasks.put(task)

    def check_cancel(self):
        if self.cancel.is_set():
            raise Cancelled()
        if self.playing and self.play_cancel.is_set():
            raise PlaybackCancelled()

    def pause(self, seconds):
        if self.cancel.wait(max(0.0, seconds)):
            raise Cancelled()

    def wait_next_tick(self, previous_tick):
        now = time.perf_counter()
        target = previous_tick + CONTROL_DT
        if target <= now:
            # 跳过错过的发送时隙，保持 2 ms 时间网格，不补发旧帧。
            target += (math.floor((now-target)/CONTROL_DT)+1)*CONTROL_DT
        self.pause(target-now)
        return target

    def shutdown(self):
        self.shutdown_requested.set()
        self.cancel.set()
        self.tasks.put(None)

    def enter_brake(self):
        """由控制线程发送 SDK 阻尼指令，避免退出时先进入无力状态。"""
        if self.robot is not None and not self.brake_applied:
            if self.robot.set_brake() is False:
                raise RuntimeError("机械臂拒绝进入阻尼模式")
            self.brake_applied = True
            self.status.emit("机械臂已进入阻尼模式")

    def run(self):
        while True:
            task = self.tasks.get()
            if task is None:
                try:
                    self.enter_brake()
                except Exception as exc:
                    self.error.emit(f"关闭时进入阻尼模式失败：{exc}")
                self.robot = None
                return
            try:
                # 延迟导入，让未安装机器人 SDK 的电脑也能预览轨迹。
                self.status.emit("正在准备机械臂…")
                from Panthera_lib import Panthera, TrajectoryRecorder
                import numpy as np
                self.np = np
                self.recorder_class = TrajectoryRecorder
                self.check_cancel()
                if self.robot is None:
                    self.robot = Panthera(str(CONFIG_PATH))
                if self.robot.motor_count != 6:
                    raise ValueError("当前界面使用六关节机械臂参数")
                for _ in range(10):
                    self.check_cancel()
                    self.robot.send_get_motor_state_cmd()
                    self.pause(0.1)
                self.record()
            except Cancelled:
                self.status.emit("任务已停止")
            except (Exception, SystemExit) as exc:
                self.status.emit("任务失败")
                self.error.emit(str(exc))
            finally:
                if self.robot is not None:
                    try:
                        if self.shutdown_requested.is_set() or self.cancel.is_set():
                            self.enter_brake()
                        else:
                            self.robot.set_stop()
                    except Exception as exc:
                        self.error.emit(f"结束控制失败：{exc}")
                self.done.emit()

    def read_state(self):
        r = self.robot
        pos, vel = list(r.get_current_pos()), list(r.get_current_vel())
        if r.has_gripper:
            gp = float(r.get_current_pos_gripper())
            gv = float(r.get_current_vel_gripper())
        else:
            gp = gv = 0.0
        if not all(math.isfinite(v) and abs(v) < 999 for v in [*pos, *vel, gp, gv]):
            raise ValueError("机械臂状态无效，请检查电机通信")
        return pos, vel, gp, gv

    def record(self, path=None):
        """持续重力补偿；只有收到开始记录命令后才创建文件。"""
        recorder = None
        recording_path = None
        if path is not None:
            self.record_commands.put(("start", path))
        start = tick = next_record = next_display = time.perf_counter()
        ready = False
        zeros = [0.0] * 6
        try:
            while True:
                self.check_cancel()
                try:
                    command, requested_path = self.record_commands.get_nowait()
                except queue.Empty:
                    command, requested_path = None, None
                # 先决定本周期的控制模式，再发送指令。回放请求不能先发纯重力帧。
                if command == "play" and recorder is None:
                    self.playing = True
                    try:
                        self.play(*requested_path)
                    except PlaybackCancelled:
                        self.status.emit("回放已停止，继续重力补偿")
                    finally:
                        self.playing = False
                    self.playback_finished.emit()
                    tick = next_display = time.perf_counter()
                    continue
                pos, vel, gp, gv = self.read_state()
                torque = self.np.clip(self.robot.get_Gravity(pos),
                                      -self.np.array(TAU_LIMIT), TAU_LIMIT)
                self.send_mit(pos, zeros, torque, zeros, zeros, (gp, 0.0, 0.0, 0.0, 0.0))
                now = time.perf_counter()
                if not ready:
                    ready = True
                    self.status.emit("重力补偿已启动：可自由拖动、开始记录或回放")
                    self.gravity_ready.emit()
                if command == "start" and recorder is None:
                    self.check_cancel()
                    recording_path = Path(requested_path)
                    recorder = self.recorder_class(str(recording_path))
                    start = next_record = next_display = now
                    self.recording_changed.emit(True, str(recording_path))
                    self.status.emit(f"正在记录：{recording_path.name}")
                elif command == "stop" and recorder is not None:
                    recorder.close()
                    recorder = None
                    self.recording_changed.emit(False, str(recording_path))
                    self.status.emit("轨迹已保存，重力补偿仍开启，可继续拖动或再次记录")
                if recorder is not None and now >= next_record:
                    recorder.log(
                        pos,
                        vel,
                        gp if self.robot.has_gripper else None,
                        gv if self.robot.has_gripper else None,
                    )
                    next_record = now + 0.01
                if now >= next_display:
                    self.sample.emit(now - start, pos)
                    snapshot = {"pos": list(pos)}
                    if self.robot.has_gripper:
                        snapshot["gripper_pos"] = gp
                    self.state_snapshot.emit(snapshot)
                    next_display = now + 0.1
                tick = self.wait_next_tick(tick)
        finally:
            if recorder is not None:
                recorder.close()
                self.recording_changed.emit(False, str(recording_path))

    def validate_limits(self, frames):
        limits = self.robot.joint_limits
        for frame in frames:
            if limits is not None and any(p < lo or p > hi for p, lo, hi in
                    zip(frame["pos"], limits["lower"], limits["upper"])):
                raise ValueError("轨迹中存在超出机械臂关节限位的位置")
            grip = self.robot.gripper_limits
            if self.robot.has_gripper and "gripper_pos" in frame and grip is not None:
                if not grip["lower"] <= frame["gripper_pos"] <= grip["upper"]:
                    raise ValueError("轨迹中存在超出夹爪限位的位置")

    def move_to_start(self, frame, minimum_duration=2.0, body_duration=0.0, completed=0):
        self.check_cancel()
        pos, _, gp, _ = self.read_state()
        current = {"pos": pos}
        if self.robot.has_gripper and "gripper_pos" in frame:
            current["gripper_pos"] = gp
        self.validate_limits([current, frame])
        duration = transition_duration(current, frame, minimum_duration)
        if self.plan_frames:
            self.playback_plan.emit(*initial_stage_preview(self.plan_frames, self.plan_stages, minimum_duration, current))
        self.status.emit(f"正在插值到轨迹起点（{duration:.2f} 秒）…")
        start = tick = time.perf_counter()
        next_display = start
        while True:
            elapsed = min(time.perf_counter()-start, duration)
            self.send_frame(transition_frame(current, frame, elapsed, duration))
            now = time.perf_counter()
            if now >= next_display or elapsed >= duration:
                self.progress.emit(int(100*elapsed/max(duration+body_duration, 1e-9)), completed)
                self.sample.emit(elapsed, list(self.robot.get_current_pos()))
                next_display = now+.1
            if elapsed >= duration:
                break
            tick = self.wait_next_tick(tick)
        # 持续发送相同的 MIT 起点目标，确认到位后再进入轨迹。
        deadline = time.perf_counter() + 5
        while time.perf_counter() < deadline:
            self.check_cancel()
            self.send_frame(frame, hold=True)
            pos, _, gp, _ = self.read_state()
            reached = all(abs(p - target) < 0.05 for p, target in zip(pos, frame["pos"]))
            grip_reached = (not self.robot.has_gripper or
                            "gripper_pos" not in frame or
                            abs(gp - frame["gripper_pos"]) < 0.05)
            if reached and grip_reached:
                return duration
            self.pause(CONTROL_DT)
        raise TimeoutError("插值结束后 5 秒内仍未到达轨迹起点，已停止电机")

    def send_mit(self, pos, velocity, torque, kp, kd, gripper):
        """缓存夹爪后，由完整的六关节 MIT 命令统一发送一次。"""
        self.check_cancel()
        if self.robot.has_gripper:
            if self.robot.gripper_control_MIT(*gripper, send=False) is False:
                raise ValueError("夹爪 MIT 命令被拒绝，本周期未发送")
        if self.robot.pos_vel_tqe_kp_kd(pos, velocity, torque, kp, kd) is False:
            raise ValueError("关节 MIT 命令被拒绝，本周期未发送")

    def send_frame(self, frame, hold=False):
        self.check_cancel()
        velocity = [0.0] * 6 if hold else frame.get("vel", [0.0] * 6)
        current_pos, _, gp, _ = self.read_state()
        # 与自由拖动使用同一实际姿态计算重力；不叠加带阈值跳变的摩擦补偿。
        torque = self.np.array(self.robot.get_Gravity(current_pos))
        torque = self.np.clip(torque, -self.np.array(TAU_LIMIT), TAU_LIMIT)
        if "gripper_pos" in frame:
            gripper = (frame["gripper_pos"],
                       0.0 if hold else frame.get("gripper_vel", 0.0), 0.0, 5.0, 0.5)
        else:
            gripper = (gp, 0.0, 0.0, 0.0, 0.0)
        self.send_mit(frame["pos"], velocity, torque, KP, KD, gripper)

    @staticmethod
    def sample_frame(frames, timestamps, elapsed):
        index = bisect.bisect_right(timestamps, elapsed)
        if index == 0:
            return frames[0]
        if index >= len(frames):
            return frames[-1]
        a, b = frames[index - 1], frames[index]
        duration = b["t"] - a["t"]
        ratio = (elapsed - a["t"]) / duration
        def cubic(p, q, v0, v1):
            u = ratio
            a3 = 2*p-2*q+duration*(v0+v1)
            a2 = -3*p+3*q-duration*(2*v0+v1)
            a1 = duration*v0
            return ((a3*u+a2)*u+a1)*u+p, (3*a3*u*u+2*a2*u+a1)/duration
        slope = [(q-p)/duration for p, q in zip(a["pos"], b["pos"])]
        samples = [cubic(p, q, v0, v1) for p, q, v0, v1 in
                   zip(a["pos"], b["pos"], a.get("vel", slope), b.get("vel", slope))]
        result = {"pos": [p for p, _ in samples], "vel": [v for _, v in samples]}
        if "gripper_pos" in a:
            grip_slope = (b["gripper_pos"] - a["gripper_pos"]) / duration
            result["gripper_pos"], result["gripper_vel"] = cubic(a["gripper_pos"], b["gripper_pos"],
                a.get("gripper_vel", grip_slope), b.get("gripper_vel", grip_slope))
        return result

    def play(self, frames, repetitions, interval, minimum_transition=2.0, stages=None):
        self.validate_limits(frames)
        self.plan_frames = frames
        self.plan_stages = stages or [{"start": 0.0, "end": frames[-1]["t"],
                                      "kind": "trajectory", "label": "轨迹"}]
        timestamps = [f["t"] for f in frames]
        completed = 0
        while repetitions == 0 or completed < repetitions:
            initial_duration = self.move_to_start(frames[0], minimum_transition, timestamps[-1], completed)
            self.status.emit(f"正在回放第 {completed + 1} 次 / {repetitions or '无限'}")
            start = tick = next_display = time.perf_counter()
            while True:
                # 轨迹时间直接跟随实际经过时间，发送周期不决定播放倍率。
                elapsed = min(time.perf_counter()-start, timestamps[-1])
                frame = self.sample_frame(frames, timestamps, elapsed)
                self.send_frame(frame)
                now = time.perf_counter()
                if now >= next_display or elapsed >= timestamps[-1]:
                    total = initial_duration+timestamps[-1]
                    self.progress.emit(int((initial_duration+elapsed)*100/total) if total else 100, completed)
                    self.sample.emit(initial_duration+elapsed, list(frame["pos"]))
                    next_display = now + 0.1
                if elapsed >= timestamps[-1]:
                    break
                tick = self.wait_next_tick(tick)
            completed += 1
            self.progress.emit(100, completed)
            if repetitions and completed >= repetitions:
                break
            deadline = time.perf_counter() + interval
            self.status.emit(f"已完成 {completed} 次，间隔 {interval:g} 秒（保持末端姿态）")
            while time.perf_counter() < deadline:
                self.send_frame(frames[-1], hold=True)
                self.pause(min(CONTROL_DT, max(0, deadline - time.perf_counter())))
        self.status.emit(f"回放完成，共 {completed} 次，继续重力补偿")


class TrajectoryPlot(QtWidgets.QWidget):
    """使用 Qt 绘图，无额外绘图库依赖。"""
    COLORS = ["#2563eb", "#dc2626", "#16a34a", "#9333ea", "#ea580c", "#0891b2"]

    def __init__(self):
        super().__init__()
        self.points = []
        self.stages = []
        self.stage_regions = []
        self.setMouseTracking(True)
        self.setMinimumHeight(220)

    def set_frames(self, frames, stages=None):
        self.stages = stages or []
        step = max(1, len(frames) // 1500)
        self.points = [(f["t"], f["pos"]) for f in frames[::step]]
        if frames and self.points[-1][0] != frames[-1]["t"]:
            self.points.append((frames[-1]["t"], frames[-1]["pos"]))
        self.update()

    def append(self, timestamp, positions):
        self.stages = []
        self.points.append((timestamp, positions))
        self.points = self.points[-1500:]
        self.update()

    def mouseMoveEvent(self, event):
        for rect, stage in self.stage_regions:
            if rect.contains(QtCore.QPointF(event.pos())):
                QtWidgets.QToolTip.showText(event.globalPos(),
                    f"{stage['label']}\n{stage['start']:.2f}–{stage['end']:.2f} 秒"
                    f"（{stage['end'] - stage['start']:.2f} 秒）", self)
                return
        QtWidgets.QToolTip.hideText()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor("#ffffff"))
        top = 62 if self.stages else 35
        area = QtCore.QRectF(65, top, max(1, self.width() - 90), max(1, self.height() - top - 35))
        self.stage_regions = []
        painter.setPen(QtGui.QColor("#64748b"))
        painter.drawRect(area)
        painter.drawText(8, 20, "关节位置 (rad)")
        for j, color in enumerate(self.COLORS):
            painter.setPen(QtGui.QColor(color))
            painter.drawText(160 + j * 55, 20, f"J{j + 1}")
        if not self.points:
            painter.drawText(area, QtCore.Qt.AlignCenter, "选择轨迹预览，或开始记录")
            return
        t0 = min([self.points[0][0]]+[stage["start"] for stage in self.stages])
        t1 = max([self.points[-1][0]]+[stage["end"] for stage in self.stages])
        def x_at(timestamp):
            return area.left() + (timestamp - t0) / max(t1 - t0, 0.001) * area.width()
        colors = {"trajectory": "#eff6ff", "gap": "#f1f5f9", "transition": "#ffedd5",
                  "start": "#ede9fe", "zero": "#dcfce7"}
        boundaries = set()
        for stage in self.stages:
            left, right = x_at(stage["start"]), x_at(stage["end"])
            region = QtCore.QRectF(left, 30, max(right-left, 1), area.bottom()-30)
            painter.fillRect(region, QtGui.QColor(colors.get(stage["kind"], "#eff6ff")))
            painter.setPen(QtGui.QColor("#334155"))
            label_rect = QtCore.QRectF(left + 3, 31, max(0, right-left-6), 26)
            label = painter.fontMetrics().elidedText(stage["label"], QtCore.Qt.ElideRight,
                                                     int(label_rect.width()))
            painter.drawText(label_rect, QtCore.Qt.AlignCenter, label)
            self.stage_regions.append((region, stage))
            boundaries.update((stage["start"], stage["end"]))
        low = min(min(p) for _, p in self.points) - 0.1
        high = max(max(p) for _, p in self.points) + 0.1
        painter.setPen(QtGui.QColor("#64748b"))
        painter.drawText(3, int(area.top()) + 10, f"{high:.2f}")
        painter.drawText(3, int(area.bottom()), f"{low:.2f}")
        painter.drawText(int(area.left()), self.height() - 8, f"{t0:.1f} s")
        painter.drawText(int(area.right()) - 70, self.height() - 8, f"{t1:.1f} s")
        for j, color in enumerate(self.COLORS):
            path = QtGui.QPainterPath()
            for i, (t, pos) in enumerate(self.points):
                x = area.left() + (t - t0) / max(t1 - t0, 0.001) * area.width()
                y = area.bottom() - (pos[j] - low) / (high - low) * area.height()
                if i == 0:
                    path.moveTo(x, y)
                else:
                    path.lineTo(x, y)
            painter.setPen(QtGui.QPen(QtGui.QColor(color), 1.5))
            painter.drawPath(path)
        painter.setPen(QtGui.QPen(QtGui.QColor("#64748b"), 1, QtCore.Qt.DashLine))
        for boundary in sorted(boundaries):
            x = x_at(boundary)
            painter.drawLine(QtCore.QPointF(x, 30), QtCore.QPointF(x, area.bottom()))


class TrajectoryWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        TRAJECTORY_DIR.mkdir(parents=True, exist_ok=True)
        self.busy = False
        self.closing = False
        self.mode = None
        self.preview_frames = []
        self.preview_stages = []
        self.initial_transition = 2.0
        self.last_robot_frame = None
        self.setWindowTitle("Panthera · 轨迹记录与回放")
        self.resize(900, 650)
        layout = QtWidgets.QVBoxLayout(self)
        directory = QtWidgets.QLabel(f"轨迹目录：{TRAJECTORY_DIR}")
        directory.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(directory)
        file_row = QtWidgets.QHBoxLayout()
        self.files = QtWidgets.QComboBox()
        self.refresh = QtWidgets.QPushButton("刷新轨迹")
        self.rename_button = QtWidgets.QPushButton("修改名称")
        self.delete_button = QtWidgets.QPushButton("删除轨迹")
        self.concatenate_button = QtWidgets.QPushButton("拼接轨迹")
        file_row.addWidget(self.files, 1)
        file_row.addWidget(self.refresh)
        file_row.addWidget(self.rename_button)
        file_row.addWidget(self.delete_button)
        file_row.addWidget(self.concatenate_button)
        layout.addLayout(file_row)
        self.info = QtWidgets.QLabel("暂无轨迹")
        layout.addWidget(self.info)
        self.plot = TrajectoryPlot()
        layout.addWidget(self.plot, 1)
        options = QtWidgets.QHBoxLayout()
        self.infinite = QtWidgets.QCheckBox("无限回放")
        self.count = QtWidgets.QSpinBox()
        self.count.setRange(1, 1000000)
        self.interval = QtWidgets.QDoubleSpinBox()
        self.interval.setRange(0, 86400)
        self.interval.setDecimals(2)
        self.interval.setSuffix(" 秒")
        for widget in (self.infinite, QtWidgets.QLabel("回放次数："), self.count,
                       QtWidgets.QLabel("每次回放间隔："), self.interval):
            options.addWidget(widget)
        layout.addLayout(options)
        self.initial_info = QtWidgets.QLabel("过渡自动插值 · 原速回放 · 控制周期 2 ms（500 Hz）")
        layout.addWidget(self.initial_info)
        buttons = QtWidgets.QHBoxLayout()
        self.gravity_button = QtWidgets.QPushButton("启动重力补偿")
        self.record_button = QtWidgets.QPushButton("开始记录（自动命名）")
        self.record_button.setEnabled(False)
        self.finish_record_button = QtWidgets.QPushButton("停止记录并保存")
        self.finish_record_button.setEnabled(False)
        self.play_button = QtWidgets.QPushButton("开始回放")
        self.stop_play_button = QtWidgets.QPushButton("停止回放（保留重力补偿）")
        self.stop_play_button.setEnabled(False)
        self.stop_button = QtWidgets.QPushButton("停止全部（进入阻尼）")
        self.stop_button.setEnabled(False)
        for button in (self.gravity_button, self.record_button, self.finish_record_button):
            buttons.addWidget(button)
        layout.addLayout(buttons)
        playback_buttons = QtWidgets.QHBoxLayout()
        playback_buttons.addWidget(self.play_button)
        playback_buttons.addWidget(self.stop_play_button)
        playback_buttons.addWidget(self.stop_button)
        layout.addLayout(playback_buttons)
        self.progress = QtWidgets.QProgressBar()
        layout.addWidget(self.progress)
        self.completed = QtWidgets.QLabel("已完成：0 次")
        self.state = QtWidgets.QLabel("就绪")
        self.live = QtWidgets.QLabel("关节位置：—")
        layout.addWidget(self.completed)
        layout.addWidget(self.state)
        layout.addWidget(self.live)
        layout.addWidget(QtWidgets.QLabel("先启动重力补偿，再记录或回放；结束记录/回放后继续重力补偿。停止全部或关闭窗口会结束重力补偿并进入阻尼。"))
        self.worker = RobotWorker()
        self.worker.status.connect(self.state.setText)
        self.worker.error.connect(self.show_error)
        self.worker.progress.connect(self.update_progress)
        self.worker.sample.connect(self.update_sample)
        self.worker.done.connect(self.task_done)
        self.worker.gravity_ready.connect(self.gravity_started)
        self.worker.recording_changed.connect(self.recording_changed)
        self.worker.playback_finished.connect(self.playback_finished)
        self.worker.state_snapshot.connect(self.update_snapshot)
        self.worker.playback_plan.connect(self.show_playback_plan)
        self.worker.finished.connect(self.close_after_shutdown)
        self.worker.start()
        self.refresh.clicked.connect(self.refresh_files)
        self.rename_button.clicked.connect(self.rename_trajectory)
        self.delete_button.clicked.connect(self.delete_trajectory)
        self.concatenate_button.clicked.connect(self.concatenate)
        self.files.currentIndexChanged.connect(self.preview)
        self.infinite.toggled.connect(lambda _: self.update_controls())
        self.record_button.clicked.connect(self.start_record)
        self.gravity_button.clicked.connect(self.start_gravity)
        self.finish_record_button.clicked.connect(self.finish_record)
        self.play_button.clicked.connect(self.start_play)
        self.stop_play_button.clicked.connect(self.stop_play)
        self.stop_button.clicked.connect(self.stop)
        self.refresh_files()

    def refresh_files(self):
        selected = self.files.currentData()
        self.files.blockSignals(True)
        self.files.clear()
        for path in sorted(TRAJECTORY_DIR.glob("*.jsonl"), reverse=True):
            if path.is_file():
                self.files.addItem(path.name, str(path))
        index = self.files.findData(selected)
        self.files.setCurrentIndex(max(0, index))
        self.files.blockSignals(False)
        self.preview()

    def preview(self):
        path = self.files.currentData()
        self.preview_frames = []
        self.preview_stages = []
        self.initial_transition = 2.0
        try:
            frames = load_frames(path) if path else []
            stages = []
            if path:
                recipe = Path(path).with_suffix(".sequence.json")
                try:
                    data = json.loads(recipe.read_text(encoding="utf-8"))
                    self.initial_transition = recipe_modules(data)[0]["duration"]
                    candidates = data.get("stages", [])
                    for stage in candidates:
                        if (0 <= stage["start"] <= stage["end"] <= frames[-1]["t"]
                                and isinstance(stage["label"], str) and isinstance(stage["kind"], str)):
                            stages.append(stage)
                except (OSError, ValueError, KeyError, TypeError, IndexError):
                    stages = []
            if frames:
                stages = stages or [{"start": 0, "end": frames[-1]["t"], "kind": "trajectory", "label": "轨迹"}]
                self.plot.set_frames(*initial_stage_preview(frames, stages, self.initial_transition, self.last_robot_frame))
            else:
                self.plot.set_frames([])
            self.preview_frames = frames
            self.preview_stages = stages
            self.info.setText(f"{len(frames)} 帧 · 时长 {frames[-1]['t']:.2f} 秒" if frames else "暂无轨迹")
        except Exception as exc:
            self.plot.set_frames([])
            self.info.setText(f"无法读取轨迹：{exc}")
        self.update_controls()
        self.initial_info.setText("过渡自动插值 · 原速回放 · 控制周期 2 ms（500 Hz）")

    def set_busy(self, busy):
        self.busy = busy
        self.update_controls()

    def update_controls(self):
        editable = self.mode in (None, "gravity")
        gravity = self.mode == "gravity" and not self.worker.cancel.is_set()
        for widget in (self.files, self.refresh, self.infinite, self.interval):
            widget.setEnabled(editable)
        self.gravity_button.setEnabled(not self.busy)
        self.record_button.setEnabled(gravity)
        self.finish_record_button.setEnabled(self.mode == "record")
        self.count.setEnabled(editable and not self.infinite.isChecked())
        self.play_button.setEnabled(gravity and bool(self.preview_frames))
        self.stop_play_button.setEnabled(self.mode == "play")
        self.stop_button.setEnabled(self.busy and self.mode != "stopping")
        self.rename_button.setEnabled(editable and bool(self.files.currentData()))
        self.delete_button.setEnabled(editable and bool(self.files.currentData()))
        self.concatenate_button.setEnabled(editable)

    def concatenate(self):
        if self.mode not in (None, "gravity"):
            return
        dialog = ConcatenateDialog(self)
        if dialog.exec_() == QtWidgets.QDialog.Accepted:
            self.refresh_files()
            self.files.setCurrentIndex(self.files.findData(str(dialog.output_path)))
            self.state.setText(f"拼接已保存：{dialog.output_path.name}")

    def rename_trajectory(self):
        selected = self.files.currentData()
        if self.mode not in (None, "gravity") or not selected:
            return
        source = Path(selected)
        name, accepted = QtWidgets.QInputDialog.getText(
            self, "修改轨迹名称", "新名称（自动保留 .jsonl 后缀）：",
            text=source.stem)
        if not accepted:
            return
        name = name.strip()
        if name.endswith(".jsonl"):
            name = name[:-6]
        if not name.strip() or name in (".", "..") or any(c in name for c in '/\\\0\n\r'):
            self.show_error("请输入有效的文件名，不能包含路径分隔符或换行。")
            return
        target = TRAJECTORY_DIR / (name + ".jsonl")
        if target == source:
            return
        if target.exists() or target.is_symlink():
            self.show_error(f"名称已存在：{target.name}，请使用其他名称。")
            return
        try:
            source.rename(target)
        except OSError as exc:
            self.show_error(f"修改名称失败：{exc}")
            return
        self.refresh_files()
        self.files.setCurrentIndex(self.files.findData(str(target)))
        self.state.setText(f"已改名：{target.name}")

    def delete_trajectory(self):
        selected = self.files.currentData()
        if self.mode not in (None, "gravity") or not selected:
            return
        path = Path(selected)
        answer = QtWidgets.QMessageBox.question(
            self, "删除轨迹", f"确定删除轨迹 {path.name}？\n删除后无法恢复。",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No)
        if answer != QtWidgets.QMessageBox.Yes:
            return
        try:
            path.unlink()
        except OSError as exc:
            self.show_error(f"删除失败：{exc}")
            self.refresh_files()
            return
        self.refresh_files()
        self.state.setText(f"已删除：{path.name}")

    def start_gravity(self):
        if self.busy:
            return
        self.mode = "gravity_starting"
        self.set_busy(True)
        self.worker.submit(("gravity",))

    def gravity_started(self):
        if self.mode == "gravity_starting" and not self.worker.cancel.is_set():
            self.mode = "gravity"
            self.update_controls()

    def start_record(self):
        if self.mode != "gravity" or self.worker.cancel.is_set():
            return
        path = TRAJECTORY_DIR / datetime.now().strftime("trajectory_%Y%m%d_%H%M%S_%f.jsonl")
        self.mode = "record_pending"
        self.update_controls()
        self.update_progress(0, 0)
        self.worker.record_commands.put(("start", path))

    def finish_record(self):
        if self.mode != "record":
            return
        self.mode = "record_stopping"
        self.update_controls()
        self.worker.record_commands.put(("stop", None))

    def recording_changed(self, recording, path):
        if recording:
            if self.mode == "record_pending" and not self.worker.cancel.is_set():
                self.mode = "record"
                self.plot.set_frames([])
                self.finish_record_button.setEnabled(True)
        else:
            self.finish_record_button.setEnabled(False)
            self.refresh_files()
            self.files.setCurrentIndex(self.files.findData(path))
            if self.mode in ("record", "record_stopping") and not self.worker.cancel.is_set():
                self.mode = "gravity"
        self.update_controls()

    def start_play(self):
        if self.mode != "gravity" or self.worker.cancel.is_set():
            return
        # 在界面线程读入文件，控制线程继续发送重力补偿，不等待文件 I/O。
        self.preview()
        if not self.preview_frames:
            return
        try:
            playback_frames = prepare_playback_frames(self.preview_frames, self.preview_stages)
        except Exception as exc:
            self.show_error(f"准备回放失败：{exc}")
            return
        self.info.setText(f"原速回放 · 轨迹时长 {playback_frames[-1]['t']:.2f} 秒 · "
                         "起始过渡自动生成，进度包含起始过渡")
        self.mode = "play"
        self.update_progress(0, 0)
        self.update_controls()
        self.worker.play_cancel.clear()
        self.worker.record_commands.put(("play", (playback_frames,
            0 if self.infinite.isChecked() else self.count.value(), self.interval.value(),
            self.initial_transition, self.preview_stages)))

    def update_snapshot(self, frame):
        self.last_robot_frame = frame

    def show_playback_plan(self, frames, stages):
        if self.mode == "play":
            self.plot.set_frames(frames, stages)
            self.info.setText(f"本轮总时长 {frames[-1]['t']:.2f} 秒（含起始过渡，不含循环间隔）")

    def stop_play(self):
        if self.mode != "play":
            return
        self.mode = "play_stopping"
        self.worker.play_cancel.set()
        self.update_controls()
        self.state.setText("正在停止回放，恢复重力补偿…")

    def playback_finished(self):
        if self.mode in ("play", "play_stopping") and not self.worker.cancel.is_set():
            self.mode = "gravity"
            self.update_controls()

    def stop(self):
        self.mode = "stopping"
        self.worker.cancel.set()
        self.update_controls()
        self.state.setText("正在停止全部并进入阻尼模式…")

    def update_progress(self, percent, completed):
        self.progress.setValue(percent)
        self.completed.setText(f"已完成：{completed} 次")

    def update_sample(self, timestamp, positions):
        self.live.setText(f"{timestamp:.2f} 秒  " + "  ".join(
            f"J{i + 1}: {p:.3f}" for i, p in enumerate(positions)))
        if self.mode == "record":
            self.plot.append(timestamp, positions)

    def show_error(self, message):
        QtWidgets.QMessageBox.critical(self, "任务错误", message)

    def task_done(self):
        self.mode = None
        self.last_robot_frame = None
        self.set_busy(False)
        self.refresh_files()

    def closeEvent(self, event):
        if self.worker.isRunning():
            self.closing = True
            self.setEnabled(False)
            self.state.setText("正在进入阻尼模式并关闭…")
            self.worker.shutdown()
            event.ignore()
        else:
            event.accept()

    def close_after_shutdown(self):
        if self.closing:
            self.close()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = TrajectoryWindow()
    window.show()
    sys.exit(app.exec_())
