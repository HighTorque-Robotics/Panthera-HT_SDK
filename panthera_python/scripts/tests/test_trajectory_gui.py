"""无硬件测试：python -m unittest discover -s scripts/tests -p test_trajectory_gui.py"""
import importlib.util
import ast
import contextlib
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import numpy as np
from PyQt5 import QtWidgets

SCRIPTS = Path(__file__).resolve().parents[1]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


gui = load_module("trajectory_gui_test_target", SCRIPTS / "5_trajectory_gui.py")
recorder = load_module("trajectory_recorder_test_target", SCRIPTS / "Panthera_lib/recorder.py")
APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class FakeRobot:
    motor_count = 6
    has_gripper = True
    joint_limits = {"lower": [-3.0]*6, "upper": [3.0]*6}
    gripper_limits = {"lower": 0.0, "upper": 2.0}
    GRAVITY = np.array([1.0, .8, .6, .4, .2, .1])

    def __init__(self, *args):
        self.positions = [.1]*6
        self.gripper = .1
        self.commands = []
        self.stop_count = 0
        self.brake_count = 0

    def get_current_pos(self): return list(self.positions)
    def get_current_vel(self): return [0.0]*6
    def get_current_pos_gripper(self): return self.gripper
    def get_current_vel_gripper(self): return 0.0
    def get_Gravity(self, pos): return self.GRAVITY.copy()
    def get_friction_compensation(self, *args): return np.zeros(6)
    def send_get_motor_state_cmd(self): pass
    def set_stop(self): self.stop_count += 1
    def set_brake(self): self.brake_count += 1

    def pos_vel_tqe_kp_kd(self, pos, vel, torque, kp, kd):
        self.commands.append((time.perf_counter(), list(pos), list(vel), list(kp), list(kd), list(torque)))
        if any(kp):
            self.positions = list(pos)

    def gripper_control_MIT(self, pos, vel, torque, kp, kd, send=True):
        if kp:
            self.gripper = pos

    def Joint_Pos_Vel(self, *args, **kwargs):
        raise AssertionError("不允许切换到位置速度控制模式")

    def gripper_control(self, *args, **kwargs):
        raise AssertionError("夹爪也必须始终使用 MIT")


class NoGripperRobot(FakeRobot):
    has_gripper = False

    def get_current_pos_gripper(self):
        raise AssertionError("无夹爪模式不应读取夹爪位置")

    def get_current_vel_gripper(self):
        raise AssertionError("无夹爪模式不应读取夹爪速度")

    def gripper_control_MIT(self, *args, **kwargs):
        raise AssertionError("无夹爪模式不应发送夹爪命令")


def load_mit_bindings():
    """执行库中实际的两个 MIT 方法，避开机器人构造和原生 SDK 导入。"""
    path = SCRIPTS / "Panthera_lib/Panthera.py"
    tree = ast.parse(path.read_text())
    robot_class = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "Panthera")
    methods = [node for node in robot_class.body if isinstance(node, ast.FunctionDef)
               and node.name in ("pos_vel_tqe_kp_kd", "gripper_control_MIT", "set_brake")]
    namespace = {"np": np}
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), "exec"), namespace)
    return namespace


MIT_BINDINGS = load_mit_bindings()


class BufferedRobot(FakeRobot):
    """使用真实 Python 包装接口，捕获 motor_send_cmd 提交的整组命令。"""
    pos_vel_tqe_kp_kd = MIT_BINDINGS["pos_vel_tqe_kp_kd"]
    gripper_control_MIT = MIT_BINDINGS["gripper_control_MIT"]

    def __init__(self, *args):
        super().__init__(*args)
        self.gripper_id = 7
        self.pending = [None] * 7
        self.sent = []
        self.Motors = [types.SimpleNamespace(pos_vel_tqe_kp_kd=self.stage_motor(i)) for i in range(7)]
        self.gripper_motor = self.Motors[-1]

    def stage_motor(self, index):
        def stage(*args):
            self.pending[index] = tuple(args)
        return stage

    def motor_send_cmd(self):
        packet = tuple(self.pending)
        self.sent.append(packet)
        # 即使 SDK 每次发送后清空缓存，也必须提交完整的七电机命令。
        self.pending = [None] * 7
        if all(command is not None for command in packet[:6]):
            arm = list(zip(*packet[:6]))
            FakeRobot.pos_vel_tqe_kp_kd(self, *arm)
        if packet[-1] is not None:
            FakeRobot.gripper_control_MIT(self, *packet[-1], send=False)


def frames(position=.1):
    return [{"t": t, "pos": [position]*6, "vel": [0.0]*6,
             "gripper_pos": position, "gripper_vel": 0.0} for t in (0.0, .02, .04)]


def make_worker():
    worker = gui.RobotWorker()
    worker.robot = FakeRobot()
    worker.np = np
    worker.recorder_class = recorder.Recorder
    return worker


def wait_until(condition, timeout=4):
    deadline = time.perf_counter() + timeout
    while not condition() and time.perf_counter() < deadline:
        APP.processEvents()
        time.sleep(.005)
    if not condition():
        raise AssertionError("等待状态转换超时")


def pump(seconds):
    deadline = time.perf_counter() + seconds
    while time.perf_counter() < deadline:
        APP.processEvents()
        time.sleep(.005)


class TrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.directory = Path(self.temp.name)
        self.directory_patch = patch.object(gui, "TRAJECTORY_DIR", self.directory)
        self.directory_patch.start()
        self.addCleanup(self.temp.cleanup)
        self.addCleanup(self.directory_patch.stop)

    def write_frames(self, name, data):
        path = self.directory / name
        path.write_text("\n".join(json.dumps(f) for f in data), encoding="utf-8")
        return path

    def test_mit_interpolation_starts_from_actual_pose(self):
        worker = make_worker()
        target = frames(.12)[0]
        worker.move_to_start(target, .01)
        commands = worker.robot.commands
        self.assertGreater(len(commands), 10)
        self.assertAlmostEqual(commands[0][1][0], .1, places=5)
        self.assertAlmostEqual(commands[-1][1][0], .12)
        self.assertTrue(all(a[1][0] <= b[1][0] for a, b in zip(commands, commands[1:])))
        self.assertTrue(all(c[3] == gui.KP and c[4] == gui.KD for c in commands))
        self.assertAlmostEqual(worker.robot.gripper, .12)
        self.assertEqual(worker.robot.stop_count, 0)

    def test_finite_repeats_and_hold_interval_send_mit(self):
        worker = make_worker()
        events = []
        worker.status.connect(lambda message: events.append((time.perf_counter(), message)))
        worker.play(frames(), 2, .05, .01)
        starts = [t for t, message in events if "正在回放第" in message]
        self.assertEqual(len(starts), 2)
        self.assertGreaterEqual(starts[1] - starts[0], .095)
        # 间隔期间仍有 MIT 指令，不能仅 sleep 等待。
        gap_start = next(t for t, message in events if "间隔" in message)
        self.assertGreater(len([c for c in worker.robot.commands if gap_start <= c[0] < gap_start + .045]), 5)
        self.assertEqual(worker.robot.stop_count, 0)

    def test_stop_play_during_initial_transition(self):
        worker = make_worker()
        worker.playing = True
        timer = threading.Timer(.03, worker.play_cancel.set)
        timer.start()
        try:
            with self.assertRaises(gui.PlaybackCancelled):
                worker.play(frames(.3), 1, 0, .1)
        finally:
            timer.join()
        self.assertLess(worker.robot.positions[0], .3)
        self.assertEqual(worker.robot.stop_count, 0)

    def test_interpolated_sampling_preserves_hold(self):
        data = frames()
        data[1]["t"] = 1.0
        data[2]["t"] = 2.0
        data[2]["pos"] = [.3]*6
        holding = gui.RobotWorker.sample_frame(data, [0, 1, 2], .5)
        moving = gui.RobotWorker.sample_frame(data, [0, 1, 2], 1.5)
        self.assertEqual(holding["pos"], [.1]*6)
        self.assertEqual(holding["vel"], [0.0]*6)
        self.assertTrue(all(abs(v-.2) < 1e-9 for v in moving["pos"]))

    def test_prepared_velocity_is_derivative_of_same_smooth_position(self):
        source = [{"t": i*.011, "pos": [.1 + .02*np.sin(i*.2)]*6,
                   "vel": [(-1)**i * 10]*6,
                   "gripper_pos": .1 + .02*np.sin(i*.2), "gripper_vel": (-1)**i * 10}
                  for i in range(30)]
        original = json.dumps(source)
        prepared = gui.prepare_playback_frames(source)
        times = [f["t"] for f in prepared]
        self.assertEqual(json.dumps(source), original)
        self.assertEqual(prepared[0]["pos"], source[0]["pos"])
        self.assertEqual(prepared[-1]["pos"], source[-1]["pos"])
        self.assertEqual(prepared[0]["vel"], [0]*6)
        self.assertEqual(prepared[-1]["vel"], [0]*6)
        for elapsed in np.linspace(.001, times[-1]-.001, 100):
            before = gui.RobotWorker.sample_frame(prepared, times, elapsed-1e-6)
            after = gui.RobotWorker.sample_frame(prepared, times, elapsed+1e-6)
            sampled = gui.RobotWorker.sample_frame(prepared, times, elapsed)
            derivative = (np.array(after["pos"])-np.array(before["pos"]))/2e-6
            np.testing.assert_allclose(sampled["vel"], derivative, atol=1e-5)
            self.assertAlmostEqual(sampled["gripper_vel"],
                (after["gripper_pos"]-before["gripper_pos"])/2e-6, places=5)
        for elapsed in times[1:-1]:
            before = gui.RobotWorker.sample_frame(prepared, times, elapsed-1e-8)
            after = gui.RobotWorker.sample_frame(prepared, times, elapsed+1e-8)
            np.testing.assert_allclose(before["vel"], after["vel"], atol=1e-5)

    def test_smoothing_preserves_original_duration_hold_and_curve_bounds(self):
        source = [{"t": t, "pos": [q]*6, "vel": [99]*6} for t, q in
                  ((0,.1), (.1,.2), (1.1,.2), (1.2,.3))]
        prepared = gui.prepare_playback_frames(source)
        self.assertEqual(prepared[-1]["t"], source[-1]["t"])
        start = next(i for i,f in enumerate(prepared) if abs(f["t"]-.1) < 1e-9)
        end = next(i for i,f in enumerate(prepared) if abs(f["t"]-1.1) < 1e-9)
        self.assertAlmostEqual(prepared[end]["t"]-prepared[start]["t"], 1.0)
        times = [f["t"] for f in prepared]
        for elapsed in np.linspace(0, times[-1], 100):
            sample = gui.RobotWorker.sample_frame(prepared, times, elapsed)
            self.assertTrue(all(.1-1e-12 <= p <= .3+1e-12 for p in sample["pos"]))

    def test_short_trajectory_avoids_tiny_final_resampling_interval(self):
        source = [{"t": t, "pos": [q]*6} for t, q in ((0,.1), (.010001,.11))]
        prepared = gui.prepare_playback_frames(source)
        dt = np.diff([f["t"] for f in prepared])
        self.assertGreater(dt.min(), .004)
        self.assertEqual(prepared[-1]["t"], source[-1]["t"])

    def test_late_control_cycle_keeps_grid_without_catchup_burst(self):
        worker = make_worker()
        with patch.object(gui.time, "perf_counter", return_value=1.101), patch.object(worker, "pause") as pause:
            target = worker.wait_next_tick(1.0)
            self.assertAlmostEqual(target, 1.102)
            self.assertAlmostEqual(pause.call_args.args[0], .001)

    def test_playback_uses_original_timeline_despite_scheduler_delay(self):
        worker = make_worker()
        clock = [10.0]
        count = [0]
        data = [{"t": t, "pos": [q]*6, "vel": [0.0]*6}
                for t, q in ((0,.1), (.1,.2))]
        def next_tick(previous):
            count[0] += 1
            clock[0] += .03 if count[0] == 3 else gui.CONTROL_DT
            return clock[0]
        with patch.object(gui.time, "perf_counter", side_effect=lambda: clock[0]), \
                patch.object(worker, "move_to_start", return_value=0.0), patch.object(worker, "wait_next_tick", side_effect=next_tick):
            worker.play(data, 1, 0)
        self.assertLessEqual(abs(clock[0]-10.0-.1), gui.CONTROL_DT)
        for command in worker.robot.commands:
            elapsed = min(command[0]-10.0, .1)
            expected = gui.RobotWorker.sample_frame(data, [0, .1], elapsed)
            np.testing.assert_allclose(command[1], expected["pos"], atol=1e-12)
        times = [command[0] for command in worker.robot.commands]
        self.assertTrue(all(b > a for a, b in zip(times, times[1:])))

    def test_ui_does_not_retime_fast_trajectory(self):
        data = [{"t": t, "pos": [q]*6, "vel": [9.0]*6}
                for t, q in ((0,.1), (.1,.5))]
        self.write_frames("fast.jsonl", data)
        window = gui.TrajectoryWindow()
        try:
            window.worker.robot = FakeRobot()
            window.worker.robot.velocity_limits = [.001]*6
            window.worker.robot.acceleration_limits = [.001]*6
            window.mode = "gravity"
            window.busy = True
            window.start_play()
            command, payload = window.worker.record_commands.get_nowait()
            self.assertEqual(command, "play")
            self.assertEqual(payload[0][-1]["t"], .1)
            self.assertIn("原速回放", window.info.text())
        finally:
            window.close()
            window.worker.wait(2000)
            APP.processEvents()

    def test_playback_uses_actual_pose_gravity_without_friction_switching(self):
        worker = make_worker()
        calls = []
        def gravity(pos):
            calls.append(list(pos))
            return FakeRobot.GRAVITY.copy()
        worker.robot.get_Gravity = gravity
        with patch.object(worker.robot, "get_friction_compensation", side_effect=AssertionError("不能发送阈值摩擦补偿")):
            worker.send_frame(frames(.2)[0])
        self.assertEqual(calls, [[.1]*6])
        np.testing.assert_allclose(worker.robot.commands[-1][5], FakeRobot.GRAVITY)

    def test_sequence_phases_and_editable_preview(self):
        a = self.write_frames("a.jsonl", frames())
        b = self.write_frames("b.jsonl", frames(.2))
        data, stages = gui.concatenate_trajectories([(a,.5,1), (b,99,99)], with_stages=True)
        self.assertEqual([s["kind"] for s in stages], ["trajectory", "gap", "transition", "trajectory"])
        self.assertAlmostEqual(stages[1]["end"] - stages[1]["start"], .5)
        self.assertEqual(stages[-1]["end"], data[-1]["t"])
        self.assertTrue(all(a["end"] == b["start"] for a,b in zip(stages, stages[1:])))
        dialog = gui.ConcatenateDialog(None)
        try:
            dialog.insert_segment(0, str(a), .5, 1)
            dialog.insert_segment(2, str(b), 0, 1)
            dialog.update_preview()
            self.assertEqual([s["kind"] for s in dialog.plot.stages], ["start"]+[s["kind"] for s in stages])
            dialog.table.cellWidget(2, 0).findChild(QtWidgets.QDoubleSpinBox, "gap_duration").setValue(1.5)
            wait_until(lambda: dialog.plot.stages[2]["end"] - dialog.plot.stages[2]["start"] == 1.5)
            dialog.name.setText("saved")
            dialog.save()
            recipe = json.loads((self.directory / "saved.sequence.json").read_text())
            self.assertEqual(recipe["version"], 2)
            self.assertEqual(recipe["modules"][0], {"kind": "start", "duration": 2.0})
            reopened = gui.ConcatenateDialog(None)
            reopened.load_recipe(self.directory / "saved.sequence.json")
            reopened.update_preview()
            self.assertEqual(reopened.plot.stages, dialog.plot.stages)
            reopened.preview_timer.stop()
            reopened.close()
        finally:
            dialog.preview_timer.stop()
            dialog.close()

    def test_zero_module_and_initial_stage_roundtrip(self):
        a = self.write_frames("a.jsonl", frames(.2))
        b = self.write_frames("b.jsonl", frames(.3))
        dialog = gui.ConcatenateDialog(None)
        reopened = gui.ConcatenateDialog(None)
        window = None
        try:
            dialog.insert_segment(0, a)
            dialog.insert_segment(1, b)
            self.assertEqual([k for k, _ in dialog.row_bindings],
                             ["start", "trajectory", "transition", "trajectory"])
            dialog.table.selectRow(1)
            dialog.add_zero()
            self.assertEqual([k for k, _ in dialog.row_bindings],
                             ["start", "trajectory", "zero", "transition", "trajectory"])
            self.assertEqual(dialog.table.columnCount(), 2)
            self.assertIsNone(dialog.table.cellWidget(1, 1))
            dialog.table.cellWidget(0, 1).setValue(3.5)
            dialog.table.cellWidget(2, 1).setValue(2.5)
            dialog.table.cellWidget(3, 1).setValue(4.5)
            modules = dialog.modules()
            body, stages = gui.compile_sequence(modules)
            zero = next(s for s in stages if s["kind"] == "zero")
            endpoint = next(f for f in body if f["t"] == zero["end"])
            np.testing.assert_allclose(endpoint["pos"], [0]*6, atol=1e-12)
            self.assertAlmostEqual(endpoint["gripper_pos"], .2)
            self.assertEqual(zero["end"]-zero["start"], 2.5)
            self.assertTrue(all(x["t"] < y["t"] for x, y in zip(body, body[1:])))
            self.assertAlmostEqual(body[-1]["pos"][0], .3)
            unknown, labels = gui.initial_stage_preview(body, stages, 3.5)
            self.assertEqual(unknown[0]["t"], 3.5)
            self.assertEqual(labels[0]["kind"], "start")
            known, _ = gui.initial_stage_preview(body, stages, 3.5, frames(.1)[0])
            self.assertEqual(known[0]["pos"], [.1]*6)
            self.assertEqual(known[0]["t"], 0)
            self.assertEqual(known[-1]["t"], body[-1]["t"]+3.5)
            dialog.name.setText("zero_plan")
            dialog.save()
            reopened.load_recipe(self.directory / "zero_plan.sequence.json")
            self.assertEqual(reopened.modules(), modules)
            window = gui.TrajectoryWindow()
            window.files.setCurrentIndex(window.files.findData(str(dialog.output_path)))
            window.mode = "gravity"
            window.worker.robot = FakeRobot()
            window.start_play()
            command, payload = window.worker.record_commands.get_nowait()
            self.assertEqual(command, "play")
            self.assertEqual(payload[3], 3.5)
            self.assertEqual(payload[4], stages)
            self.assertEqual(payload[0][-1]["t"], body[-1]["t"])
            zero_target = next(f for f in payload[0] if f["t"] == zero["end"])
            np.testing.assert_allclose(zero_target["pos"], [0]*6, atol=1e-12)
            self.assertEqual(zero_target["vel"], [0]*6)
            # Zero is also allowed after the last trajectory.
            reopened.table.selectRow(4)
            reopened.add_zero()
            final, final_stages = gui.compile_sequence(reopened.modules())
            self.assertEqual(final_stages[-1]["kind"], "zero")
            np.testing.assert_allclose(final[-1]["pos"], [0]*6, atol=1e-12)
            self.assertAlmostEqual(final[-1]["gripper_pos"], .3)
        finally:
            for d in (dialog, reopened):
                d.preview_timer.stop()
                d.close()
            if window:
                window.close()
                window.worker.wait(2000)

    def test_gap_module_insert_move_delete_and_hold(self):
        a = self.write_frames("a.jsonl", frames(.2))
        b = self.write_frames("b.jsonl", frames(.3))
        dialog = gui.ConcatenateDialog(None)
        try:
            dialog.insert_segment(0, a)
            dialog.insert_segment(1, b)
            dialog.table.selectRow(1)
            dialog.add_gap()
            self.assertEqual([k for k, _ in dialog.row_bindings],
                             ["start", "trajectory", "gap", "transition", "trajectory"])
            spin = dialog.table.cellWidget(2, 0).findChild(QtWidgets.QDoubleSpinBox, "gap_duration")
            spin.setValue(1.7)
            body, stages = gui.compile_sequence(dialog.modules())
            gap = next(s for s in stages if s["kind"] == "gap")
            self.assertAlmostEqual(gap["end"]-gap["start"], 1.7)
            prepared = gui.prepare_playback_frames(body, stages)
            held = [f for f in prepared if gap["start"] <= f["t"] <= gap["end"]]
            self.assertGreater(len(held), 10)
            for f in held:
                np.testing.assert_allclose(f["pos"], [.2]*6)
                self.assertEqual(f["vel"], [0]*6)
                self.assertAlmostEqual(f["gripper_pos"], .2)
            dialog.table.selectRow(2)
            dialog.move_segment(1)
            self.assertEqual([a["kind"] for a in dialog.actions], ["trajectory", "trajectory", "gap"])
            dialog.remove_segment()
            self.assertEqual([a["kind"] for a in dialog.actions], ["trajectory", "trajectory"])
        finally:
            dialog.preview_timer.stop()
            dialog.close()

    def test_transition_preview_explains_automatic_duration(self):
        a = self.write_frames("a.jsonl", frames(.1))
        b = self.write_frames("b.jsonl", frames(2.1))
        dialog = gui.ConcatenateDialog(None)
        try:
            dialog.insert_segment(0, a)
            dialog.insert_segment(1, b)
            dialog.table.cellWidget(2, 1).setValue(2)
            dialog.update_preview()
            self.assertIn("实际 7.50 秒", dialog.table.item(2, 0).text())
            self.assertIn("按位移自动延长", dialog.preview_info.text())
            dialog.table.cellWidget(2, 1).setValue(9)
            dialog.update_preview()
            stage = next(s for s in dialog.plot.stages if s["kind"] == "transition")
            self.assertAlmostEqual(stage["end"]-stage["start"], 9)
            self.assertIn("实际 9.00 秒", dialog.table.item(2, 0).text())
            self.assertNotIn("按位移自动延长", dialog.table.item(2, 0).text())
        finally:
            dialog.preview_timer.stop()
            dialog.close()

    def test_transition_uses_yaml_limits_per_joint(self):
        config = self.directory / "limits.yaml"
        velocity = [1, .4, 1, 1, 1, 1]
        acceleration = [2, 2, .3, 2, 2, 2]
        config.write_text(json.dumps({"robot": {"velocity_limits": velocity,
                                              "acceleration_limits": acceleration}}))
        start = {"pos": [0]*6}
        end = {"pos": [.1, 2, 1, .1, .1, .1]}
        with patch.object(gui, "CONFIG_PATH", config):
            duration = gui.transition_duration(start, end, 0)
            self.assertAlmostEqual(duration, 1.875*2/(.4*1.1))
            for d, v, a in zip(end["pos"], velocity, acceleration):
                self.assertLessEqual(1.875*d/duration, v*1.1+1e-10)
                self.assertLessEqual((10/3**.5)*d/duration**2, a*1.1+1e-10)
            self.assertEqual(gui.transition_duration(start, end, 20), 20)
            end["pos"] = [0, 0, 2, 0, 0, 0]
            self.assertAlmostEqual(gui.transition_duration(start, end, 0),
                                   ((10/3**.5)*2/(.3*1.1))**.5)
        invalid = self.directory / "invalid.yaml"
        invalid.write_text(json.dumps({"robot": {"velocity_limits": [0]*6,
                                               "acceleration_limits": [2]*6}}))
        with patch.object(gui, "CONFIG_PATH", invalid), self.assertRaises(ValueError):
            gui.transition_duration(start, end)

    def test_actual_brake_wrapper_stages_all_seven_motors_before_send(self):
        events = []
        robot = types.SimpleNamespace(
            Motors=[types.SimpleNamespace(brake=lambda i=i: events.append(("brake", i))) for i in range(7)],
            motor_send_cmd=lambda: events.append(("send",)))
        self.assertTrue(MIT_BINDINGS["set_brake"](robot))
        self.assertEqual(events, [("brake", i) for i in range(7)]+[("send",)])
        # An unsupported motor must fail before mutating or sending the packet.
        events.clear()
        robot.Motors[-1] = types.SimpleNamespace()
        with self.assertRaises(RuntimeError):
            MIT_BINDINGS["set_brake"](robot)
        self.assertEqual(events, [])

    def test_idle_shutdown_enters_brake_without_stop(self):
        worker = make_worker()
        robot = worker.robot
        worker.start()
        worker.shutdown()
        self.assertTrue(worker.wait(2000))
        self.assertEqual(robot.brake_count, 1)
        self.assertEqual(robot.stop_count, 0)
        self.assertIsNone(worker.robot)

    def test_shutdown_without_robot(self):
        worker = gui.RobotWorker()
        worker.start()
        worker.shutdown()
        self.assertTrue(worker.wait(2000))

    def test_legacy_recipe_migration(self):
        old = {"version": 1, "segments": [
            {"file": "a.jsonl", "gap": .5, "transition": 3},
            {"file": "b.jsonl", "gap": 99, "transition": 99}]}
        self.assertEqual(gui.recipe_modules(old), [
            {"kind": "start", "duration": 2.0},
            {"kind": "trajectory", "file": "a.jsonl"},
            {"kind": "gap", "duration": .5},
            {"kind": "transition", "duration": 3.0},
            {"kind": "trajectory", "file": "b.jsonl"}])

    def test_limits_reject_before_playback_motion(self):
        worker = make_worker()
        with self.assertRaises(ValueError):
            worker.play(frames(4), 1, 0)
        self.assertFalse(worker.robot.commands)

    def assert_complete_playback_packet(self, packet):
        self.assertEqual(len(packet), 7)
        self.assertTrue(all(command is not None for command in packet))
        for index, command in enumerate(packet[:6]):
            self.assertAlmostEqual(command[2], FakeRobot.GRAVITY[index])
            self.assertEqual(command[3], gui.KP[index])
            self.assertEqual(command[4], gui.KD[index])

    def test_one_complete_transmission_per_playback_frame(self):
        worker = make_worker()
        worker.robot = BufferedRobot()
        for hold, with_gripper in ((False, True), (True, True), (False, False)):
            frame = frames()[0]
            if not with_gripper:
                del frame["gripper_pos"]
                del frame["gripper_vel"]
            count = len(worker.robot.sent)
            worker.send_frame(frame, hold=hold)
            self.assertEqual(len(worker.robot.sent), count + 1)
            self.assert_complete_playback_packet(worker.robot.sent[-1])

    def test_no_gripper_mode_skips_gripper_io_and_limits(self):
        worker = make_worker()
        worker.robot = NoGripperRobot()
        frame = dict(frames()[0], gripper_pos=99.0)

        pos, vel, gripper_pos, gripper_vel = worker.read_state()
        self.assertEqual(len(pos), 6)
        self.assertEqual(len(vel), 6)
        self.assertEqual((gripper_pos, gripper_vel), (0.0, 0.0))
        worker.validate_limits([frame])
        worker.send_frame(frame)
        self.assertEqual(len(worker.robot.commands), 1)

    def test_play_request_never_sends_gravity_only_before_playback(self):
        worker = make_worker()
        worker.robot = BufferedRobot()
        real_play = worker.play
        def play_and_end_session(*args):
            self.assertEqual(worker.robot.sent, [])
            real_play(*args)
            worker.cancel.set()
        worker.play = play_and_end_session
        worker.record_commands.put(("play", (frames(), 2, .03, .01)))
        with self.assertRaises(gui.Cancelled):
            worker.record()
        self.assertGreater(len(worker.robot.sent), 10)
        for packet in worker.robot.sent:
            self.assert_complete_playback_packet(packet)

    def test_rejected_mit_command_does_not_transmit_partial_packet(self):
        worker = make_worker()
        worker.robot = BufferedRobot()
        for frame in (dict(frames()[0], gripper_pos=3), dict(frames(4)[0], gripper_pos=.1)):
            with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(ValueError):
                worker.send_frame(frame)
            self.assertEqual(worker.robot.sent, [])
        worker.send_frame(frames()[0])
        self.assertEqual(len(worker.robot.sent), 1)
        self.assert_complete_playback_packet(worker.robot.sent[0])

    def test_active_session_record_play_cancel_and_close(self):
        source = self.write_frames("source.jsonl", frames())
        fake_sdk = types.SimpleNamespace(Panthera=BufferedRobot, TrajectoryRecorder=recorder.Recorder)
        with patch.dict(sys.modules, {"Panthera_lib": fake_sdk}):
            window = gui.TrajectoryWindow()
            errors = []
            window.show_error = errors.append
            # Replace the already-connected modal error handler for headless tests.
            window.worker.error.disconnect()
            window.worker.error.connect(errors.append)
            try:
                self.assertFalse(window.play_button.isEnabled())
                window.start_gravity()
                wait_until(lambda: window.mode == "gravity")
                robot = window.worker.robot
                self.assertTrue(window.record_button.isEnabled() and window.play_button.isEnabled())
                self.assertEqual(len(list(self.directory.glob("*.jsonl"))), 1)
                window.start_record()
                wait_until(lambda: window.mode == "record")
                pump(.06)
                window.finish_record()
                wait_until(lambda: window.mode == "gravity")
                self.assertEqual(len(list(self.directory.glob("*.jsonl"))), 2)
                self.assertEqual(robot.stop_count, 0)
                window.files.setCurrentIndex(window.files.findData(str(source)))
                window.count.setValue(2)
                window.interval.setValue(.03)
                window.start_play()
                wait_until(lambda: window.mode == "gravity", timeout=8)
                self.assertEqual(window.completed.text(), "已完成：2 次")
                self.assertEqual(robot.stop_count, 0)
                wait_until(lambda: robot.commands[-1][3] == [0]*6)
                self.assertTrue(any(c[3] == gui.KP for c in robot.commands))
                # Stop during infinite playback's interval, keep the same live session.
                window.infinite.setChecked(True)
                window.interval.setValue(30)
                window.start_play()
                wait_until(lambda: "间隔 30" in window.state.text())
                window.stop_play()
                wait_until(lambda: window.mode == "gravity")
                self.assertEqual(robot.stop_count, 0)
                # Stop all enters damping; the same worker can restart gravity afterwards.
                window.stop()
                wait_until(lambda: window.mode is None)
                self.assertEqual(robot.brake_count, 1)
                self.assertEqual(robot.stop_count, 0)
                self.assertTrue(window.worker.isRunning())
                window.start_gravity()
                wait_until(lambda: window.mode == "gravity")
                self.assertIs(window.worker.robot, robot)
                # Record another segment immediately, then close while recording.
                window.start_record()
                wait_until(lambda: window.mode == "record")
                pump(.04)
                window.close()
                wait_until(lambda: not window.worker.isRunning())
                self.assertEqual(robot.stop_count, 0)
                self.assertEqual(robot.brake_count, 2)
                self.assertEqual(len(list(self.directory.glob("*.jsonl"))), 3)
                self.assertTrue(all(gui.load_frames(p) for p in self.directory.glob("*.jsonl")))
                self.assertFalse(errors, errors)
                self.assertEqual(len(robot.commands), len(robot.sent))
                for packet in robot.sent:
                    self.assertTrue(all(command is not None for command in packet))
                    for index, command in enumerate(packet[:6]):
                        self.assertAlmostEqual(command[2], FakeRobot.GRAVITY[index])
                    gains = [command[3] for command in packet[:6]]
                    self.assertIn(gains, ([0.0]*6, gui.KP))
            finally:
                window.close()
                window.worker.wait(3000)
                APP.processEvents()


if __name__ == "__main__":
    unittest.main()
