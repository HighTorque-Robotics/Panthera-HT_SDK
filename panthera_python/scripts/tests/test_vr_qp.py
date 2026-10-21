"""Offline VR/QP integration tests; native motor SDK is replaced before import.

Run: python -m unittest discover -s tests -p 'test_vr_qp.py' -v
"""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

import numpy as np
import pinocchio as pin
import yaml

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))


class NoHardwareRobot:
    def __init__(self, *args, **kwargs):
        raise AssertionError('Hardware initialization is forbidden in offline tests')


native_stub = types.ModuleType('hightorque_robot')
native_stub.Robot = NoHardwareRobot
def load_script(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


with patch.dict(sys.modules, {'hightorque_robot': native_stub}):
    from Panthera_lib import Panthera
    from Panthera_lib.qp_ik import PinocchioOSQPIK
    single = load_script('7_vr_cartesian_control')
    dual = load_script('7_vr_dual_cartesian_control')

assert single.PinocchioOSQPIK is dual.PinocchioOSQPIK is PinocchioOSQPIK
assert Panthera.__bases__ == (NoHardwareRobot,)


class OfflineArm:
    """Real SDK kinematics on the local URDF, with in-memory motor commands."""
    forward_kinematics = Panthera.forward_kinematics
    get_jacobian = Panthera.get_jacobian
    gripper_control_MIT = Panthera.gripper_control_MIT
    pos_vel_tqe_kp_kd = Panthera.pos_vel_tqe_kp_kd
    get_Gravity = Panthera.get_Gravity
    get_friction_compensation = Panthera.get_friction_compensation

    def __init__(self, config='Follower.yaml'):
        path = SCRIPTS.parent / 'robot_param' / config
        cfg = yaml.safe_load(path.read_text())
        self.model = pin.buildModelFromUrdf(str(path.parent / cfg['urdf']['file_path']))
        self.data = self.model.createData()
        self.motor_count = 6
        self.joint_names = cfg['kinematics']['joint_names']
        self.joint_ids = [self.model.getJointId(n) for n in self.joint_names]
        self.end_effector_frame_id = self.model.getFrameId(cfg['urdf']['end_effector_link'])
        self.joint_limits = {k: np.asarray(v) for k, v in cfg['robot']['joint_limits'].items()}
        self.velocity_limits = np.asarray(cfg['robot']['velocity_limits'])
        self.acceleration_limits = np.asarray(cfg['robot']['acceleration_limits'])
        self.gripper_limits = cfg['robot']['gripper_limits']
        self.gripper_id = 7
        self.q = np.array([0., .6, .6, 0., 0., 0.])
        self.pending = [None] * 7
        self.sent = []
        self.Motors = [types.SimpleNamespace(pos_vel_tqe_kp_kd=self.record_motor(i)) for i in range(7)]

    def record_motor(self, index):
        def record(*args):
            self.pending[index] = tuple(args)
        return record

    def motor_send_cmd(self):
        self.sent.append(tuple(self.pending))

    def get_current_pos(self):
        return self.q.copy()

    def get_current_vel(self):
        return np.zeros(6)


def hand(seq=1, squeeze=1., offset=0., valid=1):
    return dict(seq=seq, pose_valid=valid, pos=(.2 + offset, .3, .4),
                rot=(0., 0., 0., 1.), squeeze=squeeze, trigger=.2,
                primary=0, secondary=0, thumbstick=(0., 0.), received_at=seq * .01)


class VRQPTests(unittest.TestCase):
    def test_real_models_qp_convergence_and_limits(self):
        for config in ('Leader.yaml', 'Follower.yaml'):
            with self.subTest(config=config):
                robot = OfflineArm(config)
                solver = PinocchioOSQPIK(robot)
                q = robot.q.copy()
                goal = q + np.array([.04, .03, -.02, .02, -.02, .01])
                target = robot.forward_kinematics(goal)['transform']
                previous = np.zeros(6)
                for _ in range(300):
                    result = solver.step(q, target, .01)
                    self.assertLessEqual(np.max(np.abs(result.joint_velocity)), .8 + 1e-9)
                    self.assertLessEqual(np.max(np.abs(result.joint_velocity - previous)), .02 + 1e-9)
                    self.assertTrue(np.all(result.joint_position >= solver.lower))
                    self.assertTrue(np.all(result.joint_position <= solver.upper))
                    q, previous = result.joint_position, result.joint_velocity
                self.assertLess(result.position_error, 1e-3)
                self.assertLess(result.rotation_error, 1e-3)

    def test_gripper_batch_and_default_send(self):
        robot = OfflineArm()
        self.assertTrue(robot.gripper_control_MIT(.8, 0, 0, 3, .3, send=False))
        self.assertEqual(len(robot.sent), 0)
        self.assertTrue(robot.pos_vel_tqe_kp_kd(robot.q, [0]*6, [0]*6, [1]*6, [1]*6))
        self.assertEqual(len(robot.sent), 1)
        self.assertEqual(robot.sent[-1][-1][0], .8)
        robot.gripper_control_MIT(.5, 0, 0, 3, .3)
        self.assertEqual(len(robot.sent), 2)
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertFalse(robot.gripper_control_MIT(3, 0, 0, 3, .3, send=False))
        self.assertEqual(robot.pending[-1][0], .5)

    def test_dual_independence_duplicates_release_and_invalid(self):
        left = dual.create_arm_state(OfflineArm('Leader.yaml'), 'Leader')
        right = dual.create_arm_state(OfflineArm(), 'Follower')
        self.assertIsNot(left['qp_solver'], right['qp_solver'])
        initial_right = np.asarray(right['target_position']).copy()
        with contextlib.redirect_stdout(io.StringIO()):
            dual.process_arm_control(left, hand())
            dual.process_arm_control(left, hand(2, offset=.01))
            target = left['target_position'].copy()
            for _ in range(3):
                dual.process_arm_control(left, hand(2, offset=.01))
            np.testing.assert_array_equal(left['target_position'], target)
            np.testing.assert_array_equal(right['target_position'], initial_right)
            self.assertGreater(np.linalg.norm(left['last_command_joint_velocity']), 0)
            dual.process_arm_control(left, hand(3, squeeze=0, offset=.01))
            self.assertFalse(left['control_activated'])
            np.testing.assert_allclose(left['last_valid_joint_pos'], left['robot'].q, atol=1e-7)
            dual.process_arm_control(left, hand(4))
            dual.process_arm_control(left, hand(5, valid=0))
            self.assertFalse(left['control_activated'])
            np.testing.assert_array_equal(left['qp_solver'].previous_velocity, np.zeros(6))
        self.assertTrue(all(frame[i][1] == 0 for frame in left['robot'].sent for i in range(6) if frame[i]))

    def test_dual_qp_failure_retains_command_and_resets_velocity(self):
        state = dual.create_arm_state(OfflineArm(), 'Follower')
        with contextlib.redirect_stdout(io.StringIO()):
            dual.process_arm_control(state, hand())
            previous = state['last_valid_joint_pos'].copy()
            with patch.object(state['qp_solver'], 'step', side_effect=RuntimeError('injected')):
                dual.process_arm_control(state, hand(2, offset=.01))
        np.testing.assert_array_equal(state['last_valid_joint_pos'], previous)
        np.testing.assert_array_equal(state['qp_solver'].previous_velocity, np.zeros(6))
        self.assertIsNotNone(state['robot'].sent[-1][-1])

    def test_single_control_loop_with_local_sdk(self):
        self.run_single_loop(fail_qp=False)

    def test_single_qp_failure_flushes_gripper(self):
        self.run_single_loop(fail_qp=True)

    def run_single_loop(self, fail_qp):
        robot = OfflineArm()
        output = io.StringIO()
        captured = []
        ticks = [0]
        single.running = True
        single.control_activated = False
        single.right_hand_data = hand()

        def fake_sleep(duration):
            ticks[0] += 1
            i = ticks[0]
            single.right_hand_data = hand(i+1, squeeze=0 if i == 3 else 1,
                                          offset=.001*i, valid=0 if i == 4 else 1)
            if i >= 5:
                single.running = False

        # main() executes the actual controller, but no threads, sockets, or motors.
        with contextlib.ExitStack() as stack:
            stack.enter_context(contextlib.redirect_stdout(output))
            stack.enter_context(patch.object(single, 'Panthera', return_value=robot))
            stack.enter_context(patch.object(single.threading, 'Thread'))
            stack.enter_context(patch.object(single, 'move_to_safe_position', return_value=True))
            stack.enter_context(patch.object(single, 'execute_smooth_joint_reset', return_value=True))
            stack.enter_context(patch.object(single, 'save_and_plot_tracking', side_effect=lambda s: captured.extend(s)))
            stack.enter_context(patch.object(single.time, 'sleep', side_effect=fake_sleep))
            if fail_qp:
                stack.enter_context(patch.object(PinocchioOSQPIK, 'step', side_effect=RuntimeError('test QP failure')))
            single.main()
        self.assertNotIn('错误:', output.getvalue())
        self.assertGreaterEqual(len(captured), 5)
        self.assertGreaterEqual(len(robot.sent), 5)
        if fail_qp:
            # Error messages are throttled; assert the actual flush behavior.
            self.assertTrue(any(frame[0] is None and frame[-1] is not None
                                for frame in robot.sent))
            self.assertTrue(all(np.linalg.norm(s['target_qdot']) == 0
                                for s in captured))
        else:
            self.assertNotIn('QP求解失败', output.getvalue())
            self.assertTrue(any(np.linalg.norm(s['target_qdot']) > 0 for s in captured))


if __name__ == '__main__':
    unittest.main()
