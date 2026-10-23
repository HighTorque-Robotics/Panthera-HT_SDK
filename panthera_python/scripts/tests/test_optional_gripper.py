"""离线回归：替换 SDK Robot，绝不打开串口或发送硬件命令。"""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import tempfile
import types
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import yaml


SCRIPTS = Path(__file__).resolve().parents[1]
FOLLOWER_CONFIG = yaml.safe_load(
    (SCRIPTS.parent / 'robot_param/Follower.yaml').read_text()
)
GRIPPER_NAME = FOLLOWER_CONFIG['robot']['gripper_name']


class FakeMotor:
    def __init__(self, params, offline):
        self.name = params['name']
        self.id = params['id']
        self.online = self.name not in offline
        self.commands = []

    def get_motor_name(self):
        return self.name

    def get_motor_id(self):
        return self.id

    def get_motor_enum_type(self):
        return 'fake'

    def get_version(self):
        return types.SimpleNamespace(id=self.id if self.online else 0)

    def get_current_motor_state(self):
        return types.SimpleNamespace(position=self.id / 10, velocity=0, torque=0)

    def pos_vel_MAXtqe(self, *args):
        self.commands.append(args)

    def pos_vel_tqe_kp_kd(self, *args):
        self.commands.append(args)


class FakeRobot:
    offline = set()

    def __init__(self, config_path):
        with open(config_path, encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
        path = Path(config_path).parent / config['robot']['param_file']
        with open(path, encoding='utf-8') as stream:
            self.sdk_config = yaml.safe_load(stream)
        self.sdk_motors = []
        for board in self.sdk_config['robot']['CANboard'].values():
            for port in board['CANport'].values():
                assert port['motor_num'] == len(port['motor'])
                self.sdk_motors.extend(
                    FakeMotor(params, self.offline)
                    for params in port['motor'].values()
                )
        self.sent = 0
        self.zero_calls = 0

    def get_motors(self):
        # 验证高层关节映射不依赖底层列表顺序。
        return list(reversed(self.sdk_motors))

    def motor_send_cmd(self):
        self.sent += 1

    def set_reset_zero(self):
        self.zero_calls += 1


spec = importlib.util.spec_from_file_location(
    'panthera_optional_gripper', SCRIPTS / 'Panthera_lib/Panthera.py'
)
module = importlib.util.module_from_spec(spec)
original_sdk = sys.modules.get('hightorque_robot')
sys.modules['hightorque_robot'] = types.SimpleNamespace(Robot=FakeRobot)
try:
    spec.loader.exec_module(module)
finally:
    if original_sdk is None:
        sys.modules.pop('hightorque_robot', None)
    else:
        sys.modules['hightorque_robot'] = original_sdk
Panthera = module.Panthera


class OptionalGripperTests(unittest.TestCase):
    def setUp(self):
        FakeRobot.offline = set()
        self.output = contextlib.redirect_stdout(io.StringIO())
        self.output.__enter__()

    def tearDown(self):
        self.output.__exit__(None, None, None)

    @staticmethod
    def create_robot(gripper_installed, config_name='Follower.yaml'):
        config_dir = SCRIPTS.parent / 'robot_param'
        config = yaml.safe_load((config_dir / config_name).read_text())
        config['robot']['gripper_installed'] = gripper_installed
        config['robot']['param_file'] = str(
            config_dir / config['robot']['param_file']
        )
        config['urdf']['file_path'] = str(
            config_dir / config['urdf']['file_path']
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / config_name
            path.write_text(yaml.safe_dump(config))
            return Panthera(str(path))

    def test_six_motor_sdk_config_and_zero(self):
        robot = self.create_robot(False)
        self.assertEqual(len(robot.sdk_motors), 6)
        self.assertEqual(robot.motor_count, 6)
        self.assertEqual([motor.id for motor in robot.Motors], list(range(1, 7)))
        self.assertFalse(robot.has_gripper)
        self.assertIsNone(robot.gripper_id)
        robot.set_reset_zero()
        self.assertEqual(robot.zero_calls, 1)

    def test_absent_gripper_never_commands_joint_six(self):
        robot = self.create_robot(False)
        self.assertFalse(robot.gripper_control(1, 0.5))
        self.assertFalse(robot.gripper_control_MIT(1, 0, 0, 1, 0))
        self.assertEqual(robot.sent, 0)
        self.assertFalse(robot.Motors[5].commands)
        self.assertFalse(robot.get_current_state_gripper().available)
        self.assertEqual(robot.get_current_pos_gripper(), 0)
        robot.Joint_Pos_Vel([0] * 6, [0.1] * 6)
        self.assertEqual(robot.sent, 1)
        self.assertTrue(all(motor.commands for motor in robot.Motors))

    def test_seven_motor_behavior(self):
        robot = self.create_robot(True)
        self.assertTrue(robot.has_gripper)
        self.assertEqual(robot.motor_count, 6)
        self.assertEqual(robot.total_motor_count, 7)
        self.assertEqual(robot.gripper_id, 7)
        self.assertEqual(robot.get_current_pos_gripper(), 0.7)
        self.assertTrue(robot.gripper_control(1, 0.5))
        self.assertFalse(robot.Motors[5].commands)
        self.assertTrue(robot.gripper_motor.commands)
        robot.set_reset_zero()
        self.assertEqual(robot.zero_calls, 1)

    def test_detected_missing_gripper_blocks_native_zero(self):
        FakeRobot.offline = {GRIPPER_NAME}
        robot = self.create_robot(True)
        self.assertFalse(robot.has_gripper)
        self.assertEqual(len(robot.Motors), 7)
        self.assertEqual(robot.gripper_id, 7)
        self.assertFalse(robot.gripper_control(1, 0.5))
        with self.assertRaisesRegex(RuntimeError, 'gripper_installed: false'):
            robot.set_reset_zero()
        self.assertEqual(robot.zero_calls, 0)

    def test_missing_arm_joint_keeps_diagnostic_access(self):
        FakeRobot.offline = {'joint3'}
        robot = self.create_robot(False)
        self.assertEqual(robot.motor_count, 6)
        with self.assertRaisesRegex(RuntimeError, 'joint3'):
            robot.set_reset_zero()

    def test_leader_config(self):
        config = SCRIPTS.parent / 'robot_param/Leader.yaml'
        robot = self.create_robot(False, 'Leader.yaml')
        self.assertEqual(len(robot.sdk_motors), 6)
        self.assertEqual(len(robot.get_current_pos()), 6)

    def test_yaml_switch_recalculates_sdk_motor_count(self):
        config_dir = SCRIPTS.parent / 'robot_param'
        config = yaml.safe_load((config_dir / 'Follower.yaml').read_text())
        params_path = config_dir / config['robot']['param_file']
        params = yaml.safe_load(params_path.read_text())
        config['urdf']['file_path'] = str(config_dir / config['urdf']['file_path'])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            port = params['robot']['CANboard']['No_1_CANboard']['CANport'][
                'CANport_1'
            ]
            # 故意写错原始数量，确认两种开关值都会重新计算底层数量。
            port['motor_num'] = 99
            (path / 'motors.yaml').write_text(yaml.safe_dump(params))
            config['robot']['param_file'] = 'motors.yaml'
            for enabled in (False, True):
                with self.subTest(gripper_installed=enabled):
                    config['robot']['gripper_installed'] = enabled
                    robot_path = path / 'robot.yaml'
                    robot_path.write_text(yaml.safe_dump(config))
                    robot = Panthera(str(robot_path))
                    self.assertEqual(robot.has_gripper, enabled)
                    self.assertEqual(robot.total_motor_count, 6 + int(enabled))
                    self.assertEqual(len(robot.sdk_motors), robot.total_motor_count)
                    self.assertEqual(robot.motor_count, 6)
                    self.assertEqual(len(robot.get_current_pos()), 6)

    def test_no_gripper_model_removes_mass_and_uses_link6(self):
        # Leader URDF 保留完整夹爪，可验证裁剪前后的质量差异；Follower
        # URDF 允许用户预先手动删除夹爪，不能假定两种模型质量不同。
        without_gripper = self.create_robot(False, 'Leader.yaml')
        with_gripper = self.create_robot(True, 'Leader.yaml')

        mass_without = sum(inertia.mass for inertia in without_gripper.model.inertias)
        mass_with = sum(inertia.mass for inertia in with_gripper.model.inertias)
        self.assertLess(mass_without, mass_with)
        self.assertFalse(without_gripper.model.existFrame('gripper_center_link'))
        end_frame = without_gripper.model.frames[
            without_gripper.end_effector_frame_id
        ]
        self.assertEqual(end_frame.name, 'link6')

        q = np.array([0.2, 0.7, 0.8, -0.3, 0.4, -0.2])
        gravity_without = without_gripper.get_Gravity(q)
        gravity_with = with_gripper.get_Gravity(q)
        self.assertFalse(np.allclose(gravity_without, gravity_with))

    def test_no_gripper_urdf_removes_end_effector_and_preserves_siblings(self):
        source = '''<robot name="test">
          <link name="link6"/>
          <link name="gripper_adapter"/>
          <link name="gripper_center_link"/>
          <link name="camera_link"/>
          <joint name="adapter" type="fixed">
            <parent link="link6"/><child link="gripper_adapter"/>
          </joint>
          <joint name="tool" type="fixed">
            <parent link="gripper_adapter"/><child link="gripper_center_link"/>
          </joint>
          <joint name="camera" type="fixed">
            <parent link="link6"/><child link="camera_link"/>
          </joint>
        </robot>'''
        robot = Panthera.__new__(Panthera)
        robot.config = {'urdf': {
            'end_effector_link': 'gripper_center_link',
            'end_effector_link_without_gripper': 'link6',
        }}
        with tempfile.TemporaryDirectory() as directory:
            source_path = Path(directory) / 'source.urdf'
            source_path.write_text(source)
            output = robot._create_urdf_without_gripper(
                str(source_path), directory
            )
            root = ET.parse(output).getroot()

        remaining = {link.get('name') for link in root.findall('link')}
        self.assertEqual(remaining, {
            'link6', 'gripper_adapter', 'camera_link'
        })

    def test_no_gripper_urdf_accepts_manually_removed_end_effector(self):
        source = '''<robot name="test">
          <link name="link6"/>
          <link name="gripper_L_link"/>
          <link name="camera_link"/>
          <joint name="stale_center" type="fixed">
            <parent link="link6"/><child link="gripper_center_link"/>
          </joint>
          <joint name="stale_finger" type="fixed">
            <parent link="gripper_center_link"/><child link="gripper_L_link"/>
          </joint>
          <joint name="camera" type="fixed">
            <parent link="link6"/><child link="camera_link"/>
          </joint>
        </robot>'''
        robot = Panthera.__new__(Panthera)
        robot.config = {'urdf': {
            'end_effector_link': 'gripper_center_link',
            'end_effector_link_without_gripper': 'link6',
        }}
        with tempfile.TemporaryDirectory() as directory:
            source_path = Path(directory) / 'source.urdf'
            source_path.write_text(source)
            output = robot._create_urdf_without_gripper(
                str(source_path), directory
            )
            root = ET.parse(output).getroot()

        remaining_links = {
            link.get('name') for link in root.findall('link')
        }
        remaining_joints = {
            joint.get('name') for joint in root.findall('joint')
        }
        self.assertEqual(remaining_links, {'link6', 'camera_link'})
        self.assertEqual(remaining_joints, {'camera'})


if __name__ == '__main__':
    unittest.main()
