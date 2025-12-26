"""
高扭矩机器人电机控制Python接口

这个包提供了对高扭矩机器人电机控制C++库的Python绑定。

主要类:
    Robot: 机器人控制器,管理多个电机
    Motor: 单个电机控制
    MotorState: 电机状态数据结构
    MotorType: 电机类型枚举

示例:
    >>> import hightorque_robot as htr
    >>> robot = htr.Robot("/path/to/config.yaml")
    >>> motors = robot.get_motors()
    >>> motors[0].position(1.0)
    >>> robot.motor_send_cmd()
"""

from ._hightorque_robot import (
    # 主要类
    Robot,
    Motor,
    MotorState,
    MotorVersion,

    # 枚举类型
    MotorType,
    PosVelConvertType,

    # 配置参数类
    RobotParams,
    CANBoardParams,
    CANPortParams,
    MotorParams,

    # 辅助函数
    parse_robot_params,

    # 版本信息
    __version__,
    __cpp_sdk_version__,
)

from .utils import (
    setup_motor_control,
    print_motor_states,
    rad_to_deg,
    deg_to_rad,
    create_sinusoidal_trajectory,
)

__all__ = [
    # 主要类
    'Robot',
    'Motor',
    'MotorState',
    'MotorVersion',

    # 枚举
    'MotorType',
    'PosVelConvertType',

    # 配置
    'RobotParams',
    'CANBoardParams',
    'CANPortParams',
    'MotorParams',

    # 函数
    'parse_robot_params',
    'setup_motor_control',
    'print_motor_states',
    'rad_to_deg',
    'deg_to_rad',
    'create_sinusoidal_trajectory',

    # 版本
    '__version__',
    '__cpp_sdk_version__',
]

# 便捷访问
__author__ = "Tang"
__email__ = "user@example.com"
