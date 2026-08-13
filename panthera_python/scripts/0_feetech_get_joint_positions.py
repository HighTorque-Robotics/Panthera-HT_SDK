#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""直接读取并打印 GELLO 上 7 个飞特舵机的位置。

读取方式与 GELLO 遥操作程序一致：
  - SDK：feetech-servo-sdk
  - 协议处理器：PacketHandler(0)
  - 当前位置寄存器：56（读取 2 字节）
  - 默认波特率：1 Mbps
  - 关节 ID 顺序：1, 2, 3, 4, 5, 6；夹爪 ID：7

程序不读取机械臂 YAML，只发送舵机位置读取指令。

安装依赖：
    python -m pip install feetech-servo-sdk

运行：
    python 0_feetech_get_joint_positions.py
    python 0_feetech_get_joint_positions.py --port /dev/ttyACM1
"""

import argparse
import math
import os
import sys
import time


FEETECH_BAUD = 1_000_000
PRESENT_POSITION_ADDR = 56
ARM_IDS = [1, 2, 3, 4, 5, 6]
GRIPPER_ID = 7
SERVO_IDS = ARM_IDS + [GRIPPER_ID]
TICKS_PER_REV = 4096


def detect_default_gello_port():
    """使用 GELLO 遥操作程序相同的优先级查找飞特串口。"""
    candidates = [
        "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14028931-if00",
        "/dev/ttyACM1",
        "/dev/feetech_servos",
        "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
        "/dev/ttyUSB0",
    ]
    for path in candidates:
        if os.path.exists(path):
            return path
    return "/dev/ttyACM1"


def positive_float(value):
    """供 argparse 使用的正浮点数校验函数。"""
    number = float(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("该参数必须大于 0")
    return number


def positive_int(value):
    """供 argparse 使用的正整数校验函数。"""
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("该参数必须大于 0")
    return number


def import_feetech_sdk():
    """导入与 GELLO 遥操作程序相同的飞特 SDK 接口。"""
    try:
        from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler
    except (ImportError, OSError) as error:
        raise RuntimeError(
            "缺少或装错飞特 SDK。请安装：python -m pip install "
            "feetech-servo-sdk"
        ) from error
    return COMM_SUCCESS, PacketHandler, PortHandler


class ServoController:
    """飞特舵机只读控制器，读取逻辑与 GELLO 遥操作程序保持一致。"""

    def __init__(self, servo_ids, port, baudrate, sdk):
        comm_success, packet_handler_factory, port_handler_class = sdk
        self.servo_ids = list(servo_ids)
        self.port = port_handler_class(port)
        self.packet_handler = packet_handler_factory(0)
        self.comm_success = comm_success
        self.baudrate = baudrate

    def connect(self):
        if not self.port.openPort():
            raise RuntimeError(
                "failed to open port: {}".format(self.port.getPortName())
            )
        if not self.port.setBaudRate(self.baudrate):
            self.port.closePort()
            raise RuntimeError(
                "failed to set baudrate: {}".format(self.baudrate)
            )

    def disconnect(self):
        if self.port is not None and self.port.is_open:
            self.port.closePort()

    def read_positions(self, servo_ids=None):
        """读取完整位置帧；任一舵机失败时抛出包含其 ID 的异常。"""
        ids = self.servo_ids if servo_ids is None else list(servo_ids)
        ticks = {}

        for sid in ids:
            position, result, error = self.packet_handler.read2ByteTxRx(
                self.port,
                int(sid),
                PRESENT_POSITION_ADDR,
            )
            if result != self.comm_success:
                message = self.packet_handler.getTxRxResult(result)
                raise RuntimeError(
                    "read servo {} failed: {}".format(sid, message)
                )
            if error != 0:
                message = self.packet_handler.getRxPacketError(error)
                raise RuntimeError(
                    "servo {} returned error: {}".format(sid, message)
                )
            ticks[int(sid)] = int(position)

        return ticks


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description="直接读取并打印 GELLO 上 7 个飞特舵机的位置。"
    )
    parser.add_argument(
        "--port",
        default=None,
        help="飞特串口；不指定时按 GELLO 程序的规则自动检测",
    )
    parser.add_argument(
        "--baudrate",
        type=positive_int,
        default=FEETECH_BAUD,
        help="舵机波特率（默认：1000000）",
    )
    parser.add_argument(
        "--interval",
        type=positive_float,
        default=0.5,
        help="打印周期，单位为秒（默认：0.5）",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="只读取并打印一次",
    )
    return parser


def tick_to_degrees(tick):
    return float(tick) * 360.0 / TICKS_PER_REV


def print_positions(ticks, servo_ids):
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    milliseconds = int((time.time() % 1.0) * 1000)
    print("\n[{}.{:03d}]".format(timestamp, milliseconds))
    print("名称      舵机ID  原始位置(tick)  绝对角度(deg)  绝对位置(rad)")

    for index, sid in enumerate(servo_ids):
        name = "J{}".format(index + 1) if index < 6 else "Gripper"
        tick = ticks[sid]
        degrees = tick_to_degrees(tick)
        radians = math.radians(degrees)
        print(
            "{:<10s}{:<8d}{:<16d}{:<15.3f}{:.5f}".format(
                name, sid, tick, degrees, radians
            )
        )
    sys.stdout.flush()


def run(args, sdk):
    port = args.port or detect_default_gello_port()
    controller = ServoController(SERVO_IDS, port, args.baudrate, sdk)

    try:
        controller.connect()
        print("飞特七舵机位置接收程序已启动（只读，无 YAML）")
        print("串口：{}，波特率：{}，协议：PacketHandler(0)".format(
            port, args.baudrate
        ))
        print(
            "映射：{}".format(
                ", ".join(
                    "{} -> ID {}".format(
                        "J{}".format(index + 1) if index < 6 else "Gripper",
                        sid,
                    )
                    for index, sid in enumerate(SERVO_IDS)
                )
            )
        )
        if not args.once:
            print("按 Ctrl+C 退出。")

        last_error_print_time = 0.0
        while True:
            cycle_start = time.monotonic()
            try:
                ticks = controller.read_positions()
                print_positions(ticks, SERVO_IDS)
            except Exception as error:
                now = time.time()
                if now - last_error_print_time >= 1.0:
                    print("[FEETECH] {}".format(error), flush=True)
                    last_error_print_time = now
                if args.once:
                    return 2

            if args.once:
                return 0

            remaining_time = args.interval - (time.monotonic() - cycle_start)
            if remaining_time > 0:
                time.sleep(remaining_time)
    except KeyboardInterrupt:
        print("\n程序已停止。")
        return 0
    except Exception as error:
        print("错误：{}".format(error), file=sys.stderr)
        return 1
    finally:
        controller.disconnect()


def main():
    parser = build_argument_parser()
    args = parser.parse_args()

    try:
        sdk = import_feetech_sdk()
    except RuntimeError as error:
        print("错误：{}".format(error), file=sys.stderr)
        return 1

    return run(args, sdk)


if __name__ == "__main__":
    sys.exit(main())
