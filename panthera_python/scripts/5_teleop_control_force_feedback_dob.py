#!/usr/bin/env python3
"""六轴广义动量 DOB + joint7 夹爪直接力矩反馈。"""

import argparse
import math
import os
import sys
import time

import numpy as np

from Panthera_lib import Panthera


FOLLOWER_KP = np.array([10.0, 21.0, 21.0, 16.0, 13.0, 1.0])
FOLLOWER_KD = np.array([1.0, 2.0, 2.0, 0.9, 0.8, 0.1])

COULOMB_FRICTION = np.array([0.15, 0.12, 0.12, 0.12, 0.04, 0.04])
VISCOUS_FRICTION = np.array([0.05, 0.05, 0.05, 0.03, 0.02, 0.02])
FRICTION_SMOOTHING_VELOCITY = 0.05

TOTAL_TORQUE_LIMIT = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0])
FEEDBACK_TORQUE_LIMIT = np.array([2.5, 5.0, 5.0, 2.5, 1.2, 1.2])
FEEDBACK_TORQUE_RATE_LIMIT = np.array([16.0, 30.0, 30.0, 16.0, 8.0, 8.0])
DOB_RESET_LIMIT = np.array([16.0, 30.0, 30.0, 16.0, 8.0, 8.0])
MASTER_DAMPING = np.array([0.12, 0.20, 0.20, 0.08, 0.04, 0.04])
JOINT_LIMIT_MARGIN = 0.002

GRIPPER_KP = 4.0
GRIPPER_KD = 0.4
GRIPPER_FRICTION = 0.06
GRIPPER_FRICTION_SMOOTHING_VELOCITY = 0.10
# joint7 夹爪采用从臂实测力矩直接反向映射，不使用 DOB。
# 仍保留输出限幅和斜率限制，避免 CAN/电机瞬时异常直接施加到主臂。
GRIPPER_DIRECT_FEEDBACK_LIMIT = 3.0
GRIPPER_TOTAL_TORQUE_LIMIT = 3.0
GRIPPER_DIRECT_FEEDBACK_RATE_LIMIT = 30.0
GRIPPER_FEEDBACK_SIGN = -1.0


def rate_limit(target, previous, max_rate, dt):
    target = np.asarray(target, dtype=float)
    previous = np.asarray(previous, dtype=float)
    max_step = np.asarray(max_rate, dtype=float) * dt
    return previous + np.clip(target - previous, -max_step, max_step)


class FirstOrderLowPass:
    def __init__(self, cutoff_hz):
        self.cutoff_hz = cutoff_hz
        self.state = None

    def update(self, sample, dt):
        sample = np.asarray(sample, dtype=float)
        if self.state is None:
            self.state = sample.copy()
            return self.state.copy()
        alpha = 1.0 - math.exp(-2.0 * math.pi * self.cutoff_hz * dt)
        self.state += alpha * (sample - self.state)
        return self.state.copy()

    def reset(self):
        self.state = None


class MomentumDisturbanceObserver:
    """广义动量 DOB，输出环境施加到从臂的关节外力矩。"""

    def __init__(self, cutoff_hz):
        self.gain = 2.0 * math.pi * cutoff_hz
        self.integral = None
        self.residual = None

    def reset(self):
        self.integral = None
        self.residual = None

    def update(self, momentum, nominal_momentum_rate, dt):
        momentum = np.asarray(momentum, dtype=float)
        nominal_momentum_rate = np.asarray(nominal_momentum_rate, dtype=float)

        if self.integral is None:
            self.integral = momentum.copy()
            self.residual = np.zeros_like(momentum)
            return self.residual.copy()

        self.integral += dt * (nominal_momentum_rate + self.residual)
        self.residual = self.gain * (momentum - self.integral)
        return self.residual.copy()


class RunningBias:
    def __init__(self, size):
        self.value = np.zeros(size, dtype=float)
        self.samples = 0

    def reset(self):
        self.value.fill(0.0)
        self.samples = 0

    def update(self, sample):
        self.samples += 1
        self.value += (np.asarray(sample, dtype=float) - self.value) / self.samples


class DirectDOBTeleop:
    def __init__(self, leader, follower, args):
        if leader.motor_count != follower.motor_count:
            raise ValueError("主臂和从臂的关节数不一致")
        if leader.motor_count != len(FOLLOWER_KP):
            raise ValueError("该脚本当前仅配置为 6 自由度 Panthera")

        self.leader = leader
        self.follower = follower
        self.feedback_gain = args.feedback_gain
        self.observer_warmup_time = args.observer_warmup_time
        self.sync_time = args.sync_time
        self.bias_time = args.bias_time
        self.feedback_ramp_time = args.feedback_ramp_time
        self.period = 1.0 / args.rate_hz
        self.print_period = 1.0 / args.print_hz
        self.enable_gripper = args.enable_gripper
        self.gripper_feedback_gain = args.gripper_feedback_gain

        self.observer = MomentumDisturbanceObserver(args.dob_cutoff_hz)
        self.dob_cutoff_hz = args.dob_cutoff_hz
        self.velocity_filter = FirstOrderLowPass(args.velocity_cutoff_hz)
        self.velocity_cutoff_hz = args.velocity_cutoff_hz
        self.disturbance_bias = RunningBias(leader.motor_count)
        self.previous_feedback = np.zeros(leader.motor_count)
        self.zero = np.zeros(leader.motor_count)
        self.post_sync_initialized = False
        self.observer_reset_count = 0
        self.observer_reset_this_cycle = False

        self.previous_gripper_feedback = 0.0

    @staticmethod
    def _check_finite(**signals):
        for name, value in signals.items():
            if not np.all(np.isfinite(value)):
                raise RuntimeError(f"{name} 包含 NaN 或 Inf，停止控制")

    @staticmethod
    def _friction(velocity):
        velocity = np.asarray(velocity, dtype=float)
        return (
            COULOMB_FRICTION
            * np.tanh(velocity / FRICTION_SMOOTHING_VELOCITY)
            + VISCOUS_FRICTION * velocity
        )

    @staticmethod
    def _gripper_friction(velocity):
        return float(
            GRIPPER_FRICTION
            * np.tanh(velocity / GRIPPER_FRICTION_SMOOTHING_VELOCITY)
        )

    def _phase(self, elapsed):
        warmup_end = self.observer_warmup_time
        sync_end = warmup_end + self.sync_time
        bias_end = sync_end + self.bias_time

        if elapsed < warmup_end:
            return "观测器预热", 0.0, 0.0
        if elapsed < sync_end:
            sync = (elapsed - warmup_end) / self.sync_time
            return "从臂同步", sync, 0.0
        if elapsed < bias_end:
            return "静态校准", 1.0, 0.0

        feedback_activation = min(
            (elapsed - bias_end) / self.feedback_ramp_time, 1.0
        )
        return "力反馈", 1.0, feedback_activation

    def _estimate_disturbance(
        self,
        position,
        velocity,
        measured_torque,
        gravity,
        friction,
        dt,
    ):
        mass = np.array(
            self.follower.get_Mass_Matrix(position), dtype=float, copy=True
        )
        coriolis = np.array(
            self.follower.get_Coriolis(position, velocity), dtype=float, copy=True
        )
        momentum = mass @ velocity

        # M q_ddot + C q_dot + G + tau_f = tau_motor + tau_external
        # p_dot = tau_motor - (G + tau_f - C^T q_dot) + tau_external
        beta = gravity + friction - coriolis.T @ velocity
        nominal_momentum_rate = measured_torque - beta
        self._check_finite(
            mass=mass,
            coriolis=coriolis,
            momentum=momentum,
            nominal_momentum_rate=nominal_momentum_rate,
        )

        disturbance = self.observer.update(
            momentum, nominal_momentum_rate, dt
        )
        self.observer_reset_this_cycle = False
        if np.any(np.abs(disturbance) > DOB_RESET_LIMIT):
            self.observer.reset()
            self.previous_feedback.fill(0.0)
            self.observer_reset_count += 1
            self.observer_reset_this_cycle = True
            return np.zeros_like(disturbance)
        return disturbance

    def _limit_follower_target(self, position, velocity):
        """将遥操目标夹紧到从臂限位内侧，并阻止继续向限位外运动。"""
        position = np.asarray(position, dtype=float)
        velocity = np.asarray(velocity, dtype=float).copy()
        if self.follower.joint_limits is None:
            return position.copy(), velocity, np.zeros_like(position, dtype=bool)

        lower = np.asarray(
            self.follower.joint_limits["lower"], dtype=float
        )
        upper = np.asarray(
            self.follower.joint_limits["upper"], dtype=float
        )
        safe_lower = lower + JOINT_LIMIT_MARGIN
        safe_upper = upper - JOINT_LIMIT_MARGIN
        limited_position = np.clip(position, safe_lower, safe_upper)
        limited = np.not_equal(limited_position, position)

        outward_velocity = np.logical_or(
            (position <= safe_lower) & (velocity < 0.0),
            (position >= safe_upper) & (velocity > 0.0),
        )
        velocity[outward_velocity] = 0.0
        limited |= outward_velocity
        return limited_position, velocity, limited

    def run(self):
        start_time = time.perf_counter()
        last_time = start_time
        next_tick = start_time
        next_print = start_time + self.print_period
        report_start = start_time
        report_cycles = 0
        follower_start_pos = np.asarray(
            self.follower.get_current_pos(), dtype=float
        )
        follower_start_gripper = (
            float(self.follower.get_current_pos_gripper())
            if self.enable_gripper
            else 0.0
        )

        print("=" * 78)
        print("六轴 DOB + joint7 直接力矩反馈已启动")
        print(f"DOB/速度滤波: {self.dob_cutoff_hz:.1f}/{self.velocity_cutoff_hz:.1f} Hz")
        print(f"反馈增益: {self.feedback_gain:.2f}")
        print("反馈链路: DOB外力矩 -> 增益 -> 安全限幅/斜率 -> 主臂")
        print("注意: 不使用运动门控；主臂静止时从臂外力也会传到主臂。")
        print(
            "启动期间请保持双臂无接触；屏幕进入“力反馈 100%”后再操作。"
        )
        if not self.enable_gripper:
            print("夹爪控制已关闭；需要时使用 --enable-gripper。")
        else:
            print(f"夹爪直接力矩反馈增益: {self.gripper_feedback_gain:.2f}")
            print("夹爪反馈链路: -从臂joint7实测力矩 -> 主臂joint7")
            print("夹爪不使用 DOB、惯量模型、滤波或运动门控。")
        print("按 Ctrl+C 停止。")
        print("=" * 78)

        while True:
            now = time.perf_counter()
            elapsed = now - start_time
            dt = float(np.clip(now - last_time, 0.0002, 0.02))
            last_time = now
            phase, sync_activation, feedback_activation = self._phase(elapsed)

            leader_pos = np.asarray(self.leader.get_current_pos(), dtype=float)
            leader_vel = np.asarray(self.leader.get_current_vel(), dtype=float)
            follower_pos = np.asarray(self.follower.get_current_pos(), dtype=float)
            follower_vel = np.asarray(self.follower.get_current_vel(), dtype=float)
            follower_torque = np.asarray(
                self.follower.get_current_torque(), dtype=float
            )
            leader_gravity = np.asarray(
                self.leader.get_Gravity(leader_pos), dtype=float
            )
            follower_gravity = np.asarray(
                self.follower.get_Gravity(follower_pos), dtype=float
            )
            leader_friction = self._friction(leader_vel)
            follower_friction = self._friction(follower_vel)
            observer_velocity = self.velocity_filter.update(follower_vel, dt)
            observer_friction = self._friction(observer_velocity)
            self._check_finite(
                leader_pos=leader_pos,
                leader_vel=leader_vel,
                follower_pos=follower_pos,
                follower_vel=follower_vel,
                follower_torque=follower_torque,
                leader_gravity=leader_gravity,
                follower_gravity=follower_gravity,
            )

            disturbance_raw = self._estimate_disturbance(
                follower_pos,
                observer_velocity,
                follower_torque,
                follower_gravity,
                observer_friction,
                dt,
            )

            sync_end = self.observer_warmup_time + self.sync_time
            bias_end = sync_end + self.bias_time
            if elapsed >= sync_end and not self.post_sync_initialized:
                self.observer.reset()
                self.disturbance_bias.reset()
                self.previous_feedback.fill(0.0)
                self.previous_gripper_feedback = 0.0
                self.post_sync_initialized = True
                disturbance_raw = np.zeros_like(disturbance_raw)
            if sync_end <= elapsed < bias_end:
                self.disturbance_bias.update(disturbance_raw)

            disturbance = disturbance_raw - self.disturbance_bias.value
            if self.observer_reset_this_cycle:
                disturbance.fill(0.0)
            feedback_target = (
                feedback_activation * self.feedback_gain * disturbance
            )
            feedback_target = np.clip(
                feedback_target,
                -FEEDBACK_TORQUE_LIMIT,
                FEEDBACK_TORQUE_LIMIT,
            )
            feedback_torque = rate_limit(
                feedback_target,
                self.previous_feedback,
                FEEDBACK_TORQUE_RATE_LIMIT,
                dt,
            )
            if feedback_activation <= 0.0 or self.observer_reset_this_cycle:
                feedback_torque.fill(0.0)
            self.previous_feedback = feedback_torque.copy()

            leader_torque = np.clip(
                leader_gravity
                + leader_friction
                + feedback_torque
                - MASTER_DAMPING * leader_vel,
                -TOTAL_TORQUE_LIMIT,
                TOTAL_TORQUE_LIMIT,
            )
            follower_torque_cmd = np.clip(
                follower_gravity + follower_friction,
                -TOTAL_TORQUE_LIMIT,
                TOTAL_TORQUE_LIMIT,
            )
            follower_target_pos = (
                (1.0 - sync_activation) * follower_start_pos
                + sync_activation * leader_pos
            )
            follower_target_vel = sync_activation * leader_vel
            (
                follower_target_pos,
                follower_target_vel,
                limited_joints,
            ) = self._limit_follower_target(
                follower_target_pos, follower_target_vel
            )

            if not self.leader.pos_vel_tqe_kp_kd(
                self.zero, self.zero, leader_torque, self.zero, self.zero
            ):
                raise RuntimeError("主臂控制指令被拒绝")
            if not self.follower.pos_vel_tqe_kp_kd(
                follower_target_pos,
                follower_target_vel,
                follower_torque_cmd,
                FOLLOWER_KP,
                FOLLOWER_KD,
            ):
                raise RuntimeError("从臂控制指令被拒绝")

            gripper_feedback = 0.0
            gripper_measured_torque = 0.0
            if self.enable_gripper:
                (
                    gripper_feedback,
                    gripper_measured_torque,
                ) = self._control_gripper(
                    sync_activation,
                    feedback_activation,
                    follower_start_gripper,
                    dt,
                )

            report_cycles += 1
            if now >= next_print:
                actual_rate = report_cycles / max(now - report_start, 1e-6)
                tracking_error = np.max(np.abs(leader_pos - follower_pos))
                disturbance_text = np.array2string(
                    disturbance, precision=2, suppress_small=True
                )
                feedback_text = np.array2string(
                    feedback_torque, precision=2, suppress_small=True
                )
                limited_joint_numbers = np.flatnonzero(limited_joints) + 1
                limit_text = ""
                if limited_joint_numbers.size:
                    limit_text = (
                        "  限位夹紧=J"
                        + ",J".join(map(str, limited_joint_numbers))
                    )
                gripper_text = ""
                if self.enable_gripper:
                    gripper_text = (
                        f"  夹爪测量={gripper_measured_torque:+.2f} Nm"
                        f"  夹爪直接反馈={gripper_feedback:+.2f} Nm"
                    )
                print(
                    f"{phase} {feedback_activation:4.0%}  "
                    f"频率={actual_rate:5.1f} Hz  误差={tracking_error:.3f} rad  "
                    f"DOB={disturbance_text}  实际反馈={feedback_text}  "
                    f"重置={self.observer_reset_count:d}"
                    f"{limit_text}{gripper_text}"
                )
                next_print = now + self.print_period
                report_start = now
                report_cycles = 0

            next_tick += self.period
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                next_tick = time.perf_counter()

    def _control_gripper(
        self,
        sync_activation,
        feedback_activation,
        follower_start_gripper,
        dt,
    ):
        leader_pos = float(self.leader.get_current_pos_gripper())
        leader_vel = float(self.leader.get_current_vel_gripper())
        follower_measured_torque = float(
            self.follower.get_current_torque_gripper()
        )
        self._check_finite(
            leader_gripper_pos=leader_pos,
            leader_gripper_vel=leader_vel,
            follower_gripper_torque=follower_measured_torque,
        )

        feedback_target = float(
            np.clip(
                feedback_activation
                * self.gripper_feedback_gain
                * GRIPPER_FEEDBACK_SIGN
                * follower_measured_torque,
                -GRIPPER_DIRECT_FEEDBACK_LIMIT,
                GRIPPER_DIRECT_FEEDBACK_LIMIT,
            )
        )
        feedback_torque = float(
            rate_limit(
                [feedback_target],
                [self.previous_gripper_feedback],
                [GRIPPER_DIRECT_FEEDBACK_RATE_LIMIT],
                dt,
            )[0]
        )
        if feedback_activation <= 0.0:
            feedback_torque = 0.0
        self.previous_gripper_feedback = feedback_torque

        leader_friction = self._gripper_friction(leader_vel)
        leader_command = float(
            np.clip(
                leader_friction
                + feedback_torque,
                -GRIPPER_TOTAL_TORQUE_LIMIT,
                GRIPPER_TOTAL_TORQUE_LIMIT,
            )
        )
        leader_target = float(
            np.clip(
                leader_pos,
                self.leader.gripper_limits["lower"],
                self.leader.gripper_limits["upper"],
            )
        )
        follower_target = float(
            np.clip(
                (1.0 - sync_activation) * follower_start_gripper
                + sync_activation * leader_pos,
                self.follower.gripper_limits["lower"],
                self.follower.gripper_limits["upper"],
            )
        )
        if not self.leader.gripper_control_MIT(
            leader_target, 0.0, leader_command, 0.0, 0.0
        ):
            raise RuntimeError("主臂夹爪控制指令被拒绝")
        if not self.follower.gripper_control_MIT(
            follower_target,
            sync_activation * leader_vel,
            0.0,
            GRIPPER_KP,
            GRIPPER_KD,
        ):
            raise RuntimeError("从臂夹爪控制指令被拒绝")
        return feedback_torque, follower_measured_torque


def parse_args():
    parser = argparse.ArgumentParser(
        description="Panthera 六轴 DOB + joint7 直接力矩反馈"
    )
    parser.add_argument("--feedback-gain", type=float, default=0.35)
    parser.add_argument("--dob-cutoff-hz", type=float, default=3.0)
    parser.add_argument("--velocity-cutoff-hz", type=float, default=12.0)
    parser.add_argument(
        "--gripper-feedback-gain",
        type=float,
        default=None,
        help="夹爪反馈增益；默认跟随 --feedback-gain",
    )
    parser.add_argument("--observer-warmup-time", type=float, default=1.0)
    parser.add_argument("--sync-time", type=float, default=3.0)
    parser.add_argument("--bias-time", type=float, default=1.0)
    parser.add_argument("--feedback-ramp-time", type=float, default=1.0)
    parser.add_argument("--rate-hz", type=float, default=250.0)
    parser.add_argument("--print-hz", type=float, default=2.0)
    parser.add_argument("--enable-gripper", action="store_true")
    args = parser.parse_args()

    if args.gripper_feedback_gain is None:
        args.gripper_feedback_gain = args.feedback_gain

    if not 0.0 <= args.feedback_gain <= 1.0:
        parser.error("--feedback-gain 必须在 [0, 1] 内")
    if not 0.0 <= args.gripper_feedback_gain <= 1.0:
        parser.error("--gripper-feedback-gain 必须在 [0, 1] 内")
    cutoff_frequencies = (args.dob_cutoff_hz, args.velocity_cutoff_hz)
    if min(cutoff_frequencies) <= 0.0:
        parser.error("DOB 和速度截止频率必须大于 0")
    timing = (
        args.observer_warmup_time,
        args.sync_time,
        args.bias_time,
        args.feedback_ramp_time,
        args.rate_hz,
        args.print_hz,
    )
    if min(timing) <= 0.0:
        parser.error("所有时间和频率参数必须大于 0")
    if 2.0 * math.pi * args.dob_cutoff_hz / args.rate_hz >= 1.0:
        parser.error("DOB 截止频率过高；需满足 2*pi*cutoff/rate < 1")
    return args


def stop_robot(robot, name):
    if robot is None:
        return
    try:
        robot.set_stop()
    except Exception as exc:
        print(f"警告: {name}停机指令失败: {exc}")


def main():
    args = parse_args()
    script_dir = os.path.dirname(os.path.abspath(__file__))
    leader_config = os.path.join(script_dir, "../robot_param/Leader.yaml")
    follower_config = os.path.join(script_dir, "../robot_param/Follower.yaml")
    leader = None
    follower = None
    exit_code = 0

    try:
        leader = Panthera(leader_config)
        follower = Panthera(follower_config)
        DirectDOBTeleop(leader, follower, args).run()
    except KeyboardInterrupt:
        print("\n收到停止请求。")
    except Exception as exc:
        print(f"\n控制异常: {exc}")
        exit_code = 1
    finally:
        stop_robot(follower, "从臂")
        stop_robot(leader, "主臂")
        print("主臂和从臂已发送停机指令。")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
