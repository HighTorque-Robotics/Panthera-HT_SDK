"""Bounded differential IK for Panthera using Pinocchio and OSQP."""

from dataclasses import dataclass

import numpy as np
import osqp
from scipy.sparse import csc_matrix, eye
from scipy.spatial.transform import Rotation


@dataclass
class QPIKResult:
    joint_position: np.ndarray
    joint_velocity: np.ndarray
    position_error: float
    rotation_error: float
    status: str
    iterations: int


class PinocchioOSQPIK:
    """Solve one velocity-level IK step with bounded joint motion."""

    def __init__(
            self, robot, position_weight=100.0, rotation_weight=2.0,
            position_gain=6.0, rotation_gain=4.0, damping=1e-3,
            smoothness=2e-2, max_joint_velocity=0.8,
            max_joint_acceleration=2.0, max_linear_velocity=0.25,
            max_angular_velocity=0.8, joint_limit_margin=0.005,
            joint_limit_slowdown_distance=0.15):
        self.robot = robot
        self.count = robot.motor_count
        self.position_weight = float(position_weight)
        self.rotation_weight = float(rotation_weight)
        self.position_gain = float(position_gain)
        self.rotation_gain = float(rotation_gain)
        self.damping = float(damping)
        self.smoothness = float(smoothness)
        self.max_linear_velocity = float(max_linear_velocity)
        self.max_angular_velocity = float(max_angular_velocity)
        self.joint_limit_margin = float(joint_limit_margin)
        self.joint_limit_slowdown_distance = float(
            joint_limit_slowdown_distance
        )
        self.previous_velocity = np.zeros(self.count, dtype=float)

        velocity_limit = np.full(
            self.count, float(max_joint_velocity), dtype=float
        )
        if getattr(robot, 'velocity_limits', None) is not None:
            velocity_limit = np.minimum(
                velocity_limit,
                np.asarray(robot.velocity_limits, dtype=float),
            )
        self.velocity_limit = velocity_limit

        acceleration_limit = np.full(
            self.count, float(max_joint_acceleration), dtype=float
        )
        if getattr(robot, 'acceleration_limits', None) is not None:
            acceleration_limit = np.minimum(
                acceleration_limit,
                np.asarray(robot.acceleration_limits, dtype=float),
            )
        self.acceleration_limit = acceleration_limit

        self.lower = None
        self.upper = None
        if robot.joint_limits is not None:
            self.lower = np.asarray(
                robot.joint_limits['lower'], dtype=float
            )
            self.upper = np.asarray(
                robot.joint_limits['upper'], dtype=float
            )

    def reset(self, velocity=None):
        if velocity is None:
            self.previous_velocity.fill(0.0)
        else:
            velocity = np.asarray(velocity, dtype=float)
            if velocity.shape != (self.count,):
                raise ValueError("QP IK velocity has an invalid shape")
            self.previous_velocity = velocity.copy()

    @staticmethod
    def _clip_norm(vector, limit):
        vector = np.asarray(vector, dtype=float)
        norm = float(np.linalg.norm(vector))
        if norm > limit:
            return vector * (limit / norm)
        return vector

    def _velocity_bounds(self, joint, dt):
        lower = -self.velocity_limit.copy()
        upper = self.velocity_limit.copy()

        acceleration_step = self.acceleration_limit * dt
        lower = np.maximum(
            lower, self.previous_velocity - acceleration_step
        )
        upper = np.minimum(
            upper, self.previous_velocity + acceleration_step
        )

        if self.lower is not None:
            safe_lower = self.lower + self.joint_limit_margin
            safe_upper = self.upper - self.joint_limit_margin
            position_lower = (
                safe_lower - joint
            ) / dt
            position_upper = (
                safe_upper - joint
            ) / dt
            constrained_lower = np.maximum(lower, position_lower)
            constrained_upper = np.minimum(upper, position_upper)

            slowdown = self.joint_limit_slowdown_distance
            if slowdown > 0.0:
                lower_scale = np.clip(
                    (joint - safe_lower) / slowdown, 0.0, 1.0
                )
                upper_scale = np.clip(
                    (safe_upper - joint) / slowdown, 0.0, 1.0
                )
                constrained_lower = np.maximum(
                    constrained_lower, -self.velocity_limit * lower_scale
                )
                constrained_upper = np.minimum(
                    constrained_upper, self.velocity_limit * upper_scale
                )

            conflict = constrained_lower > constrained_upper
            if np.any(conflict):
                for index in np.flatnonzero(conflict):
                    distance_lower = joint[index] - safe_lower[index]
                    distance_upper = safe_upper[index] - joint[index]
                    if distance_upper <= distance_lower:
                        constrained_lower[index] = constrained_upper[index]
                    else:
                        constrained_upper[index] = constrained_lower[index]
            lower, upper = constrained_lower, constrained_upper

        if np.any(lower > upper):
            raise RuntimeError("QP IK joint constraints are infeasible")
        return lower, upper

    def step(self, joint, target_transform, dt, feedforward_twist=None):
        joint = np.asarray(joint, dtype=float)
        target = np.asarray(target_transform, dtype=float)
        if joint.shape != (self.count,) or target.shape != (4, 4):
            raise ValueError("invalid QP IK joint or target shape")
        if dt <= 0.0:
            raise ValueError("QP IK dt must be positive")
        if feedforward_twist is None:
            feedforward_twist = np.zeros(6, dtype=float)
        else:
            feedforward_twist = np.asarray(
                feedforward_twist, dtype=float
            )
            if feedforward_twist.shape != (6,):
                raise ValueError("QP IK feedforward twist must have 6 values")

        fk = self.robot.forward_kinematics(joint)
        jacobian = self.robot.get_jacobian(joint)
        if fk is None or jacobian is None:
            raise RuntimeError("failed to evaluate Panthera kinematics for QP IK")
        current = np.asarray(fk['transform'], dtype=float)
        jacobian = np.asarray(jacobian, dtype=float)

        position_error_vector = target[:3, 3] - current[:3, 3]
        rotation_error_vector = Rotation.from_matrix(
            target[:3, :3] @ current[:3, :3].T
        ).as_rotvec()
        desired_linear = self._clip_norm(
            self.position_gain * position_error_vector
            + feedforward_twist[:3],
            self.max_linear_velocity,
        )
        desired_angular = self._clip_norm(
            self.rotation_gain * rotation_error_vector
            + feedforward_twist[3:],
            self.max_angular_velocity,
        )
        desired_twist = np.concatenate([desired_linear, desired_angular])

        weights = np.diag(
            [self.position_weight] * 3 + [self.rotation_weight] * 3
        )
        regularization = self.damping + self.smoothness
        hessian = jacobian.T @ weights @ jacobian
        hessian += regularization * np.eye(self.count)
        gradient = -jacobian.T @ weights @ desired_twist
        gradient -= self.smoothness * self.previous_velocity

        lower, upper = self._velocity_bounds(joint, dt)
        solver = osqp.OSQP()
        solver.setup(
            P=csc_matrix((hessian + hessian.T) * 0.5),
            q=gradient,
            A=eye(self.count, format='csc'),
            l=lower,
            u=upper,
            verbose=False,
            polishing=False,
            warm_starting=True,
            max_iter=400,
            eps_abs=1e-4,
            eps_rel=1e-4,
            check_termination=25,
        )
        solver.warm_start(x=self.previous_velocity)
        solution = solver.solve(raise_error=False)
        if solution.x is None or solution.info.status_val not in (1, 2):
            raise RuntimeError(f"OSQP IK failed: {solution.info.status}")

        # OSQP允许数值容差；下发前再次裁剪，保证实体机命令严格满足边界。
        velocity = np.clip(
            np.asarray(solution.x, dtype=float), lower, upper
        )
        next_joint = joint + velocity * dt
        if self.lower is not None:
            next_joint = np.clip(
                next_joint,
                self.lower + self.joint_limit_margin,
                self.upper - self.joint_limit_margin,
            )
        self.previous_velocity = velocity.copy()

        achieved = np.asarray(
            self.robot.forward_kinematics(next_joint)['transform'], dtype=float
        )
        position_error = float(np.linalg.norm(
            target[:3, 3] - achieved[:3, 3]
        ))
        rotation_error = float(Rotation.from_matrix(
            target[:3, :3] @ achieved[:3, :3].T
        ).magnitude())
        return QPIKResult(
            joint_position=next_joint,
            joint_velocity=velocity,
            position_error=position_error,
            rotation_error=rotation_error,
            status=solution.info.status,
            iterations=int(solution.info.iter),
        )
