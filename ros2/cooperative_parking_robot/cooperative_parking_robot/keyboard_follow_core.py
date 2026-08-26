"""Pure control and safety logic for bounded keyboard pair following.

The keyboard command is interpreted as the virtual pair-centre twist.  Front
and Rear receive rigid-body feed-forward velocities, then small opposite
ArUco corrections keep the relative gap, lateral offset, and yaw captured at
arm time.  Keeping this module ROS-free makes every sign and stop gate easy to
regression-test before touching real motors.
"""

from dataclasses import dataclass
import math

from cooperative_parking_robot.rigid_body_kinematics import (
    RigidBodyKinematics,
)


ZERO_COMMAND = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class KeyboardFollowLimits:
    linear_limit_mps: float = 0.08
    angular_limit_rps: float = 0.20
    gap_kp: float = 1.0
    lateral_kp: float = 1.0
    yaw_kp: float = 1.0
    linear_correction_limit_mps: float = 0.015
    yaw_correction_limit_rps: float = 0.08
    gap_stop_m: float = 0.03
    lateral_stop_m: float = 0.03
    yaw_stop_rad: float = math.radians(5.0)
    max_session_distance_m: float = 0.30

    def validate(self):
        values = tuple(float(value) for value in self.__dict__.values())
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError('keyboard-follow limits must be finite and positive')
        if self.linear_correction_limit_mps >= self.linear_limit_mps:
            raise ValueError('linear correction must be below the command limit')
        if self.yaw_correction_limit_rps >= self.angular_limit_rps:
            raise ValueError('yaw correction must be below the command limit')
        if self.max_session_distance_m > 1.0:
            raise ValueError('keyboard-follow session distance must not exceed 1m')


@dataclass(frozen=True)
class FollowDecision:
    """One fail-closed decision from the keyboard-follow safety evaluator."""

    outcome: str
    reason: str = ''


def angle_norm(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def clamp(value, limit):
    """Clamp a scalar symmetrically around zero."""
    limit = abs(float(limit))
    return max(-limit, min(limit, float(value)))


def is_zero(command, tolerance=1e-9):
    """Return whether every twist component is effectively zero."""
    return all(abs(float(value)) <= tolerance for value in command)


def capture_aruco_reference(relative_pose):
    """Capture the raw current ArUco pose without adding an offset.

    The user-defined axle spacing is exactly the currently observed forward
    value.  This helper deliberately does not add robot length, camera offset,
    or a nominal vehicle wheelbase.
    """
    values = tuple(float(value) for value in relative_pose)
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError('ArUco reference must contain three finite values')
    if values[0] <= 0.0:
        raise ValueError('ArUco forward reference must be positive')
    return values


def follow_pair_commands(limits, intent, separation_m, *, gap_error_m,
                         lateral_error_m, yaw_error_rad):
    """Convert a virtual-pair keyboard intent into Front/Rear commands.

    Positive relative errors mean Front is farther ahead, farther left, or
    rotated more counter-clockwise than at arm time.  Opposite half
    corrections slow Front and speed Rear along the corresponding axis.
    Rotation feed-forward comes from rigid-body kinematics, so Q/E rotates the
    pair around its midpoint instead of making both robots spin in place.
    """
    limits.validate()
    values = tuple(float(value) for value in intent)
    numeric = values + (
        float(separation_m), float(gap_error_m), float(lateral_error_m),
        float(yaw_error_rad))
    if len(values) != 3 or not all(math.isfinite(value) for value in numeric):
        raise ValueError('keyboard-follow command values must be finite')
    if separation_m <= 0.0:
        raise ValueError('pair separation must be positive')

    kinematics = RigidBodyKinematics(separation_m)
    front, rear = kinematics.split(*values)
    gap_correction = clamp(
        limits.gap_kp * gap_error_m,
        limits.linear_correction_limit_mps)
    lateral_correction = clamp(
        limits.lateral_kp * lateral_error_m,
        limits.linear_correction_limit_mps)
    yaw_correction = clamp(
        limits.yaw_kp * yaw_error_rad,
        limits.yaw_correction_limit_rps)

    front = (
        front[0] - 0.5 * gap_correction,
        front[1] - 0.5 * lateral_correction,
        front[2] - 0.5 * yaw_correction,
    )
    rear = (
        rear[0] + 0.5 * gap_correction,
        rear[1] + 0.5 * lateral_correction,
        rear[2] + 0.5 * yaw_correction,
    )
    return RigidBodyKinematics.limit_twist_pair(
        front, rear,
        linear_limit=limits.linear_limit_mps,
        angular_limit=limits.angular_limit_rps)


def evaluate_follow(limits, *, gap_error_m, lateral_error_m, yaw_error_rad,
                    front_distance_m, rear_distance_m, hardware_ok,
                    manual_ok, marker_ok, odom_ok, graph_ok, estop):
    """Return CONTINUE or a fail-closed decision for one control cycle."""
    limits.validate()
    numeric = (
        gap_error_m, lateral_error_m, yaw_error_rad,
        front_distance_m, rear_distance_m)
    if not all(math.isfinite(float(value)) for value in numeric):
        return FollowDecision('FAULT', '비정상 수치(NaN/Inf) 수신')
    if estop:
        return FollowDecision('ESTOP', '비상정지 신호 감지')
    if not graph_ok:
        return FollowDecision('FAULT', '다른 주행 명령 발행자와 충돌')
    if not hardware_ok:
        return FollowDecision('FAULT', '하드웨어 준비 신호 끊김')
    if not manual_ok:
        return FollowDecision('FAULT', '양쪽 수동 제어권 확인 실패')
    if not marker_ok:
        return FollowDecision('FAULT', 'ArUco 관측이 끊기거나 오래됨')
    if not odom_ok:
        return FollowDecision('FAULT', '엔코더 odometry가 끊기거나 오래됨')
    if abs(float(gap_error_m)) > limits.gap_stop_m:
        return FollowDecision(
            'FAULT', f'로봇 간격 변화 {gap_error_m * 100.0:+.1f} cm')
    if abs(float(lateral_error_m)) > limits.lateral_stop_m:
        return FollowDecision(
            'FAULT', f'좌우 어긋남 {lateral_error_m * 100.0:+.1f} cm')
    if abs(float(yaw_error_rad)) > limits.yaw_stop_rad:
        return FollowDecision(
            'FAULT',
            f'상대 각도 변화 {math.degrees(yaw_error_rad):+.1f} deg')
    if max(front_distance_m, rear_distance_m) >= \
            limits.max_session_distance_m:
        return FollowDecision('LIMIT', '세션 누적 이동거리 제한 도달')
    return FollowDecision('CONTINUE')
