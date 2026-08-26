"""Pure safety and command logic for the bounded two-robot drive test.

This module intentionally has no ROS or Flask dependency so the sign convention,
distance limit, and stop decisions can be regression-tested on a development PC.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DriveTestLimits:
    """Conservative limits for the first straight-line hardware test."""

    speed_mps: float = 0.0628
    distance_m: float = 0.10
    max_distance_m: float = 0.20
    max_duration_s: float = 4.0
    max_command_speed_mps: float = 0.08
    gap_kp: float = 1.0
    gap_correction_limit_mps: float = 0.015
    yaw_kp: float = 1.0
    yaw_correction_limit_rps: float = 0.08
    gap_error_stop_m: float = 0.03
    lateral_drift_stop_m: float = 0.03
    yaw_error_stop_rad: float = math.radians(5.0)
    odom_mismatch_stop_m: float = 0.03
    reverse_motion_stop_m: float = 0.01

    def validate(self):
        values = tuple(self.__dict__.values())
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError('drive-test limits must be finite')
        if not 0.0 < self.speed_mps <= self.max_command_speed_mps:
            raise ValueError('speed_mps must be in (0, max_command_speed_mps]')
        if not 0.0 < self.distance_m <= self.max_distance_m <= 0.20:
            raise ValueError('distance_m must be in (0, max_distance_m <= 0.20]')
        if not 0.0 < self.max_duration_s <= 10.0:
            raise ValueError('max_duration_s must be in (0, 10]')
        if min(
                self.gap_kp, self.gap_correction_limit_mps,
                self.yaw_kp, self.yaw_correction_limit_rps,
                self.gap_error_stop_m, self.lateral_drift_stop_m,
                self.yaw_error_stop_rad, self.odom_mismatch_stop_m,
                self.reverse_motion_stop_m) <= 0.0:
            raise ValueError('drive-test gains and safety limits must be positive')


@dataclass(frozen=True)
class DriveDecision:
    outcome: str
    reason: str = ''


def angle_norm(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def clamp(value, limit):
    limit = abs(float(limit))
    return max(-limit, min(limit, float(value)))


def odom_progress(start_pose, current_pose):
    """Signed displacement along the robot's heading at test start."""
    sx, sy, syaw = (float(value) for value in start_pose)
    cx, cy, _ = (float(value) for value in current_pose)
    dx, dy = cx - sx, cy - sy
    return dx * math.cos(syaw) + dy * math.sin(syaw)


def pair_commands(limits, gap_error_m, yaw_error_rad):
    """Return Front/Rear commands while preserving the pair correction signs.

    Positive gap error means Front moved too far away, so Front is slowed and
    Rear is accelerated.  Positive relative yaw receives opposite half
    corrections on the two robots.  A common scale preserves those relations
    if either linear command would exceed the test cap.
    """
    limits.validate()
    gap_correction = clamp(
        limits.gap_kp * float(gap_error_m),
        limits.gap_correction_limit_mps)
    yaw_correction = clamp(
        limits.yaw_kp * float(yaw_error_rad),
        limits.yaw_correction_limit_rps)

    front = [limits.speed_mps - 0.5 * gap_correction,
             0.0, -0.5 * yaw_correction]
    rear = [limits.speed_mps + 0.5 * gap_correction,
            0.0, 0.5 * yaw_correction]
    peak = max(abs(front[0]), abs(rear[0]))
    if peak > limits.max_command_speed_mps:
        scale = limits.max_command_speed_mps / peak
        front = [value * scale for value in front]
        rear = [value * scale for value in rear]
    return tuple(front), tuple(rear)


def evaluate_running(limits, *, elapsed_s, front_progress_m,
                     rear_progress_m, gap_error_m, lateral_drift_m,
                     yaw_error_rad, hardware_ok, manual_ok, marker_ok,
                     odom_ok, estop):
    """Make the continue/complete/stop decision for one 50 Hz cycle."""
    limits.validate()
    numeric = (
        elapsed_s, front_progress_m, rear_progress_m, gap_error_m,
        lateral_drift_m, yaw_error_rad)
    if not all(math.isfinite(float(value)) for value in numeric):
        return DriveDecision('FAULT', '비정상 수치(NaN/Inf) 수신')
    if estop:
        return DriveDecision('ESTOP', '비상정지 신호 감지')
    if not hardware_ok:
        return DriveDecision('FAULT', '하드웨어 준비 신호 끊김')
    if not manual_ok:
        return DriveDecision('FAULT', '양쪽 수동 제어권 확인 실패')
    if not marker_ok:
        return DriveDecision('FAULT', 'ArUco 관측이 끊기거나 오래됨')
    if not odom_ok:
        return DriveDecision('FAULT', '엔코더 odometry가 끊기거나 오래됨')
    if abs(float(gap_error_m)) > limits.gap_error_stop_m:
        return DriveDecision(
            'FAULT', f'로봇 간격 변화 {gap_error_m * 100.0:+.1f} cm')
    if abs(float(lateral_drift_m)) > limits.lateral_drift_stop_m:
        return DriveDecision(
            'FAULT', f'좌우 어긋남 {lateral_drift_m * 100.0:+.1f} cm')
    if abs(float(yaw_error_rad)) > limits.yaw_error_stop_rad:
        return DriveDecision(
            'FAULT',
            f'상대 각도 변화 {math.degrees(yaw_error_rad):+.1f} deg')
    if min(front_progress_m, rear_progress_m) < -limits.reverse_motion_stop_m:
        return DriveDecision('FAULT', '명령 반대 방향 이동 감지')
    mismatch = abs(float(front_progress_m) - float(rear_progress_m))
    if mismatch > limits.odom_mismatch_stop_m:
        return DriveDecision(
            'FAULT', f'양쪽 이동거리 차이 {mismatch * 100.0:.1f} cm')
    # Either robot reaching the hard target stops the pair.  This avoids one
    # robot overshooting while waiting for a stalled peer.
    if max(front_progress_m, rear_progress_m) >= limits.distance_m:
        return DriveDecision('COMPLETED', '설정한 시험 거리 도달')
    if float(elapsed_s) >= limits.max_duration_s:
        return DriveDecision('FAULT', '최대 시험시간 초과')
    return DriveDecision('CONTINUE')
