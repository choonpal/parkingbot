"""ROS-independent virtual rigid-pair teleoperation control and safety.

Every input is a pair-centre quantity or a measured relative-pose error.  No
robot is a leader: the command is split symmetrically and feedback corrections
are equal and opposite so the pair-centre intent is preserved.
"""

from dataclasses import dataclass
import math
import statistics
from urllib.parse import urlparse

from cooperative_parking_robot.rigid_body_kinematics import RigidBodyKinematics


ZERO_COMMAND = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class RigidPairTeleopLimits:
    linear_limit_mps: float = 0.08
    angular_limit_rps: float = 0.20
    gap_kp: float = 1.8
    lateral_kp: float = 1.0
    yaw_kp: float = 1.5
    heading_kp: float = 1.2
    gap_deadband_m: float = 0.004
    yaw_deadband_rad: float = math.radians(0.8)
    heading_deadband_rad: float = math.radians(0.5)
    gap_correction_limit_mps: float = 0.025
    linear_correction_limit_mps: float = 0.015
    yaw_correction_limit_rps: float = 0.08
    heading_correction_limit_rps: float = 0.04
    gap_stop_m: float = 0.03
    lateral_stop_m: float = 0.03
    yaw_stop_rad: float = math.radians(5.0)
    max_session_distance_m: float = 0.30

    def validate(self):
        values = tuple(float(value) for value in self.__dict__.values())
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError('rigid-pair teleop limits must be finite and positive')
        if self.linear_correction_limit_mps >= self.linear_limit_mps:
            raise ValueError('linear correction must be below the command limit')
        if self.gap_correction_limit_mps >= self.linear_limit_mps:
            raise ValueError('gap correction must be below the command limit')
        if (self.yaw_correction_limit_rps >= self.angular_limit_rps or
                self.heading_correction_limit_rps >= self.angular_limit_rps):
            raise ValueError(
                'yaw/heading correction must be below the command limit')
        if self.max_session_distance_m > 1.0:
            raise ValueError('rigid-pair session distance must not exceed 1m')


@dataclass(frozen=True)
class RigidPairDecision:
    """One fail-closed decision for a pair-centre control cycle."""

    outcome: str
    reason: str = ''


@dataclass(frozen=True)
class PlacementGuide:
    """Read-only ID0 placement hint; never authorizes or commands motion."""

    state: str
    raw_forward_m: float | None = None
    centre_distance_m: float | None = None
    centre_error_m: float | None = None
    raw_lateral_error_m: float | None = None
    raw_yaw_error_rad: float | None = None
    calibration_available: bool = False
    estimate_available: bool = False


class MarkerRecoveryGate:
    """Bound an ArUco dropout without discarding a newer valid pose.

    ``marker_visible`` is an unstamped Bool topic, so its delivery order cannot
    be compared safely with the stamped pose topic.  A delayed negative Bool
    therefore never invalidates a pose that is still within ``timeout_s``.
    Actual loss is decided by the valid-pose age.  Once that age expires, the
    controller holds zero during ``loss_grace_s`` and requires
    ``recovery_samples`` new poses before motion can resume.
    """

    def __init__(self, timeout_s=0.35, loss_grace_s=0.60,
                 recovery_samples=3):
        self.timeout_s = float(timeout_s)
        self.loss_grace_s = float(loss_grace_s)
        if isinstance(recovery_samples, bool):
            raise ValueError('recovery_samples must be a positive integer')
        self.recovery_samples = int(recovery_samples)
        if (not math.isfinite(self.timeout_s) or self.timeout_s <= 0.0 or
                not math.isfinite(self.loss_grace_s) or
                self.loss_grace_s <= 0.0 or
                self.loss_grace_s > 2.0):
            raise ValueError(
                'marker timeout/grace must be positive and grace <= 2 seconds')
        if (self.recovery_samples <= 0 or
                self.recovery_samples != recovery_samples):
            raise ValueError('recovery_samples must be a positive integer')
        self.visible = False
        self.pose_time = None
        self.true_time = None
        self.false_time = None
        self.unstable_since = None
        self.recovery_count = 0

    @staticmethod
    def _stamp(now):
        value = float(now)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError('marker event time must be finite and non-negative')
        return value

    def note_pose(self, now):
        now = self._stamp(now)
        if self.pose_time is not None and now < self.pose_time:
            return False
        if (self.pose_time is not None and
                now - self.pose_time > self.timeout_s):
            self.recovery_count = 0
            if self.unstable_since is None:
                self.unstable_since = now
        self.pose_time = now
        self.true_time = now
        self.visible = True
        self.recovery_count = min(
            self.recovery_samples, self.recovery_count + 1)
        if self.recovery_count >= self.recovery_samples:
            self.unstable_since = None
        return True

    def note_visibility(self, visible, now):
        now = self._stamp(now)
        self.visible = bool(visible)
        if self.visible:
            self.true_time = now
            return
        self.false_time = now
        # Bool has no source stamp and may arrive after a newer PoseStamped.
        # Preserve that valid pose until its own bounded freshness timeout.
        if (self.pose_time is None or
                now - self.pose_time > self.timeout_s):
            self.recovery_count = 0
            if self.unstable_since is None:
                self.unstable_since = now

    def fresh(self, now):
        now = self._stamp(now)
        return (
            self.pose_time is not None and
            self.recovery_count >= self.recovery_samples and
            0.0 <= now - self.pose_time <= self.timeout_s)

    def recovery_pending(self, now):
        now = self._stamp(now)
        if self.fresh(now):
            return False
        # A camera/transport stop produces no further visibility callbacks.
        # Start a bounded zero-hold from the exact pose-expiry instant rather
        # than faulting merely because no negative Bool arrived afterwards.
        if (self.unstable_since is None and self.pose_time is not None and
                now - self.pose_time > self.timeout_s):
            self.unstable_since = self.pose_time + self.timeout_s
            self.recovery_count = 0
        if self.unstable_since is None:
            return False
        return 0.0 <= now - self.unstable_since <= self.loss_grace_s

    def unstable_age_s(self, now):
        now = self._stamp(now)
        if self.unstable_since is None:
            return None
        return max(0.0, now - self.unstable_since)


def evaluate_placement_guide(*, relative_pose, marker_fresh, stable,
                             pair_separation_m, aruco_distance_offset_m,
                             centre_tolerance_m=0.015,
                             lateral_tolerance_m=0.015,
                             yaw_tolerance_rad=math.radians(2.0)):
    """Classify a camera-based placement candidate without any motion output.

    ``relative_pose`` is the raw ID0 pose in ``rear_base``.  Only forward has
    the configured camera-to-marker offset; lateral/yaw extrinsics are not
    inferred, so the result is explicitly a candidate rather than a physical
    alignment guarantee.
    """
    offset = float(aruco_distance_offset_m)
    calibration_available = math.isfinite(offset) and offset > 0.0
    if not marker_fresh or relative_pose is None:
        return PlacementGuide('마커 찾기',
                              calibration_available=calibration_available)
    values = tuple(float(value) for value in relative_pose)
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        return PlacementGuide('마커 찾기',
                              calibration_available=calibration_available)
    if not stable:
        return PlacementGuide('안정화 중',
                              calibration_available=calibration_available)
    tolerances = (centre_tolerance_m, lateral_tolerance_m, yaw_tolerance_rad)
    if (not all(math.isfinite(float(value)) and float(value) > 0.0
                for value in tolerances) or
            not math.isfinite(float(pair_separation_m)) or
            float(pair_separation_m) <= 0.0):
        raise ValueError('placement guide tolerances/separation must be positive')
    raw_x, raw_lateral, raw_yaw = values
    if not calibration_available:
        return PlacementGuide('보정값 없음')
    centre_distance = raw_x + offset
    centre_error = centre_distance - float(pair_separation_m)
    guide = dict(
        raw_forward_m=raw_x,
        centre_distance_m=centre_distance, centre_error_m=centre_error,
        raw_lateral_error_m=raw_lateral, raw_yaw_error_rad=raw_yaw,
        calibration_available=True, estimate_available=True)
    if abs(centre_error) > float(centre_tolerance_m):
        return PlacementGuide('앞뒤 조정', **guide)
    if abs(raw_lateral) > float(lateral_tolerance_m):
        return PlacementGuide('좌우 조정', **guide)
    if abs(raw_yaw) > float(yaw_tolerance_rad):
        return PlacementGuide('yaw 조정', **guide)
    return PlacementGuide('정렬 후보', **guide)


def angle_norm(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def median_relative_pose(samples):
    """Median-filter measured ArUco pose while respecting yaw wraparound."""
    values = [tuple(float(value) for value in sample) for sample in samples]
    if not values or any(len(sample) != 3 for sample in values):
        raise ValueError('relative pose samples must contain three axes')
    if not all(math.isfinite(value) for sample in values for value in sample):
        raise ValueError('relative pose samples must be finite')
    anchor = values[-1][2]
    unwrapped_yaws = [
        anchor + angle_norm(sample[2] - anchor) for sample in values]
    return (
        statistics.median(sample[0] for sample in values),
        statistics.median(sample[1] for sample in values),
        angle_norm(statistics.median(unwrapped_yaws)),
    )


def relative_pose_step_is_plausible(
        previous, candidate, *, forward_step_m=0.020,
        lateral_step_m=0.020, yaw_step_rad=math.radians(3.0)):
    """Reject camera pose jumps faster than the bounded pair can produce."""
    old = tuple(float(value) for value in previous)
    new = tuple(float(value) for value in candidate)
    limits = (float(forward_step_m), float(lateral_step_m),
              float(yaw_step_rad))
    if (len(old) != 3 or len(new) != 3 or
            not all(math.isfinite(value) for value in old + new + limits) or
            not all(value > 0.0 for value in limits)):
        raise ValueError('relative pose values/step limits must be finite')
    return (
        abs(new[0] - old[0]) <= limits[0] and
        abs(new[1] - old[1]) <= limits[1] and
        abs(angle_norm(new[2] - old[2])) <= limits[2])


def clamp(value, limit):
    limit = abs(float(limit))
    return max(-limit, min(limit, float(value)))


def smooth_deadband_correction(error, *, kp, deadband, limit):
    """Return a continuous correction based on error outside a deadband."""
    error = float(error)
    kp = float(kp)
    deadband = float(deadband)
    limit = float(limit)
    if (not all(math.isfinite(value) for value in
                (error, kp, deadband, limit)) or
            kp <= 0.0 or deadband <= 0.0 or limit <= 0.0):
        raise ValueError('correction inputs must be finite and positive')
    excess = abs(error) - deadband
    if excess <= 0.0:
        return 0.0
    return math.copysign(min(kp * excess, limit), error)


def bounded_gap_correction(limits, gap_error_m):
    """Return bounded gap feedback without a speed jump at the deadband."""
    return smooth_deadband_correction(
        gap_error_m, kp=limits.gap_kp,
        deadband=limits.gap_deadband_m,
        limit=limits.gap_correction_limit_mps)


def bounded_relative_yaw_correction(limits, yaw_error_rad):
    """Ignore measured ArUco yaw noise, then correct sustained misalignment."""
    return smooth_deadband_correction(
        yaw_error_rad, kp=limits.yaw_kp,
        deadband=limits.yaw_deadband_rad,
        limit=limits.yaw_correction_limit_rps)


def is_zero(command, tolerance=1e-9):
    return all(abs(float(value)) <= tolerance for value in command)


def request_origin_is_same_host(origin, host):
    """Allow local CLI calls; reject browser requests from another origin."""
    if not origin:
        return True
    try:
        return urlparse(str(origin)).netloc.lower() == str(host).lower()
    except (TypeError, ValueError):
        return False


class OdomPathAccumulator:
    """Accumulate measured planar path while rejecting teleport-like jumps."""

    def __init__(self, max_step_m=0.10):
        self.max_step_m = float(max_step_m)
        if not math.isfinite(self.max_step_m) or self.max_step_m <= 0.0:
            raise ValueError('max_step_m must be finite and positive')
        self.reset()

    def reset(self):
        self.distance_m = 0.0
        self._previous = None

    def add(self, pose):
        """Add an x/y pose; return False when a sample jump is unsafe."""
        x, y = (float(pose[0]), float(pose[1]))
        if not all(math.isfinite(value) for value in (x, y)):
            return False
        if self._previous is None:
            self._previous = (x, y)
            return True
        step = math.hypot(x - self._previous[0], y - self._previous[1])
        if step > self.max_step_m:
            return False
        self.distance_m += step
        self._previous = (x, y)
        return True


def relative_pose_is_stable(samples, *, forward_span_m=0.010,
                            lateral_span_m=0.010,
                            yaw_span_rad=math.radians(2.0)):
    """Require three tightly clustered ID0 samples before arming."""
    values = [tuple(float(value) for value in sample) for sample in samples]
    if len(values) < 3 or any(len(sample) != 3 for sample in values):
        return False
    if not all(math.isfinite(value) for sample in values for value in sample):
        return False
    if max(sample[0] for sample in values) - min(sample[0] for sample in values) > forward_span_m:
        return False
    if max(sample[1] for sample in values) - min(sample[1] for sample in values) > lateral_span_m:
        return False
    anchor = values[-1][2]
    yaws = [anchor + angle_norm(sample[2] - anchor) for sample in values]
    return max(yaws) - min(yaws) <= yaw_span_rad


def capture_pair_reference(relative_pose):
    """Capture the actual current ID0 forward/lateral/yaw reference exactly."""
    values = tuple(float(value) for value in relative_pose)
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError('ArUco reference must contain three finite values')
    if values[0] <= 0.0:
        raise ValueError('ArUco forward reference must be positive')
    return values


def capture_aligned_pair_reference(relative_pose):
    """Keep measured spacing while targeting zero lateral and yaw error."""
    forward, _, _ = capture_pair_reference(relative_pose)
    return (forward, 0.0, 0.0)


def pair_heading(front_yaw_rad, rear_yaw_rad):
    """Return the wrap-safe mean heading of the two wheel odometry poses."""
    front = float(front_yaw_rad)
    rear = float(rear_yaw_rad)
    if not all(math.isfinite(value) for value in (front, rear)):
        raise ValueError('pair headings must be finite')
    sine = math.sin(front) + math.sin(rear)
    cosine = math.cos(front) + math.cos(rear)
    if math.hypot(sine, cosine) <= 1e-9:
        raise ValueError('pair headings cannot be opposite')
    return math.atan2(sine, cosine)


def heading_hold_omega(limits, reference_rad, front_yaw_rad, rear_yaw_rad):
    """Return common yaw feedback that holds the pair's armed heading."""
    error = angle_norm(
        pair_heading(front_yaw_rad, rear_yaw_rad) - float(reference_rad))
    return -smooth_deadband_correction(
        error, kp=limits.heading_kp,
        deadband=limits.heading_deadband_rad,
        limit=limits.heading_correction_limit_rps)


def hold_translation_heading(limits, intent, reference_rad,
                             front_yaw_rad, rear_yaw_rad):
    """Add common heading feedback to translation, leaving Q/E unchanged."""
    values = tuple(float(value) for value in intent)
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError('pair intent must contain three finite values')
    if abs(values[2]) > 1e-9 or math.hypot(values[0], values[1]) <= 1e-9:
        return values
    return (
        values[0], values[1],
        heading_hold_omega(
            limits, reference_rad, front_yaw_rad, rear_yaw_rad))


def split_pair_centre_twist(limits, intent, separation_m, *, gap_error_m,
                            lateral_error_m, yaw_error_rad):
    """Split one virtual pair-centre twist into symmetric robot commands."""
    limits.validate()
    values = tuple(float(value) for value in intent)
    numeric = values + (
        float(separation_m), float(gap_error_m), float(lateral_error_m),
        float(yaw_error_rad))
    if len(values) != 3 or not all(math.isfinite(value) for value in numeric):
        raise ValueError('rigid-pair command values must be finite')
    if separation_m <= 0.0:
        raise ValueError('pair separation must be positive')

    front, rear = RigidBodyKinematics(separation_m).split(*values)
    gap_correction = bounded_gap_correction(limits, gap_error_m)
    lateral_correction = clamp(
        limits.lateral_kp * lateral_error_m,
        limits.linear_correction_limit_mps)
    yaw_correction = bounded_relative_yaw_correction(limits, yaw_error_rad)
    front = (
        front[0] - 0.5 * gap_correction,
        front[1] - 0.5 * lateral_correction,
        front[2] - 0.5 * yaw_correction)
    rear = (
        rear[0] + 0.5 * gap_correction,
        rear[1] + 0.5 * lateral_correction,
        rear[2] + 0.5 * yaw_correction)
    return RigidBodyKinematics.limit_twist_pair(
        front, rear, linear_limit=limits.linear_limit_mps,
        angular_limit=limits.angular_limit_rps)


def evaluate_rigid_pair(limits, *, gap_error_m, lateral_error_m,
                        yaw_error_rad, front_distance_m, rear_distance_m,
                        hardware_ok, manual_ok, marker_ok, odom_ok, graph_ok,
                        estop):
    """Return CONTINUE or a fail-closed safety decision for this cycle."""
    limits.validate()
    numeric = (gap_error_m, lateral_error_m, yaw_error_rad,
               front_distance_m, rear_distance_m)
    if not all(math.isfinite(float(value)) for value in numeric):
        return RigidPairDecision('FAULT', '비정상 수치(NaN/Inf) 수신')
    if estop:
        return RigidPairDecision('ESTOP', '비상정지 신호 감지')
    if not graph_ok:
        return RigidPairDecision('FAULT', '다른 주행 명령 발행자와 충돌')
    if not hardware_ok:
        return RigidPairDecision('FAULT', '하드웨어 준비 신호 끊김')
    if not manual_ok:
        return RigidPairDecision('FAULT', '양쪽 수동 제어권 확인 실패')
    if not marker_ok:
        return RigidPairDecision('FAULT', 'ArUco 관측이 끊기거나 오래됨')
    if not odom_ok:
        return RigidPairDecision('FAULT', '엔코더 odometry가 끊기거나 오래됨')
    if abs(float(gap_error_m)) > limits.gap_stop_m:
        return RigidPairDecision(
            'FAULT', f'로봇 간격 변화 {gap_error_m * 100.0:+.1f} cm')
    if abs(float(lateral_error_m)) > limits.lateral_stop_m:
        return RigidPairDecision(
            'FAULT', f'좌우 어긋남 {lateral_error_m * 100.0:+.1f} cm')
    if abs(float(yaw_error_rad)) > limits.yaw_stop_rad:
        return RigidPairDecision(
            'FAULT', f'상대 각도 변화 {math.degrees(yaw_error_rad):+.1f} deg')
    if max(front_distance_m, rear_distance_m) >= limits.max_session_distance_m:
        return RigidPairDecision('LIMIT', '세션 누적 이동거리 제한 도달')
    return RigidPairDecision('CONTINUE')
