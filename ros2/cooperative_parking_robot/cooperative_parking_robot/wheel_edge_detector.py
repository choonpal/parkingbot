"""초음파 좌/우 에지에서 바퀴 축 중심을 계산하는 순수 로직."""

from collections import deque
from dataclasses import dataclass
import math


def gripper_target_base_x(sensor_center_base_x, sensor_to_gripper_x_m):
    """센서 중심 검출 base-x를 그리퍼 중심 정렬 base-x로 변환한다.

    offset은 ``gripper_center_x - sensor_x``로 정의한다. 그리퍼가 센서보다
    +X 앞에 있으면 base는 센서 중심 검출 위치보다 offset만큼 뒤로 간다.
    """
    return float(sensor_center_base_x) - float(sensor_to_gripper_x_m)


class SideEdgeDetector:
    def __init__(self, threshold_m=0.10, exit_hysteresis_m=0.02,
                 window_size=3):
        self.threshold = float(threshold_m)
        self.exit_threshold = self.threshold + float(exit_hysteresis_m)
        self.samples = deque(maxlen=int(window_size))
        self.inside = False
        self.entry_x = None
        self.center_x = None
        self.center_time = None

    def update(self, distance_m, position_x, timestamp):
        if self.center_x is not None:
            return self.center_x
        distance_m = float(distance_m)
        if math.isnan(distance_m) or distance_m == 0.0:
            return None
        if distance_m == math.inf:
            # sensor_msgs/Range uses +inf for clear/out-of-range. In real HC-SR04
            # data that is often the first sample after leaving a wheel, so it
            # must be allowed to close an active edge.
            sample = max(1.0, self.exit_threshold + 0.10)
        elif distance_m == -math.inf:
            # -inf means an object closer than min_range: conservatively inside.
            sample = 0.0
        elif distance_m < 0.0:
            return None
        else:
            sample = distance_m
        self.samples.append(sample)
        if len(self.samples) < self.samples.maxlen:
            return None
        filtered = sum(self.samples) / len(self.samples)
        if not self.inside and filtered < self.threshold:
            self.inside = True
            self.entry_x = float(position_x)
        elif self.inside and filtered > self.exit_threshold:
            self.center_x = (self.entry_x + float(position_x)) / 2.0
            self.center_time = float(timestamp)
        return self.center_x


class DualWheelEdgeDetector:
    SIDES = ('left', 'right')

    def __init__(self, threshold_m=0.10, exit_hysteresis_m=0.02,
                 window_size=3, pair_timeout_s=1.0):
        self.threshold_m = threshold_m
        self.exit_hysteresis_m = exit_hysteresis_m
        self.window_size = window_size
        self.pair_timeout_s = float(pair_timeout_s)
        self.reset()

    def reset(self):
        self.detectors = {
            side: SideEdgeDetector(
                self.threshold_m, self.exit_hysteresis_m, self.window_size)
            for side in self.SIDES
        }

    def update(self, side, distance_m, position_x, timestamp):
        if side not in self.detectors:
            raise ValueError(f'unknown ultrasonic side: {side}')
        self.detectors[side].update(distance_m, position_x, timestamp)
        centers = [self.detectors[s].center_x for s in self.SIDES]
        if any(center is None for center in centers):
            completed = [
                self.detectors[s].center_time for s in self.SIDES
                if self.detectors[s].center_time is not None
            ]
            if completed and timestamp - min(completed) > self.pair_timeout_s:
                self.reset()
            return None
        times = [self.detectors[s].center_time for s in self.SIDES]
        if max(times) - min(times) > self.pair_timeout_s:
            self.reset()
            return None
        return sum(centers) / len(centers)


@dataclass(frozen=True)
class AxleDetection:
    index: int
    center_x: float
    final: bool


class AxleSequenceDetector:
    """Count paired wheel axes encountered along one entry direction.

    Same-side entry means Front must pass the first (rear) axle and select the
    second (front) axle. Rear follows later and selects the first axle.
    """

    def __init__(
            self,
            target_index,
            expected_spacing_m,
            spacing_tolerance_m,
            direction=1.0,
            expected_first_position_m=None,
            position_tolerance_m=None,
            **pair_kwargs):
        self.target_index = int(target_index)
        self.expected_spacing = float(expected_spacing_m)
        self.spacing_tolerance = float(spacing_tolerance_m)
        self.direction = 1.0 if float(direction) >= 0.0 else -1.0
        self.expected_first_position = (
            None if expected_first_position_m is None else
            float(expected_first_position_m))
        self.position_tolerance = (
            None if position_tolerance_m is None else
            float(position_tolerance_m))
        if self.target_index < 1:
            raise ValueError("target_index must be at least 1")
        if self.expected_spacing <= 0.0 or self.spacing_tolerance <= 0.0:
            raise ValueError("spacing and tolerance must be positive")
        if ((self.expected_first_position is None) !=
                (self.position_tolerance is None)):
            raise ValueError(
                "expected first position and tolerance must be configured together")
        if (self.expected_first_position is not None and
                (not math.isfinite(self.expected_first_position) or
                 not math.isfinite(self.position_tolerance) or
                 self.position_tolerance <= 0.0)):
            raise ValueError("invalid absolute axle position window")
        self.pair_kwargs = dict(pair_kwargs)
        self.reset()

    def reset(self):
        self.pair = DualWheelEdgeDetector(**self.pair_kwargs)
        self.centers = []

    def set_expected_spacing(self, spacing_m):
        spacing_m = float(spacing_m)
        if spacing_m <= 0.0:
            raise ValueError("expected spacing must be positive")
        if self.centers:
            raise RuntimeError("cannot change axle spacing during a scan")
        self.expected_spacing = spacing_m

    def set_expected_geometry(self, spacing_m, first_position_m):
        spacing_m = float(spacing_m)
        first_position_m = float(first_position_m)
        if (spacing_m <= 0.0 or not math.isfinite(spacing_m) or
                not math.isfinite(first_position_m)):
            raise ValueError("invalid expected axle geometry")
        if self.centers:
            raise RuntimeError("cannot change axle geometry during a scan")
        self.expected_spacing = spacing_m
        self.expected_first_position = first_position_m

    def update(self, side, distance_m, position_x, timestamp):
        center = self.pair.update(side, distance_m, position_x, timestamp)
        if center is None:
            return None

        if self.expected_first_position is not None:
            expected_absolute = (
                self.expected_first_position +
                self.direction * len(self.centers) * self.expected_spacing)
            if abs(float(center) - expected_absolute) > self.position_tolerance:
                self.pair.reset()
                return None

        # Rear needs only the first pair. For Front, reject a second object
        # that is not approximately one wheelbase beyond the first pair.
        if self.centers:
            expected = (
                self.centers[-1] + self.direction * self.expected_spacing)
            if abs(float(center) - expected) > self.spacing_tolerance:
                self.pair.reset()
                return None

        self.centers.append(float(center))
        index = len(self.centers)
        event = AxleDetection(
            index=index,
            center_x=float(center),
            final=index == self.target_index,
        )
        self.pair.reset()
        return event
