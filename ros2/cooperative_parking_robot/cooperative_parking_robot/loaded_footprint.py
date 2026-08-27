#!/usr/bin/env python3
"""Fixed-yaw footprint geometry for the lifted vehicle and two robots."""

from dataclasses import dataclass
import math


def _positive_finite(name, value):
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return value


def _nonnegative_finite(name, value):
    value = float(value)
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
    return value


def _finite(name, value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


@dataclass(frozen=True)
class LoadedFootprint:
    """Axis-aligned footprint in ``base_virtual`` while yaw is held fixed."""

    length_m: float
    width_m: float
    safety_margin_m: float

    @property
    def half_length_m(self):
        return self.length_m / 2.0

    @property
    def half_width_m(self):
        return self.width_m / 2.0

    def half_extent_cells(self, resolution):
        resolution = _positive_finite("resolution", resolution)
        return (
            int(math.ceil(self.half_length_m / resolution)),
            int(math.ceil(self.half_width_m / resolution)),
        )


def compute_loaded_footprint(
        wheelbase_m,
        robot_length_m=0.565,
        robot_width_m=0.420,
        vehicle_length_m=0.90,
        vehicle_width_m=0.35,
        safety_margin_m=0.06,
        vehicle_center_offset_x_m=0.0,
        vehicle_center_offset_y_m=0.0):
    """Return the combined fixed-yaw rectangular footprint.

    Assumptions:

    * Front and Rear robot/gripper centres coincide with the respective
      vehicle axle centres.
    * ``robot_length_m`` is along vehicle front/rear (+x), and
      ``robot_width_m`` is along vehicle left/right (+y).
    * Front and Rear gripper-centre separation equals the target wheelbase.

    The robot pair spans ``wheelbase + robot_length`` longitudinally.  CCTV
    vehicle centre and the Front/Rear midpoint need not coincide, so the
    body-frame offset is included.  The returned rectangle is deliberately
    symmetric about the **vehicle control point**, which is conservative even
    when the actual union is asymmetric.
    """

    wheelbase = _positive_finite("wheelbase_m", wheelbase_m)
    robot_length = _positive_finite("robot_length_m", robot_length_m)
    robot_width = _positive_finite("robot_width_m", robot_width_m)
    vehicle_length = _positive_finite("vehicle_length_m", vehicle_length_m)
    vehicle_width = _positive_finite("vehicle_width_m", vehicle_width_m)
    margin = _nonnegative_finite("safety_margin_m", safety_margin_m)
    offset_x = _finite(
        "vehicle_center_offset_x_m", vehicle_center_offset_x_m)
    offset_y = _finite(
        "vehicle_center_offset_y_m", vehicle_center_offset_y_m)

    pair_length = wheelbase + robot_length
    # pair centre는 vehicle centre에서 -offset에 있다. 차량 제어점을
    # 중심으로 하는 대칭 외접사각형은 |offset|+반경까지 포함한다.
    half_length = max(
        vehicle_length / 2.0,
        abs(offset_x) + pair_length / 2.0) + margin
    half_width = max(
        vehicle_width / 2.0,
        abs(offset_y) + robot_width / 2.0) + margin
    loaded_length = 2.0 * half_length
    loaded_width = 2.0 * half_width

    return LoadedFootprint(
        length_m=loaded_length,
        width_m=loaded_width,
        safety_margin_m=margin,
    )
