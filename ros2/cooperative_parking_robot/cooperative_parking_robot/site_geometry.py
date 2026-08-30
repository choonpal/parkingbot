#!/usr/bin/env python3
"""Measured ceiling-camera geometry used by production vision wrappers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class CameraGeometry:
    optical_axis_ground_m: Tuple[float, float]
    optical_center_height_m: float


# 2026-08-28 replacement-camera 640x360 rectified calibration principal points
# projected through the final, post-swap registered homographies. These are
# optical-axis/floor intersections, not the physical vertical projection of
# the tilted camera housings. Production launch parameters take precedence;
# these values are only a fail-safe fallback for an otherwise unconfigured
# direct launch.
CAMERA_GEOMETRY = {
    'cam0': CameraGeometry((2.319423, 2.315810), 2.610),
    'cam2': CameraGeometry((1.891773, 1.296094), 2.610),
}

# Both overhead robot ArUco markers are centered on the robot base/rotation
# center. Only the marker plane height needs parallax correction.
ROBOT_MARKER_HEIGHT_M = 0.120
FRONT_MARKER_OFFSET_X_M = 0.0
REAR_MARKER_OFFSET_X_M = 0.0

# 2026-08-30 현장 실측 차량 높이는 0.530m지만, 대기 차량 YOLO는 존재 확인에만
# 사용하고 목표 pose는 고정한다. 검출 표시까지 이동시키는 시차 보정은 적용하지
# 않는다.
MEASURED_VEHICLE_HEIGHT_M = 0.530
VEHICLE_DETECTION_EFFECTIVE_HEIGHT_M = None
