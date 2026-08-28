#!/usr/bin/env python3
'''Regression tests for localization math that do not require ROS2.'''

import math
from pathlib import Path
import sys
import unittest

try:
    import cv2
    import numpy as np
    HAS_OPENCV = True
except ImportError:
    HAS_OPENCV = False


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from cooperative_parking_robot.aruco_utils import (  # noqa: E402
    ArucoDetectorCompat,
    apply_relative_pose_alignment,
    marker_center_to_base_link,
    relative_yaw_from_rotation,
)
from cooperative_parking_robot.encoder_odometry import EncoderOdometry  # noqa: E402
from cooperative_parking_robot.kalman_filter import PoseEKF, ScalarKalman  # noqa: E402


class LocalizationMathTest(unittest.TestCase):
    def test_aligned_pose_calibration_zeros_measured_mounting_bias(self):
        forward, lateral, yaw = apply_relative_pose_alignment(
            0.215930574, 0.012879378, math.radians(-3.155371),
            lateral_offset_m=-0.012879378,
            yaw_offset_rad=math.radians(3.155371))
        self.assertAlmostEqual(forward, 0.215930574)
        self.assertAlmostEqual(lateral, 0.0)
        self.assertAlmostEqual(yaw, 0.0)

    def test_aligned_pose_calibration_rejects_nonfinite_values(self):
        with self.assertRaises(ValueError):
            apply_relative_pose_alignment(
                0.2, float('nan'), 0.0,
                lateral_offset_m=0.0, yaw_offset_rad=0.0)

    def test_rear_marker_yaw_uses_negative_normal(self):
        for expected_deg in (0.0, 20.0, -20.0):
            yaw = math.radians(expected_deg)
            # Ry(yaw) @ Rx(pi), matching the solvePnP object-point convention.
            rotation = [
                [math.cos(yaw), 0.0, -math.sin(yaw)],
                [0.0, -1.0, 0.0],
                [-math.sin(yaw), 0.0, -math.cos(yaw)],
            ]
            actual = relative_yaw_from_rotation(rotation)
            self.assertAlmostEqual(actual, yaw, places=9)

    @unittest.skipUnless(HAS_OPENCV, 'OpenCV/NumPy not installed')
    def test_rear_marker_solvepnp_yaw(self):
        half = 0.025
        object_points = np.array([
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)
        expected = math.radians(20.0)
        rotation = np.array([
            [math.cos(expected), 0.0, -math.sin(expected)],
            [0.0, -1.0, 0.0],
            [-math.sin(expected), 0.0, -math.cos(expected)],
        ])
        rvec, _ = cv2.Rodrigues(rotation)
        camera_matrix = np.array([
            [800.0, 0.0, 640.0],
            [0.0, 800.0, 360.0],
            [0.0, 0.0, 1.0],
        ])
        image_points, _ = cv2.projectPoints(
            object_points, rvec, np.array([[0.0], [0.0], [1.0]]),
            camera_matrix, np.zeros(5))
        ok, estimated_rvec, _ = cv2.solvePnP(
            object_points, image_points.reshape(-1, 2),
            camera_matrix, np.zeros(5), flags=cv2.SOLVEPNP_ITERATIVE)
        self.assertTrue(ok)
        estimated_rotation, _ = cv2.Rodrigues(estimated_rvec)
        actual = relative_yaw_from_rotation(estimated_rotation)
        self.assertAlmostEqual(actual, expected, places=6)

    def test_marker_offset_zero_keeps_marker_center(self):
        # offset 0 → 마커 중심이 곧 base_link (기존 동작 하위호환)
        for yaw in (0.0, math.pi / 3.0, -2.0):
            bx, by = marker_center_to_base_link(1.5, -0.7, yaw, 0.0)
            self.assertAlmostEqual(bx, 1.5, places=12)
            self.assertAlmostEqual(by, -0.7, places=12)

    def test_front_edge_marker_shifts_back_along_heading(self):
        # yaw=0에서 앞끝(+x) 마커(offset +0.10)면 base_link은 마커보다 -x로 0.10
        bx, by = marker_center_to_base_link(2.0, 0.5, 0.0, 0.10)
        self.assertAlmostEqual(bx, 1.90, places=12)
        self.assertAlmostEqual(by, 0.50, places=12)
        # yaw=90°면 오프셋이 world +y 축을 따르므로 y가 줄어든다
        bx, by = marker_center_to_base_link(2.0, 0.5, math.pi / 2.0, 0.10)
        self.assertAlmostEqual(bx, 2.00, places=12)
        self.assertAlmostEqual(by, 0.40, places=12)

    def test_rear_edge_marker_uses_negative_offset(self):
        # 뒤끝(−x) 마커(offset −0.10)면 base_link은 마커보다 +x로 0.10
        bx, by = marker_center_to_base_link(0.5, 0.0, 0.0, -0.10)
        self.assertAlmostEqual(bx, 0.60, places=12)
        self.assertAlmostEqual(by, 0.0, places=12)

    def test_pure_rotation_of_offset_marker_keeps_base_link_fixed(self):
        # 로봇이 제자리에서 회전해도 base_link 위치는 (거의) 고정 —
        # 마커 중심은 base_link 둘레로 offset 반경 원을 그린다.
        # 이 성질이 §7 정적 검증의 회전 테스트 근거다.
        offset = 0.12
        base_ref = (3.0, 1.0)
        for yaw in (0.0, math.radians(37.0), math.pi, -math.radians(90.0)):
            marker_x = base_ref[0] + offset * math.cos(yaw)
            marker_y = base_ref[1] + offset * math.sin(yaw)
            bx, by = marker_center_to_base_link(marker_x, marker_y, yaw, offset)
            self.assertAlmostEqual(bx, base_ref[0], places=12)
            self.assertAlmostEqual(by, base_ref[1], places=12)

    def test_scalar_kalman_reset_preserves_actual_initial_spacing(self):
        filt = ScalarKalman(init=0.25)
        filt.reset(0.27, raw_value=0.27)
        # 같은 raw 절대값의 첫 predict에서 0.25로 되돌아가면 안 된다.
        filt.predict(0.27)
        self.assertAlmostEqual(filt.x, 0.27, places=12)
        filt.predict(0.26)
        self.assertAlmostEqual(filt.x, 0.26, places=12)

    def test_encoder_reset_is_discarded(self):
        odom = EncoderOdometry()
        odom.update([10000, 10000, 10000, 10000])
        result = odom.update([0, 0, 0, 0])
        self.assertTrue(result['discontinuity'])
        self.assertEqual(result['dx_body'], 0.0)
        self.assertEqual(result['dy_body'], 0.0)
        self.assertEqual(result['dtheta'], 0.0)

    def test_extreme_repeated_outlier_cannot_force_jump(self):
        ekf = PoseEKF()
        measurement_noise = [
            [0.02 ** 2, 0.0, 0.0],
            [0.0, 0.02 ** 2, 0.0],
            [0.0, 0.0, math.radians(3.0) ** 2],
        ]
        accepted = [
            ekf.correct(100.0, 100.0, math.pi / 2.0, measurement_noise)
            for _ in range(10)
        ]
        self.assertFalse(any(accepted))
        self.assertEqual(ekf.pose(), (0.0, 0.0, 0.0))

    def test_bounded_consistent_measurements_can_reacquire(self):
        ekf = PoseEKF()
        measurement_noise = [
            [0.02 ** 2, 0.0, 0.0],
            [0.0, 0.02 ** 2, 0.0],
            [0.0, 0.0, math.radians(3.0) ** 2],
        ]
        # 재획득 게이트는 "이미 수렴한 필터"에만 의미가 있다. 첫 측정은
        # 보정이 아니라 절대 초기화이므로 먼저 한 번 수렴시킨다.
        self.assertTrue(ekf.correct(0.0, 0.0, 0.0, measurement_noise))
        self.assertTrue(ekf.initialized)
        ekf.P = [
            [1e-4, 0.0, 0.0],
            [0.0, 1e-4, 0.0],
            [0.0, 0.0, 1e-4],
        ]
        results = [
            ekf.correct(0.3, 0.0, 0.0, measurement_noise)
            for _ in range(5)
        ]
        self.assertEqual(results, [False, False, False, False, True])
        self.assertTrue(ekf.forced_reacquire)

    def test_first_fix_initializes_beyond_chi2_gate(self):
        """초기 배치 오차가 커도 첫 절대측정은 반드시 수용돼야 한다."""
        ekf = PoseEKF()
        measurement_noise = [
            [0.02 ** 2, 0.0, 0.0],
            [0.0, 0.02 ** 2, 0.0],
            [0.0, 0.0, math.radians(3.0) ** 2],
        ]
        self.assertFalse(ekf.initialized)
        # 초기 P=0.25 기준 chi2 게이트를 크게 벗어나는 배치 오차.
        self.assertTrue(ekf.correct(3.0, 2.0, math.pi / 2.0, measurement_noise))
        x, y, yaw = ekf.pose()
        self.assertAlmostEqual(x, 3.0)
        self.assertAlmostEqual(y, 2.0)
        self.assertAlmostEqual(yaw, math.pi / 2.0)
        self.assertTrue(ekf.initialized)

    def test_first_fix_still_rejects_out_of_workspace(self):
        """첫 측정이라도 작업공간 밖 좌표는 초기값으로 삼지 않는다."""
        ekf = PoseEKF()
        self.assertFalse(ekf.correct(100.0, 100.0, 0.0))
        self.assertFalse(ekf.initialized)
        self.assertEqual(ekf.pose(), (0.0, 0.0, 0.0))

    def test_opencv_46_detector_fallback(self):
        class LegacyAruco:
            DICT_4X4_50 = 0
            constructor_calls = []

            class DetectorParameters:
                def __init__(self):
                    LegacyAruco.constructor_calls.append('new')

            def DetectorParameters_create(self):
                self.constructor_calls.append('legacy')
                return object()

            def getPredefinedDictionary(self, dictionary_id):
                return dictionary_id

            def detectMarkers(self, image, dictionary, parameters=None):
                self.last_parameters = parameters
                return ['corners'], ['ids'], ['rejected']

        class LegacyCv2:
            aruco = LegacyAruco()

        detector = ArucoDetectorCompat(LegacyCv2(), 'DICT_4X4_50')
        self.assertEqual(LegacyCv2.aruco.constructor_calls, ['legacy'])
        self.assertEqual(
            detector.detect_markers('image'),
            (['corners'], ['ids'], ['rejected']),
        )


if __name__ == '__main__':
    unittest.main()


def test_encoder_odometry_rejects_invalid_calibration_values():
    import pytest
    with pytest.raises(ValueError):
        EncoderOdometry(wheel_radius=0.0)
    with pytest.raises(ValueError):
        EncoderOdometry(ppr=0.0)
    with pytest.raises(ValueError):
        EncoderOdometry(lx=0.0, ly=0.0)
    with pytest.raises(ValueError):
        EncoderOdometry(max_delta_ticks=0)
    with pytest.raises(ValueError):
        EncoderOdometry(axis_sign=(-1.0, 1.0))


def test_encoder_odometry_applies_hardware_axis_sign_to_body_delta():
    import pytest
    raw_odom = EncoderOdometry(wheel_radius=0.05, ppr=100.0)
    corrected_odom = EncoderOdometry(
        wheel_radius=0.05, ppr=100.0,
        axis_sign=(-1.0, -1.0, -1.0))
    raw_odom.update([0, 0, 0, 0])
    corrected_odom.update([0, 0, 0, 0])

    raw = raw_odom.update([-10, -10, -10, -10])
    corrected = corrected_odom.update([-10, -10, -10, -10])
    assert raw['dx_body'] < 0.0
    assert corrected['dx_body'] == pytest.approx(-raw['dx_body'])
    assert corrected['dy_body'] == pytest.approx(0.0)
    assert corrected['dtheta'] == pytest.approx(0.0)


def test_stm32_bridge_wires_command_axis_sign_into_odometry():
    source = (
        PACKAGE_ROOT / 'cooperative_parking_robot/stm32_bridge_node.py'
    ).read_text()
    assert 'axis_sign=self.command_sign' in source
