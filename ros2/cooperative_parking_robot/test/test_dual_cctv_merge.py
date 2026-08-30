#!/usr/bin/env python3
"""천장 CCTV 2대 병합(v1.11) 회귀 테스트.

여기서 지키려는 안전 성질은 세 가지다.
  1. 겹침 영역의 같은 차량이 장애물 두 개로 남지 않는다.
  2. 어떤 카메라도 보지 못하는 슬롯은 절대 "빈자리"로 발행되지 않는다.
  3. 단일 카메라 구성의 동작/토픽이 그대로 유지된다(하위호환).
"""

import math
import json
import re
from pathlib import Path
from types import SimpleNamespace

import pytest

from cooperative_parking_robot.bev_fusion_core import (
    DETECTION_ENVELOPE_VERSION,
    CameraDetection,
    SlotOccupancyTracker,
    TargetLatchTracker,
    VehicleDimensionTracker,
    coverage_grid_values,
    decode_detection_envelope,
    encode_detection_envelope,
    image_corner_coverage,
    merge_detections,
    perception_is_available,
    point_in_polygon,
    slot_observability,
    summarize_merge,
    target_presence_state,
)
from cooperative_parking_robot.cctv_merge_node import CctvMergeNode

ROOT = Path(__file__).resolve().parents[1]

SLOT_POLYGONS = {
    # cam0 시야 안쪽
    'P1': [(1.0, 3.2), (2.0, 3.2), (2.0, 3.8), (1.0, 3.8)],
    # cam2 시야 안쪽
    'P4': [(5.0, 3.2), (6.0, 3.2), (6.0, 3.8), (5.0, 3.8)],
}
COVERAGE_CAM0 = [(0.0, 0.0), (3.5, 0.0), (3.5, 4.0), (0.0, 4.0)]
COVERAGE_CAM2 = [(2.5, 0.0), (6.5, 0.0), (6.5, 4.0), (2.5, 4.0)]


# ======================================================================
# 검출 envelope
# ======================================================================

def test_detection_envelope_round_trip_preserves_world_coordinates():
    detection = CameraDetection(
        'cam0', (1.0, 2.0),
        polygon=[(0.5, 1.5), (1.5, 1.5), (1.5, 2.5), (0.5, 2.5)],
        yaw=0.1, length_m=0.9, width_m=0.35, confidence=0.9, axis_dist_m=0.4)
    text = encode_detection_envelope(
        'cam0', 1234, 7, COVERAGE_CAM0, [detection])
    decoded = decode_detection_envelope(text)

    assert decoded['version'] == DETECTION_ENVELOPE_VERSION
    assert decoded['camera_id'] == 'cam0'
    assert decoded['stamp_ns'] == 1234
    assert decoded['coverage_polygon'] == [
        (0.0, 0.0), (3.5, 0.0), (3.5, 4.0), (0.0, 4.0)]
    assert len(decoded['detections']) == 1
    assert decoded['detections'][0].center == (1.0, 2.0)
    assert decoded['detections'][0].length_m == pytest.approx(0.9)


def test_coverage_grid_honors_nonzero_origin_and_ceil_sized_edge():
    polygon = {'cam0': [(-0.4, -0.8), (3.43, -0.8),
                         (3.43, 0.2), (-0.4, 0.2)]}
    width = math.ceil(3.83 / 0.05)
    values = coverage_grid_values(
        width, 20, 0.05, polygon, origin_x_m=-0.4, origin_y_m=-0.8)
    assert width == 77
    assert values[0] == 0
    assert values[width - 1] == 0


def test_detection_envelope_rejects_unknown_version():
    text = encode_detection_envelope('cam0', 1, 1, None, [])
    broken = text.replace('"version": 1', '"version": 99')
    with pytest.raises(ValueError, match='unsupported detection envelope'):
        decode_detection_envelope(broken)


def test_detection_envelope_requires_camera_id():
    with pytest.raises(ValueError, match='camera_id'):
        decode_detection_envelope('{"version": 1, "camera_id": ""}')


# ======================================================================
# 중복 제거
# ======================================================================

def test_overlap_region_vehicle_is_merged_into_one_obstacle():
    """겹침 영역의 같은 차량이 장애물 2개로 남으면 A*가 통로를 막는다."""
    near = CameraDetection(
        'cam0', (3.00, 2.00),
        polygon=[(2.6, 1.8), (3.4, 1.8), (3.4, 2.2), (2.6, 2.2)],
        axis_dist_m=0.4)
    far = CameraDetection(
        'cam2', (3.08, 2.03),
        polygon=[(2.7, 1.8), (3.5, 1.8), (3.5, 2.3), (2.7, 2.3)],
        axis_dist_m=1.8)
    other = CameraDetection(
        'cam2', (5.00, 3.00),
        polygon=[(4.6, 2.8), (5.4, 2.8), (5.4, 3.2), (4.6, 3.2)],
        axis_dist_m=0.5)

    merged = merge_detections([near, far, other])
    assert len(merged) == 2

    duplicated = [item for item in merged if len(item.sources) > 1]
    assert len(duplicated) == 1
    assert set(duplicated[0].sources) == {'cam0', 'cam2'}
    # 광축에 가까운 cam0 관측을 채택해야 parallax 오차가 작다.
    assert duplicated[0].center == pytest.approx((3.00, 2.00))


def test_center_blend_mixes_positions_when_requested():
    near = CameraDetection('cam0', (3.00, 2.00), axis_dist_m=0.4)
    far = CameraDetection('cam2', (3.20, 2.00), axis_dist_m=1.8)
    merged = merge_detections([near, far], center_blend=0.5)
    assert len(merged) == 1
    assert merged[0].center[0] == pytest.approx(3.10)


def test_distinct_vehicles_are_not_merged():
    a = CameraDetection('cam0', (1.0, 1.0), axis_dist_m=0.2)
    b = CameraDetection('cam2', (4.0, 1.0), axis_dist_m=0.2)
    assert len(merge_detections([a, b])) == 2


def test_waiting_flag_is_or_across_cameras():
    """경계에 걸친 차량을 한 카메라가 밖으로 봤다고 타겟에서 빼면 안 된다."""
    inside = CameraDetection('cam0', (2.30, 0.60), in_waiting=True,
                             axis_dist_m=0.3)
    outside = CameraDetection('cam2', (2.32, 0.61), in_waiting=False,
                              axis_dist_m=1.9)
    merged = merge_detections([inside, outside])
    assert len(merged) == 1
    assert merged[0].in_waiting is True


# ======================================================================
# coverage / 슬롯 관측 자격
# ======================================================================

def test_coverage_polygon_is_derived_from_homography():
    homography = [[0.005, 0.0, 0.0], [0.0, 0.005, 0.0], [0.0, 0.0, 1.0]]
    coverage = image_corner_coverage(homography, 1280, 720, margin_px=0.0)
    assert coverage[0] == pytest.approx((0.0, 0.0))
    assert coverage[2] == pytest.approx((6.395, 3.595))
    assert point_in_polygon(3.0, 2.0, coverage)
    assert not point_in_polygon(7.0, 2.0, coverage)


def test_coverage_margin_shrinks_the_polygon():
    homography = [[0.005, 0.0, 0.0], [0.0, 0.005, 0.0], [0.0, 0.0, 1.0]]
    full = image_corner_coverage(homography, 1280, 720, margin_px=0.0)
    inset = image_corner_coverage(homography, 1280, 720, margin_px=20.0)
    assert inset[0][0] > full[0][0]
    assert inset[2][0] < full[2][0]


def test_slot_observability_tracks_which_camera_sees_each_slot():
    both = slot_observability(
        SLOT_POLYGONS, {'cam0': COVERAGE_CAM0, 'cam2': COVERAGE_CAM2})
    assert both == {'P1': True, 'P4': True}

    cam0_only = slot_observability(SLOT_POLYGONS, {'cam0': COVERAGE_CAM0})
    assert cam0_only == {'P1': True, 'P4': False}


def test_coverage_grid_marks_only_live_camera_view_as_free():
    values = coverage_grid_values(
        6, 4, 1.0,
        {'cam0': [(0.0, 0.0), (3.0, 0.0), (3.0, 4.0), (0.0, 4.0)]})
    assert values[1 * 6 + 1] == 0
    assert values[1 * 6 + 4] == -1
    assert coverage_grid_values(2, 2, 1.0, {}) == [-1, -1, -1, -1]


def test_require_full_slot_coverage_is_stricter():
    # P1을 반만 덮는 시야
    partial = [(0.0, 0.0), (1.6, 0.0), (1.6, 4.0), (0.0, 4.0)]
    loose = slot_observability({'P1': SLOT_POLYGONS['P1']}, {'cam0': partial})
    strict = slot_observability(
        {'P1': SLOT_POLYGONS['P1']}, {'cam0': partial}, require_full_slot=True)
    assert loose['P1'] is True    # 중심만 보이면 통과
    assert strict['P1'] is False  # 네 모서리를 다 봐야 통과


# ======================================================================
# 안전 성질: 관측 불가 슬롯은 빈자리가 아니다
# ======================================================================

def test_unobserved_slot_is_never_reported_empty():
    """cam2가 죽어도 P4를 '빈자리'로 발행하면 로봇이 차 있는 칸으로 간다."""
    tracker = SlotOccupancyTracker(
        ['P1', 'P4'], empty_confirm_frames=3, occupied_hold_s=0.0, now=0.0)
    cam0_only = slot_observability(SLOT_POLYGONS, {'cam0': COVERAGE_CAM0})

    for step in range(10):
        tracker.update(SLOT_POLYGONS, [], cam0_only, now=1.0 + step)

    assert tracker.empty_slot_ids() == ['P1']
    assert tracker.state['P4']['observed'] is False


def test_slot_becomes_empty_again_after_camera_recovers():
    tracker = SlotOccupancyTracker(
        ['P1', 'P4'], empty_confirm_frames=3, occupied_hold_s=0.0, now=0.0)
    cam0_only = slot_observability(SLOT_POLYGONS, {'cam0': COVERAGE_CAM0})
    both = slot_observability(
        SLOT_POLYGONS, {'cam0': COVERAGE_CAM0, 'cam2': COVERAGE_CAM2})

    for step in range(10):
        tracker.update(SLOT_POLYGONS, [], cam0_only, now=1.0 + step)
    for step in range(5):
        tracker.update(SLOT_POLYGONS, [], both, now=20.0 + step)

    assert sorted(tracker.empty_slot_ids()) == ['P1', 'P4']


def test_vehicle_overlap_marks_slot_occupied_immediately():
    tracker = SlotOccupancyTracker(
        ['P1', 'P4'], empty_confirm_frames=3, occupied_hold_s=0.0, now=0.0)
    both = slot_observability(
        SLOT_POLYGONS, {'cam0': COVERAGE_CAM0, 'cam2': COVERAGE_CAM2})
    for step in range(5):
        tracker.update(SLOT_POLYGONS, [], both, now=1.0 + step)
    assert sorted(tracker.empty_slot_ids()) == ['P1', 'P4']

    parked = merge_detections([CameraDetection(
        'cam0', (1.5, 3.5),
        polygon=[(1.1, 3.25), (1.9, 3.25), (1.9, 3.75), (1.1, 3.75)])])
    tracker.update(SLOT_POLYGONS, parked, both, now=10.0)
    assert tracker.empty_slot_ids() == ['P4']


def test_slot_starts_occupied_before_any_observation():
    """부팅 직후 아무것도 못 본 상태에서 빈자리를 만들어내면 안 된다."""
    tracker = SlotOccupancyTracker(['P1'], now=0.0)
    assert tracker.empty_slot_ids() == []
    assert tracker.state['P1']['occupied'] is True


# ======================================================================
# 타겟 latch / 차량 치수 (단일 카메라 규칙과 동일해야 함)
# ======================================================================

def test_target_latches_only_after_stationary_hold():
    latch = TargetLatchTracker(
        stationary_tolerance_m=0.02, stationary_hold_s=2.0)
    assert latch.update((2.3, 0.6), 0.0) is None
    assert latch.update((2.3, 0.6), 1.0) is None
    assert latch.update((2.3, 0.6), 2.5) is not None
    assert latch.just_latched is True


def test_moving_target_resets_the_hold_window():
    latch = TargetLatchTracker(
        stationary_tolerance_m=0.02, stationary_hold_s=2.0)
    latch.update((2.30, 0.60), 0.0)
    latch.update((2.50, 0.60), 1.9)   # 20cm 이동 — 앵커 재설정
    assert latch.update((2.50, 0.60), 2.5) is None


def test_pre_mission_latch_expires_but_active_mission_can_preserve_it():
    latch = TargetLatchTracker(
        stationary_tolerance_m=0.02, stationary_hold_s=0.0,
        detection_timeout_s=0.5)
    assert latch.update((2.3, 0.6), 0.0) is None
    assert latch.update((2.3, 0.6), 0.0) == pytest.approx((2.3, 0.6))
    assert latch.update(None, 0.4) == pytest.approx((2.3, 0.6))
    assert latch.update(None, 0.6) is None

    latch.update((2.3, 0.6), 1.0)
    latch.update((2.3, 0.6), 1.0)
    assert latch.update(None, 10.0, preserve_latched=True) == pytest.approx(
        (2.3, 0.6))


def test_intermittent_noisy_detection_can_reach_ready_with_filter_and_grace():
    latch = TargetLatchTracker(
        stationary_tolerance_m=0.04,
        stationary_hold_s=2.0,
        detection_timeout_s=1.2,
        position_filter_window=3)
    observations = [
        (0.00, (0.420, 0.680)),
        (0.35, None),
        (0.70, (0.430, 0.690)),
        (1.05, None),
        (1.40, (0.410, 0.680)),
        (1.75, None),
        (2.10, (0.440, 0.690)),
    ]

    result = None
    for now, target in observations:
        result = latch.update(target, now)

    assert result is not None
    assert latch.just_latched


def test_visible_movement_revokes_ready_without_waiting_for_miss_timeout():
    latch = TargetLatchTracker(
        stationary_tolerance_m=0.04,
        stationary_hold_s=0.0,
        detection_timeout_s=1.2,
        position_filter_window=3)
    assert latch.update((0.42, 0.68), 0.0) is None
    assert latch.update((0.42, 0.68), 0.0) is not None

    assert latch.update((0.52, 0.68), 0.1) is None
    assert latch.latched is None


def test_target_presence_states_distinguish_absent_ready_and_unavailable():
    assert target_presence_state(False, False, True) == 'ABSENT'
    assert target_presence_state(False, True, True) == 'DETECTING'
    assert target_presence_state(True, True, True) == 'READY'
    assert target_presence_state(False, False, False) == (
        'PERCEPTION_UNAVAILABLE')
    # A stale stream may never retain READY from its previous frame.
    assert target_presence_state(True, True, False) == (
        'PERCEPTION_UNAVAILABLE')


@pytest.mark.parametrize('stale_camera', ['cam0', 'cam2'])
def test_required_camera_timeout_makes_perception_unavailable(stale_camera):
    states = {'cam0': {'alive': True}, 'cam2': {'alive': True}}
    states[stale_camera]['alive'] = False

    assert not perception_is_available(states, require_all_cameras=True)
    assert target_presence_state(True, True, False) == (
        'PERCEPTION_UNAVAILABLE')


def test_target_presence_recovers_using_only_new_healthy_observations():
    assert perception_is_available(
        {'cam0': {'alive': True}, 'cam2': {'alive': True}},
        require_all_cameras=True)
    assert target_presence_state(False, True, False) == (
        'PERCEPTION_UNAVAILABLE')
    assert target_presence_state(False, True, True) == 'DETECTING'
    assert target_presence_state(True, True, True) == 'READY'
    assert target_presence_state(False, False, True) == 'ABSENT'


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class ResetTracker:
    def __init__(self):
        self.reset_count = 0
        self.stable_since = None
        self.tolerance = 0.05
        self.hold_s = 1.0
        self.timeout_s = 1.0

    def reset(self):
        self.reset_count += 1


def test_publish_fail_closed_resets_trackers_and_publishes_unavailable():
    node = CctvMergeNode.__new__(CctvMergeNode)
    node.target_tracker = ResetTracker()
    node.dimension_tracker = ResetTracker()
    node.spec_sent = True
    node.spec_last_publish_wall = 1.0
    node.latest_vehicle_class = 'car'
    node.latest_classified_wheelbase = 0.7
    node.fixed_wheelbase = 0.6
    node._last_target_ready = True
    node._last_target_state = 'READY'
    node._target_last_observed_wall = 1.0
    node.target_presence_timeout_s = 1.0
    node.pub_target_ready = RecordingPublisher()
    node.pub_target_status = RecordingPublisher()
    node._publish_empty_slots = lambda **_kwargs: None
    node._publish_map = lambda *_args: None
    node._publish_status = lambda *_args: None
    node.get_logger = lambda: SimpleNamespace(info=lambda *_args: None)

    CctvMergeNode._publish_fail_closed(node, {
        'cam0': {'alive': False}, 'cam2': {'alive': True}})

    status = json.loads(node.pub_target_status.messages[-1].data)
    assert node.target_tracker.reset_count == 1
    assert node.dimension_tracker.reset_count == 1
    assert node.pub_target_ready.messages[-1].data is False
    assert node._last_target_ready is False
    assert status['state'] == 'PERCEPTION_UNAVAILABLE'
    assert status['ready'] is False


def test_mission_snapshot_only_owns_an_approved_active_park():
    node = CctvMergeNode.__new__(CctvMergeNode)
    node.mission_snapshot = {'merged': []}
    node.target_tracker = SimpleNamespace(latched=(2.3, 0.6))

    node.fleet_state = 'WAIT_TARGET'
    assert not CctvMergeNode._mission_snapshot_active(node)
    node.fleet_state = 'WAIT_LIFT'
    assert CctvMergeNode._mission_snapshot_active(node)
    node.target_tracker.latched = None
    assert not CctvMergeNode._mission_snapshot_active(node)


def test_perception_suspend_command_is_edge_triggered():
    node = CctvMergeNode.__new__(CctvMergeNode)
    node.perception_suspended = False
    node.pub_perception_suspend = RecordingPublisher()
    node.get_logger = lambda: SimpleNamespace(info=lambda *_args: None)

    CctvMergeNode._set_perception_suspended(node, True)
    CctvMergeNode._set_perception_suspended(node, True)
    CctvMergeNode._set_perception_suspended(node, False)

    assert [msg.data for msg in node.pub_perception_suspend.messages] == [
        True, False]


def test_dual_launch_fails_closed_when_any_camera_is_missing_by_default():
    dual = (ROOT / 'launch/cctv_server_dual.launch.py').read_text()
    assert "'require_all_cameras', default_value='true'" in dual
    merge = (ROOT / 'cooperative_parking_robot/cctv_merge_node.py').read_text()
    assert "declare_parameter('require_all_cameras', True)" in merge


def test_dual_launch_uses_one_shared_tensorrt_model():
    dual = (ROOT / 'launch/cctv_server_dual.launch.py').read_text()
    assert "executable='shared_yolo_bev_map'" in dual
    assert "'enable_shared_yolo', default_value='true'" in dual
    assert "'shared_inference_rate_hz', default_value='4.0'" in dual
    assert 'TimerAction' not in dual
    assert 'yolo_cam2_start_delay_s' not in dual


def test_vehicle_dimensions_reject_out_of_range_masks():
    tracker = VehicleDimensionTracker(0.90, 0.35, padding_m=0.03)
    for _ in range(7):
        assert tracker.update_dimensions(0.88, 0.33) is True
        assert tracker.dimension_valid is False
    assert tracker.update_dimensions(0.88, 0.33) is True
    assert tracker.length_m == pytest.approx(0.94)
    assert tracker.update_dimensions(99.0, 0.30) is False
    assert tracker.length_m == pytest.approx(0.94)


def test_vehicle_dimension_window_rejects_single_oversized_outlier():
    tracker = VehicleDimensionTracker(0.90, 0.35, padding_m=0.03)
    samples = [(0.88, 0.33)] * 7 + [(2.0, 1.0)]
    for length, width in samples:
        tracker.update_dimensions(length, width)
    assert tracker.dimension_valid
    assert tracker.length_m == pytest.approx(0.94)
    assert tracker.width_m == pytest.approx(0.39)


def test_camera_dimension_windows_are_independent():
    trackers = {
        camera: VehicleDimensionTracker(0.90, 0.35)
        for camera in ('cam0', 'cam2')}
    for _ in range(8):
        trackers['cam0'].update_dimensions(0.88, 0.33)
    trackers['cam2'].update_dimensions(1.20, 0.50)
    assert trackers['cam0'].dimension_valid
    assert not trackers['cam2'].dimension_valid


def test_vehicle_yaw_ema_handles_axis_wraparound():
    """+89도와 -89도는 같은 장축이다. 평균이 0도로 무너지면 안 된다."""
    tracker = VehicleDimensionTracker(0.90, 0.35, yaw_alpha=0.5)
    tracker.update_yaw(math.radians(89.0))
    tracker.update_yaw(math.radians(-89.0))
    assert abs(math.degrees(tracker.yaw)) > 80.0


# ======================================================================
# 진단
# ======================================================================

def test_merge_status_reports_dead_camera_and_duplicates():
    merged = merge_detections([
        CameraDetection('cam0', (3.0, 2.0), axis_dist_m=0.4),
        CameraDetection('cam2', (3.05, 2.0), axis_dist_m=1.8),
    ])
    tracker = SlotOccupancyTracker(['P1', 'P4'], now=0.0)
    import json
    status = json.loads(summarize_merge(
        {'cam0': {'alive': True, 'age_s': 0.05, 'detections': 1,
                  'coverage_ready': True},
         'cam2': {'alive': False, 'age_s': 9.9, 'detections': 0,
                  'coverage_ready': False}},
        merged, tracker.state, 123))

    assert status['cameras']['cam2']['alive'] is False
    assert status['duplicates_removed'] == 1
    assert status['multi_camera_detections'] == 1
    assert set(status['slots']) == {'P1', 'P4'}


# ======================================================================
# 하위호환 / 배선 검증 (소스 정적 검사)
# ======================================================================

def test_single_camera_defaults_are_unchanged():
    """기존 cctv_server.launch.py 경로가 그대로 동작해야 한다."""
    source = (ROOT / 'cooperative_parking_robot/yolo_bev_map_node.py').read_text()
    assert "declare_parameter('publish_detections', False)" in source
    assert "declare_parameter('publish_mission_outputs', True)" in source
    assert "declare_parameter('image_topic', '/cctv/image_rect')" in source
    # 단일 카메라 launch는 새 파라미터를 건드리지 않는다.
    single = (ROOT / 'launch/cctv_server.launch.py').read_text()
    assert 'publish_detections' not in single
    assert 'cctv_merge' not in single


def test_only_one_publisher_owns_each_mission_topic():
    """Sensor 인스턴스는 /parking/* publisher를 아예 만들지 않아야 한다."""
    source = (ROOT / 'cooperative_parking_robot/yolo_bev_map_node.py').read_text()
    assert 'if self.publish_mission_outputs:' in source
    assert 'self.pub_map = None' in source
    dual = (ROOT / 'launch/cctv_server_dual.launch.py').read_text()
    assert "'publish_mission_outputs': False" in dual
    # 상판 마커 노드는 카메라가 2대여도 인스턴스 하나만 띄운다.
    assert dual.count("executable='cctv_robot_marker'") == 1
    assert dual.count("executable='cctv_merge'") == 1
    assert dual.count("executable='shared_yolo_bev_map'") == 1
    assert dual.count("executable='yolo_bev_map'") == 0
    assert dual.count("executable='cctv_rectify'") == 2


def test_dual_launch_uses_separate_calibration_per_camera():
    dual = (ROOT / 'launch/cctv_server_dual.launch.py').read_text()
    assert 'cctv0_camera_calibration.npz' in dual
    assert 'cctv2_camera_calibration.npz' in dual
    assert 'homography_cam0_rectified.npy' in dual
    assert 'homography_cam2_rectified.npy' in dual
    # 두 카메라가 하나의 layout(=하나의 map frame)을 공유한다.
    assert dual.count("'layout_config'") >= 3


def test_marker_node_supports_multiple_cameras():
    source = (
        ROOT / 'cooperative_parking_robot/cctv_robot_marker_node.py').read_text()
    assert "declare_parameter('image_topics_csv', '')" in source
    assert "declare_parameter('homography_files_csv', '')" in source
    assert 'def _selection_cost' in source
    assert 'def _publish_selected' in source
    # 하위호환 속성 유지
    assert 'self.image_topic' in source


def test_calibrator_supports_appending_second_camera_layout():
    source = (
        ROOT / 'cooperative_parking_robot/bev_layout_calibrator_node.py'
    ).read_text()
    assert "declare_parameter('append_existing_layout', False)" in source
    assert 'merge_layout_registrations' in source
    launch = (ROOT / 'launch/bev_layout_calibration.launch.py').read_text()
    assert 'append_existing_layout' in launch
    assert 'camera_label' in launch


def test_registered_layout_yaml_carries_merge_node_parameters():
    core = (ROOT / 'cooperative_parking_robot/bev_layout_core.py').read_text()
    assert 'cctv_merge_node:' in core
    assert 'def load_layout_yaml' in core
    assert 'def merge_layout_registrations' in core


def test_merge_node_is_registered_as_console_script():
    setup_text = (ROOT / 'setup.py').read_text()
    # 진입점 모듈은 배포본마다 다르다. 직접 노드를 가리키기도 하고
    # (cctv_merge_node:main), 통합 래퍼를 거치기도 한다
    # (mvp_integration_nodes:cctv_merge_main). 확인할 것은 "cctv_merge 라는
    # 실행파일이 등록돼 있는가"이지 어느 모듈을 거치는가가 아니다.
    assert re.search(r"'cctv_merge = cooperative_parking_robot\.[\w.]+:\w+'",
                     setup_text)
