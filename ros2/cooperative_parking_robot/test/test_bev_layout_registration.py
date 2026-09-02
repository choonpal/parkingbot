"""브라우저형 BEV/주차면 등록과 실제 주차 연결 회귀 테스트."""

from pathlib import Path
import re

import numpy as np
import pytest
import yaml

from cooperative_parking_robot.bev_layout_core import (
    homography_reprojection_errors,
    load_layout_yaml,
    render_parking_layout_yaml,
    transform_points,
    validate_reference_pairs,
)
from cooperative_parking_robot.parking_geometry import build_slot


ROOT = Path(__file__).resolve().parents[1]


def test_checked_in_site_layout_matches_measured_map_and_zones():
    parsed = yaml.safe_load(
        (ROOT / 'config' / 'parking_layout.yaml').read_text(encoding='utf-8'))
    vision = parsed['/**']['ros__parameters']
    fleet = parsed['fleet_manager_node']['ros__parameters']
    merge = parsed['cctv_merge_node']['ros__parameters']

    assert vision['map_origin_x_m'] == pytest.approx(-0.40)
    assert vision['map_origin_y_m'] == pytest.approx(-0.80)
    assert vision['map_width_m'] == pytest.approx(4.80)
    assert vision['map_height_m'] == pytest.approx(4.63)
    assert merge['map_origin_x_m'] == pytest.approx(-0.40)
    assert merge['map_origin_y_m'] == pytest.approx(-0.80)
    assert merge['map_width_m'] == pytest.approx(4.80)
    assert merge['map_height_m'] == pytest.approx(4.63)
    assert vision['waiting_polygon'] == pytest.approx(
        [0.0, 0.0, 1.2, 0.0, 1.2, 0.8, 0.0, 0.8])
    assert vision['robot_start_polygon'] == pytest.approx(
        [3.2, 0.0, 4.0, 0.0, 4.0, 0.8, 3.2, 0.8])
    assert vision['front_start_pose'] == pytest.approx([3.6, 0.6, 180.0])
    assert vision['rear_start_pose'] == pytest.approx([3.6, 0.2, 180.0])
    assert vision['waiting_yaw_deg'] == pytest.approx(180.0)
    assert 'waiting_yaw_deg' not in fleet
    assert vision['slot_ids'] == ['P1', 'P2', 'P3', 'P4']
    assert vision['slot_coords'] == pytest.approx(
        [1.2, 2.2, 2.0, 2.2, 2.8, 2.2, 3.6, 2.2])


def test_pixel_to_metre_homography_and_reprojection_error():
    # 100px가 1m인 단순 H. 새 도구의 출력 단위는 cm가 아니라 metre다.
    matrix = np.array([
        [0.01, 0.0, 0.0],
        [0.0, 0.01, 0.0],
        [0.0, 0.0, 1.0],
    ])
    references = [
        {'pixel': [0, 0], 'world': [0, 0]},
        {'pixel': [200, 0], 'world': [2, 0]},
        {'pixel': [200, 100], 'world': [2, 1]},
        {'pixel': [0, 100], 'world': [0, 1]},
    ]

    assert transform_points(matrix, [(150, 50)]) == pytest.approx([(1.5, 0.5)])
    errors, rms, maximum = homography_reprojection_errors(matrix, references)
    assert errors == pytest.approx([0.0, 0.0, 0.0, 0.0])
    assert rms == pytest.approx(0.0)
    assert maximum == pytest.approx(0.0)


def test_reference_validation_rejects_duplicate_and_short_input():
    base = [
        {'pixel': [0, 0], 'world': [0, 0]},
        {'pixel': [1, 0], 'world': [1, 0]},
        {'pixel': [1, 1], 'world': [1, 1]},
        {'pixel': [0, 1], 'world': [0, 1]},
    ]
    assert len(validate_reference_pairs(base)) == 4
    with pytest.raises(ValueError, match='at least four'):
        validate_reference_pairs(base[:3])
    duplicate = list(base)
    duplicate[-1] = {'pixel': [0, 0], 'world': [0, 1]}
    with pytest.raises(ValueError, match='pixel points must be unique'):
        validate_reference_pairs(duplicate)


def test_generated_layout_is_valid_ros_parameter_yaml():
    slot = build_slot(
        'P1', center=(3.0, 2.0), size=(1.8, 0.7), yaw_deg=90.0)
    text = render_parking_layout_yaml(
        [slot],
        [(2.1, 0.3), (2.5, 0.3), (2.5, 0.9), (2.1, 0.9)],
        map_width_m=6.0,
        map_height_m=4.0,
        map_resolution_m=0.05)

    parsed = yaml.safe_load(text)
    vision = parsed['/**']['ros__parameters']
    fleet = parsed['fleet_manager_node']['ros__parameters']
    assert 'yolo_bev_map_node' not in parsed
    assert vision['layout_registered'] is True
    assert fleet['layout_registered'] is True
    assert vision['slot_ids'] == ['P1']
    assert vision['slot_coords'] == pytest.approx([3.0, 2.0])
    assert vision['slot_sizes'] == pytest.approx([1.8, 0.7])
    assert vision['slot_yaws_deg'] == pytest.approx([90.0])
    assert len(vision['slot_polygons']) == 8
    assert fleet['use_staged_slot_entry'] is True
    assert fleet['parking_direction'] == 'forward'
    assert fleet['simultaneous_entry'] is False
    assert fleet['waiting_x'] == pytest.approx(2.3)
    assert fleet['waiting_y'] == pytest.approx(0.6)
    assert vision['waiting_yaw_deg'] == pytest.approx(0.0)
    assert 'waiting_yaw_deg' not in fleet
    assert vision['car_size_m'] == pytest.approx(0.90)
    assert fleet['source_vehicle_fallback_mask_m'] == pytest.approx(
        vision['car_size_m'])
    assert parsed['cctv_merge_node']['ros__parameters'][
        'car_size_m'] == pytest.approx(vision['car_size_m'])
    assert fleet['waiting_polygon'] == pytest.approx(
        [2.1, 0.3, 2.5, 0.3, 2.5, 0.9, 2.1, 0.9])


def test_layout_loader_accepts_dual_wildcard_and_legacy_node_block(tmp_path):
    slot = build_slot(
        'P1', center=(3.0, 2.0), size=(1.8, 0.7), yaw_deg=90.0)
    rendered = render_parking_layout_yaml(
        [slot], [(2.1, 0.3), (2.5, 0.3), (2.5, 0.9), (2.1, 0.9)],
        map_width_m=6.0, map_height_m=4.0, map_resolution_m=0.05)
    wildcard_text = rendered

    wildcard_path = tmp_path / 'wildcard.yaml'
    wildcard_path.write_text(wildcard_text, encoding='utf-8')
    wildcard = load_layout_yaml(str(wildcard_path))
    assert [slot.slot_id for slot in wildcard['slots']] == ['P1']

    legacy_path = tmp_path / 'legacy.yaml'
    legacy_path.write_text(
        wildcard_text.replace('/**:', 'yolo_bev_map_node:', 1),
        encoding='utf-8')
    legacy = load_layout_yaml(str(legacy_path))
    assert [slot.slot_id for slot in legacy['slots']] == ['P1']


def test_generated_waiting_yaw_is_explicit_and_finite():
    slot = build_slot(
        'P1', center=(3.0, 2.0), size=(1.8, 0.7), yaw_deg=90.0)
    text = render_parking_layout_yaml(
        [slot], [(2.1, 0.3), (2.5, 0.3), (2.5, 0.9), (2.1, 0.9)],
        map_width_m=6.0, map_height_m=4.0, map_resolution_m=0.05,
        waiting_yaw_deg=180.0)

    parsed = yaml.safe_load(text)
    assert parsed['/**']['ros__parameters'][
        'waiting_yaw_deg'] == pytest.approx(180.0)
    with pytest.raises(ValueError, match='waiting_yaw_deg'):
        render_parking_layout_yaml(
            [slot], [(2.1, 0.3), (2.5, 0.3), (2.5, 0.9), (2.1, 0.9)],
            map_width_m=6.0, map_height_m=4.0, map_resolution_m=0.05,
            waiting_yaw_deg=float('nan'))


def test_canonical_layout_uses_negative_map_geometry_and_generated_output_is_ignored():
    canonical = yaml.safe_load(
        (ROOT / 'config' / 'parking_layout.yaml').read_text())
    canonical_params = canonical['/**']['ros__parameters']
    assert canonical_params['map_origin_x_m'] < 0.0
    assert canonical_params['map_origin_y_m'] < 0.0
    ignore = (ROOT.parent.parent / '.gitignore').read_text()
    assert '/dual_tile_homography_tool/output/' in ignore


def test_calibration_copy_preserves_runtime_layout_by_default():
    script = (ROOT.parent.parent / 'dual_tile_homography_tool' /
              'copy_results_to_project.sh').read_text()
    assert 'COPY_LAYOUT="false"' in script
    assert 'if [ "$COPY_LAYOUT" = "true" ]' in script
    assert 'parking_layout.backup_' in script


def test_layout_rejects_slot_outside_zero_origin_occupancy_grid():
    outside = build_slot(
        'P-out', center=(0.2, 2.0), size=(1.8, 0.7), yaw_deg=0.0)
    with pytest.raises(ValueError, match='inside map bounds'):
        render_parking_layout_yaml(
            [outside], [(1, 1), (2, 1), (2, 2), (1, 2)],
            map_width_m=6.0,
            map_height_m=4.0,
            map_resolution_m=0.05)


def test_browser_uses_original_image_coordinates_and_no_direct_camera_open():
    source = (
        ROOT / 'cooperative_parking_robot' /
        'bev_layout_calibrator_node.py').read_text(encoding='utf-8')
    assert re.search(r'\*\s*canvas\.width\s*/\s*rect\.width', source)
    assert re.search(r'\*\s*canvas\.height\s*/\s*rect\.height', source)
    assert 'cv2.VideoCapture' not in source
    assert "'/cctv/image_rect'" in source
    assert "web_port', 5001" in source


def test_registered_slot_yaw_reaches_staged_rigid_body_controller():
    fleet = (ROOT / 'cooperative_parking_robot' /
             'fleet_manager_node.py').read_text(encoding='utf-8')
    sync = (ROOT / 'cooperative_parking_robot' /
            'rigid_body_sync_node.py').read_text(encoding='utf-8')
    vision = (ROOT / 'cooperative_parking_robot' /
              'yolo_bev_map_node.py').read_text(encoding='utf-8')
    for token in (
            'parse_registered_slots', 'check_slot_fit',
            'make_approach_candidates', '_rotation_space_free',
            '_insertion_corridor_free'):
        assert token in fleet
    assert 'selected_approach.target_pose.yaw_rad' in fleet
    assert 'ALIGN_SLOT_YAW' in sync
    assert 'ALIGN_SLOT_CENTERLINE' in sync
    assert 'INSERT_ALONG_SLOT_AXIS' in sync
    assert 'SLOT_POSE_MISSING' in sync
    assert 'path_mission_stamp_ns' in sync
    assert 'polygon_overlap_ratio' in vision
    assert 'cv2.fillPoly(grid, [contour], 100)' in vision
    assert 'slot_empty_confirm_frames' in vision
    assert 'associate_transported_vehicle' in vision
    assert "'vehicle_length_m': self.vehicle_length" in vision


def test_calibration_launch_is_bootstrap_independent_of_yolo():
    source = (
        ROOT / 'launch' /
        'bev_layout_calibration.launch.py').read_text(encoding='utf-8')
    assert "executable='bev_layout_calibrator'" in source
    assert "executable='cctv_rectify'" in source
    assert "executable='yolo_bev_map'" not in source
    assert "executable='fleet_manager'" not in source

    # 한-PC 통합 launch에서도 패키지 예시 YAML이 아닌 현장 생성
    # YAML을 외부 경로로 전달할 수 있어야 한다.
    full = (ROOT / 'launch' / 'full_system.launch.py').read_text(
        encoding='utf-8')
    assert "DeclareLaunchArgument(\n            'layout_config'" in full
    assert "layout_config = LaunchConfiguration('layout_config')" in full
