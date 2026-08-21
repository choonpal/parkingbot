"""Static and ROS-independent regression tests for Jetson vision integration."""

from pathlib import Path
import re

import numpy as np
import pytest

from cooperative_parking_robot.vision_utils import (
    correct_floor_projection,
    normalize_model_mode,
    pnp_distance_m,
    select_marker_by_id,
)


ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'cooperative_parking_robot'


def test_model_mode_is_explicit_and_rejects_unknown_values():
    assert normalize_model_mode('coco') == 'coco'
    assert normalize_model_mode('generic') == 'coco'
    assert normalize_model_mode('vehicle_seg') == 'vehicle_seg'
    assert normalize_model_mode('parking_seg') == 'parking_seg'
    with pytest.raises(ValueError):
        normalize_model_mode('guess_from_filename')


def test_marker_filter_rejects_tiny_false_positive_and_selects_largest():
    tiny = np.array([[[0, 0], [10, 0], [10, 10], [0, 10]]], dtype=float)
    medium = np.array([[[0, 0], [40, 0], [40, 40], [0, 40]]], dtype=float)
    large = np.array([[[0, 0], [60, 0], [60, 60], [0, 60]]], dtype=float)
    corners = [tiny, medium, large]
    ids = np.array([[10], [10], [10]])
    selected, area = select_marker_by_id(
        corners, ids, 10, min_area_px=1000.0,
        frame_width=1280, frame_height=720)
    assert np.array_equal(np.asarray(selected), large[0])
    assert area == pytest.approx(3600.0)


def test_marker_filter_can_use_resolution_relative_threshold():
    marker = np.array([[[0, 0], [30, 0], [30, 30], [0, 30]]], dtype=float)
    ids = np.array([[11]])
    selected, _ = select_marker_by_id(
        [marker], ids, 11, min_area_ratio=0.002,
        frame_width=1280, frame_height=720)
    assert selected is None


def test_pnp_distance_uses_euclidean_translation_not_only_z():
    assert pnp_distance_m(np.array([[3.0], [4.0], [12.0]])) == pytest.approx(13.0)


def test_only_camera_publisher_node_owns_video_capture():
    camera = (PKG / 'opencv_camera_node.py').read_text()
    web = (PKG / 'jetson_vision_web_node.py').read_text()
    yolo = (PKG / 'yolo_bev_map_node.py').read_text()
    marker = (PKG / 'cctv_robot_marker_node.py').read_text()
    assert 'cv2.VideoCapture' in camera
    assert 'cv2.VideoCapture(' not in web
    assert 'cv2.VideoCapture(' not in yolo
    assert 'cv2.VideoCapture(' not in marker


def test_generic_coco_model_is_not_interpreted_as_parking_classes():
    source = (PKG / 'yolo_bev_map_node.py').read_text()
    assert "declare_parameter('model_mode', 'coco')" in source
    assert "declare_parameter('coco_vehicle_class_ids', [2, 3, 5, 7])" in source
    assert "self.model_mode in ('vehicle_seg', 'parking_seg')" in source
    assert "if self.model_mode == 'vehicle_seg':" in source
    assert 'polygon_overlap_ratio' in source
    assert 'if not os.path.isfile(mp):' in source
    assert 'requires a local model file' in source
    assert 'if cls not in self.coco_vehicle_ids' in source
    assert 'custom_model' not in source
    assert 'use_pretrained_fallback' not in source


def test_cctv_launch_exposes_camera_model_marker_and_web_controls():
    source = (ROOT / 'launch/cctv_server.launch.py').read_text()
    for token in (
            "'enable_opencv_camera'", "'model_mode'",
            "'homography_scale_to_m'",
            "'min_marker_area_px'", "'enable_operator_ui'",
            "'enable_debug_overlay'",
            "executable='opencv_camera'", "executable='jetson_vision_web'"):
        assert token in source
    assert 'allow_model_download' not in source
    assert "default_value='vehicle_seg'" in source
    assert "'models', 'parking_vehicle_yolo11n_seg.pt'" in source
    assert "default_value='false'" in source  # optional camera/web are opt-in


def test_dual_cctv_launch_wires_configurable_coco_vehicle_classes():
    source = (ROOT / 'launch/cctv_server_dual.launch.py').read_text()
    assert 'def _int_array(name):' in source
    assert "'coco_vehicle_class_ids': _int_array(" in source
    assert "'coco_vehicle_class_ids')" in source
    assert re.search(
        r"'coco_vehicle_class_ids',\s*default_value='\[2, 3, 5, 7\]'",
        source)


def test_dual_cctv_uses_runtime_per_camera_calibration_paths():
    source = (ROOT / 'launch/cctv_server_dual.launch.py').read_text()
    assert "runtime_config_dir, 'cctv0_camera_calibration.npz'" in source
    assert "runtime_config_dir, 'cctv2_camera_calibration.npz'" in source
    assert "share, 'config', 'cctv0_camera_calibration.npz'" not in source
    assert "share, 'config', 'cctv2_camera_calibration.npz'" not in source


def test_dual_cctv_provisional_camera_and_calibration_resolution_is_640x480():
    source = (ROOT / 'launch/cctv_server_dual.launch.py').read_text()
    assert "'camera_width_px', default_value='640'" in source
    assert "'camera_height_px', default_value='480'" in source
    assert "'calibration_width_px', default_value='640'" in source
    assert "'calibration_height_px', default_value='480'" in source


def test_operator_ui_launch_is_independent_from_debug_overlay():
    launch_dir = ROOT / 'launch'
    for name in (
            'cctv_server.launch.py',
            'cctv_server_dual.launch.py',
            'full_system.launch.py'):
        source = (launch_dir / name).read_text()
        assert re.search(
            r"'enable_operator_ui',\s*default_value='true'", source)
        assert re.search(
            r"'enable_debug_overlay',\s*default_value='false'", source)
        assert "'enable_debug_web'" not in source
        assert 'PythonExpression' in source
        assert "'enable_operator_ui': _bool('enable_operator_ui')" in source
        assert "'enable_debug_overlay': _bool('enable_debug_overlay')" in source


def test_web_node_keeps_operator_ui_without_duplicate_yolo_inference():
    source = (PKG / 'jetson_vision_web_node.py').read_text()
    assert "declare_parameter('enable_debug_overlay', False)" in source
    assert 'self.enable_debug_overlay = bool(' in source
    assert 'self.enable_debug_overlay and requested_enable_aruco' in source
    assert 'if self.enable_debug_overlay:' in source
    assert 'if self.annotated_publisher is not None:' in source
    assert 'from ultralytics import YOLO' not in source
    assert 'def _run_yolo' not in source
    assert 'def _draw_yolo' not in source


def test_web_monitor_is_diagnostic_not_a_mission_output_publisher():
    source = (PKG / 'jetson_vision_web_node.py').read_text()
    assert '/cctv/debug/annotated' in source
    for mission_topic in (
            '/parking/map', '/parking/target_pose', '/parking/empty_slots',
            '/front/cctv_pose', '/rear/cctv_pose'):
        assert mission_topic not in source


def test_web_worker_waits_for_first_real_frame():
    source = (PKG / 'jetson_vision_web_node.py').read_text()
    assert 'self._latest_frame is not None' in source
    assert 'self._input_sequence != self._processed_sequence' in source


def test_operator_ui_submits_authenticated_vehicle_requests_and_waits_for_fleet():
    source = (PKG / 'jetson_vision_web_node.py').read_text()
    assert "@app.route('/api/retrieve', methods=['POST'])" in source
    assert "'type': 'retrieve'" in source
    assert "'vehicle_number': vehicle_number" in source
    assert "'password': password" in source
    assert "'destination_slot_id': destination_slot_id" in source
    assert "body.get('password', '')" in source
    assert "'source_slot_id': selected['slot_id']" not in source
    assert "'client_id': self._ui_client_id" in source
    assert "'submitted': submitted" in source
    assert "fleet.get('request_status')" in source
    assert "fleet.get('last_completed')" in source
    assert 'completion_sequence' in source
    assert 'lastCompletion=null' in source
    assert 'lastCompletion===null' in source


def test_web_uses_only_safe_registry_summary_for_slot_buttons():
    source = (PKG / 'jetson_vision_web_node.py').read_text()
    assert "fleet.get('parking_slots', [])" in source
    assert "slot['retrieve_enabled']" in source
    assert "value.get('retrievable', False)" in source
    assert "value.get('final_vehicle_pose'" not in source
    assert "value.get('vehicle_spec'" not in source


def test_yolo_and_marker_share_homography_unit_scale_parameter():
    yolo = (PKG / 'yolo_bev_map_node.py').read_text()
    marker = (PKG / 'cctv_robot_marker_node.py').read_text()
    launch = (ROOT / 'launch/cctv_server.launch.py').read_text()
    for source in (yolo, marker, launch):
        assert 'homography_scale_to_m' in source


def test_parking_layout_is_externalized_from_python_source():
    import yaml
    layout_path = ROOT / 'config/parking_layout.yaml'
    layout = yaml.safe_load(layout_path.read_text())
    yolo_params = layout['/**']['ros__parameters']
    fleet_params = layout['fleet_manager_node']['ros__parameters']
    assert 'yolo_bev_map_node' not in layout
    assert yolo_params['layout_registered'] is False
    assert fleet_params['layout_registered'] is False
    assert len(yolo_params['waiting_zone']) == 4
    assert len(yolo_params['slot_coords']) >= 2
    assert len(yolo_params['slot_coords']) % 2 == 0
    assert yolo_params['map_resolution'] == fleet_params['map_resolution']
    launch = (ROOT / 'launch/cctv_server.launch.py').read_text()
    assert "'layout_config'" in launch
    assert "EnvironmentVariable('HOME')" in launch
    assert "'.ros', 'adaptive_valet_bot'" in launch
    assert "'require_registered_layout': True" in launch


def test_floor_homography_parallax_correction_moves_toward_optical_axis():
    corrected = correct_floor_projection(
        floor_x=2.0, floor_y=1.0,
        camera_ground_x=0.0, camera_ground_y=0.0,
        camera_height=2.0, object_height=0.2)
    assert corrected == pytest.approx((1.8, 0.9))
    assert correct_floor_projection(2.0, 1.0, 0.0, 0.0, 0.0, 0.0) == (2.0, 1.0)
    with pytest.raises(ValueError):
        correct_floor_projection(1.0, 1.0, 0.0, 0.0, 0.2, 0.2)


def test_real_robot_launches_default_to_front_first_entry():
    launch_dir = ROOT / 'launch'
    for name in (
            'front_robot.launch.py',
            'rear_robot.launch.py',
            'full_system.launch.py',
            'cctv_server.launch.py',
            'cctv_server_dual.launch.py'):
        source = (launch_dir / name).read_text()
        assert re.search(
            r'DeclareLaunchArgument\(\s*[' + chr(34) + chr(39) +
            r']simultaneous_entry[' + chr(34) + chr(39) +
            r'],\s*default_value=[' + chr(34) + chr(39) +
            r']false[' + chr(34) + chr(39) + r']',
            source)
        assert source.count(
            chr(39) + 'simultaneous_entry' + chr(39)) + source.count(
            chr(34) + 'simultaneous_entry' + chr(34)) >= 2

    layout = (ROOT / 'config/parking_layout.yaml').read_text()
    assert 'simultaneous_entry: false' in layout
