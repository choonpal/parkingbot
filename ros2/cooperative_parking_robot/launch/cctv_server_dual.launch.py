#!/usr/bin/env python3
"""천장 CCTV 2대(기본 /dev/video0, /dev/video2) Jetson 서버 launch.

구조
----
    camera 0 ──> opencv_camera(cam0) ──> /cctv0/image_raw
                    └─> cctv_rectify(cam0, calib0) ──> /cctv0/image_rect
                            ├─> yolo_bev_map(cam0, sensor 모드) ──> /cctv0/detections
                            └────────────────┐
    camera 2 ──> opencv_camera(cam2) ──> /cctv2/image_raw
                    └─> cctv_rectify(cam2, calib2) ──> /cctv2/image_rect
                            ├─> yolo_bev_map(cam2, sensor 모드) ──> /cctv2/detections
                            └────────────────┤
                                             v
                                    cctv_merge_node
                                      /parking/map
                                      /parking/empty_slots
                                      /parking/target_pose
                                      /parking/vehicle_spec
                                      /parking/vehicle_pose_feedback
                                      /parking/target_ready
                                             v
                                    fleet_manager_node (변경 없음)

    cctv_robot_marker_node 는 **한 개만** 띄우고 두 영상을 모두 구독한다.
    /front/cctv_pose publisher가 둘이 되면 pose_fusion EKF가 같은 정보로
    두 번 correct하기 때문이다(자세한 이유는 노드 docstring 참조).

단일 카메라로 되돌리려면 기존 cctv_server.launch.py를 그대로 쓰면 된다.
이 파일은 기존 launch를 건드리지 않는 별도 파일이다.

필요 파일 (자세한 생성 절차는 docs/DUAL_CCTV_MERGE_20260812.md)
  config/cctv0_camera_calibration.npz          카메라0 내부 파라미터
  config/cctv2_camera_calibration.npz          카메라2 내부 파라미터
  ~/.ros/adaptive_valet_bot/homography_cam0_rectified.npy
  ~/.ros/adaptive_valet_bot/homography_cam2_rectified.npy
  ~/.ros/adaptive_valet_bot/parking_layout.yaml   (두 카메라 슬롯이 합쳐진 하나)
"""

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution,
    PythonExpression, TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _float(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _int(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def _bool(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def _float_array(name):
    """'0.0, 1.2, ...' 형태의 launch 인자를 double[] 파라미터로 넘긴다."""
    return ParameterValue(
        LaunchConfiguration(name), value_type=List[float])


def _int_array(name):
    """'[2, 3, 5, 7]' 형태의 launch 인자를 integer[] 파라미터로 넘긴다."""
    return ParameterValue(
        LaunchConfiguration(name), value_type=List[int])


def generate_launch_description():
    enable_cameras = LaunchConfiguration('enable_opencv_camera')
    enable_markers = LaunchConfiguration('enable_cctv_robot_markers')
    enable_web = PythonExpression([
        "'", LaunchConfiguration('enable_operator_ui'),
        "'.lower() in ('true', '1', 'yes', 'on') or '",
        LaunchConfiguration('enable_debug_overlay'),
        "'.lower() in ('true', '1', 'yes', 'on')",
    ])

    runtime_config_dir = PathJoinSubstitution([
        EnvironmentVariable('HOME'), '.ros', 'adaptive_valet_bot'])
    default_vehicle_model = PathJoinSubstitution([
        FindPackageShare('cooperative_parking_robot'),
        'models', 'parking_vehicle_yolo11n_seg.pt'])
    default_calib0 = PathJoinSubstitution([
        runtime_config_dir, 'cctv0_camera_calibration.npz'])
    default_calib2 = PathJoinSubstitution([
        runtime_config_dir, 'cctv2_camera_calibration.npz'])
    default_h0 = PathJoinSubstitution([
        runtime_config_dir, 'homography_cam0_rectified.npy'])
    default_h2 = PathJoinSubstitution([
        runtime_config_dir, 'homography_cam2_rectified.npy'])
    default_layout = PathJoinSubstitution([
        runtime_config_dir, 'parking_layout.yaml'])
    default_registry_database = PathJoinSubstitution([
        runtime_config_dir, 'parking_registry.db'])

    common_vision_params = {
        'model_path': LaunchConfiguration('model_path'),
        'model_mode': LaunchConfiguration('model_mode'),
        'inference_imgsz': _int('inference_imgsz'),
        'process_every_n': _int('process_every_n'),
        'confidence': _float('confidence'),
        'yaw_pca_min_ratio': _float('yaw_pca_min_ratio'),
        'yaw_ema_alpha': _float('yaw_ema_alpha'),
        'yaw_limit_deg': _float('yaw_limit_deg'),
        'classifier_path': LaunchConfiguration('classifier_path'),
        'coco_vehicle_class_ids': _int_array('coco_vehicle_class_ids'),
        'homography_scale_to_m': _float('homography_scale_to_m'),
        'coverage_margin_px': _float('coverage_margin_px'),
        'require_dependencies': True,
        'require_homography': True,
        'require_registered_layout': True,
        'use_fixed_wheelbase': True,
        'fixed_wheelbase_m': _float('fixed_wheelbase_m'),
        # sensor 모드: /parking/*는 병합 노드만 발행한다.
        'publish_detections': True,
        'publish_mission_outputs': False,
    }

    return LaunchDescription([
        # ============================================================
        # 카메라
        # ============================================================
        DeclareLaunchArgument(
            'enable_opencv_camera', default_value='false',
            description='true면 이 패키지가 두 카메라를 cv2로 단독 점유'),
        DeclareLaunchArgument(
            'camera0_id', default_value='0',
            description='/dev/video0'),
        DeclareLaunchArgument(
            'camera2_id', default_value='2',
            description='/dev/video2'),
        DeclareLaunchArgument('camera_width_px', default_value='640'),
        DeclareLaunchArgument('camera_height_px', default_value='480'),
        DeclareLaunchArgument('camera_fps', default_value='30.0'),
        DeclareLaunchArgument('camera0_gstreamer_pipeline', default_value=''),
        DeclareLaunchArgument('camera2_gstreamer_pipeline', default_value=''),
        DeclareLaunchArgument('cctv0_raw_topic', default_value='/cctv0/image_raw'),
        DeclareLaunchArgument('cctv0_rect_topic', default_value='/cctv0/image_rect'),
        DeclareLaunchArgument('cctv2_raw_topic', default_value='/cctv2/image_raw'),
        DeclareLaunchArgument('cctv2_rect_topic', default_value='/cctv2/image_rect'),
        DeclareLaunchArgument('cctv0_camera_calib', default_value=default_calib0),
        DeclareLaunchArgument('cctv2_camera_calib', default_value=default_calib2),
        DeclareLaunchArgument('calibration_width_px', default_value='640'),
        DeclareLaunchArgument('calibration_height_px', default_value='480'),

        # ============================================================
        # YOLO / BEV
        # ============================================================
        DeclareLaunchArgument('model_path', default_value=default_vehicle_model),
        DeclareLaunchArgument('model_mode', default_value='vehicle_seg'),
        DeclareLaunchArgument('inference_imgsz', default_value='640'),
        DeclareLaunchArgument(
            'process_every_n', default_value='3',
            description='카메라 2대분 추론이 동시에 돌므로 1대일 때보다 크게 둔다'),
        DeclareLaunchArgument('confidence', default_value='0.4'),
        DeclareLaunchArgument('yaw_pca_min_ratio', default_value='1.25'),
        DeclareLaunchArgument('yaw_ema_alpha', default_value='0.15'),
        DeclareLaunchArgument('yaw_limit_deg', default_value='90.0'),
        DeclareLaunchArgument('classifier_path', default_value=''),
        DeclareLaunchArgument(
            'coco_vehicle_class_ids', default_value='[2, 3, 5, 7]',
            description='COCO class IDs; camera pipeline test uses [0]'),
        DeclareLaunchArgument('homography_cam0_file', default_value=default_h0),
        DeclareLaunchArgument('homography_cam2_file', default_value=default_h2),
        DeclareLaunchArgument(
            'homography_scale_to_m', default_value='1.0',
            description='브라우저 등록 H는 pixel->metre이므로 1.0'),
        DeclareLaunchArgument(
            'coverage_margin_px', default_value='8.0',
            description='영상 테두리를 잘라 coverage polygon을 만들 여유 픽셀'),
        DeclareLaunchArgument('fixed_wheelbase_m', default_value='0.70'),
        DeclareLaunchArgument(
            'layout_config', default_value=default_layout,
            description='두 카메라의 슬롯이 합쳐진 하나의 등록 파일'),
        DeclareLaunchArgument(
            'parking_registry_db_path',
            default_value=default_registry_database,
            description='Fleet Parking Registry SQLite 파일'),
        DeclareLaunchArgument(
            'simultaneous_entry', default_value='false',
            description='Fleet retrieve preflight entry timing policy'),
        DeclareLaunchArgument(
            'planning_validation_mode', default_value='warn_only',
            description='MVP model-based planning checks: enforce or warn_only'),

        # 카메라별 광축 지상점/높이 — parallax 보정 (실측 전엔 0.0)
        DeclareLaunchArgument('cam0_ground_x_m', default_value='0.0'),
        DeclareLaunchArgument('cam0_ground_y_m', default_value='0.0'),
        DeclareLaunchArgument('cam0_height_m', default_value='0.0'),
        DeclareLaunchArgument('cam2_ground_x_m', default_value='0.0'),
        DeclareLaunchArgument('cam2_ground_y_m', default_value='0.0'),
        DeclareLaunchArgument('cam2_height_m', default_value='0.0'),
        DeclareLaunchArgument('vehicle_detection_height_m', default_value='0.0'),
        # 상판 마커 노드가 카메라를 고를 때 쓰는 광축 지상점 [x0,y0, x2,y2].
        # 위 cam*_ground_*와 같은 값을 넣는다(중복이지만 배열 타입이 필요).
        DeclareLaunchArgument(
            'camera_ground_points', default_value='[0.0, 0.0, 0.0, 0.0]',
            description='[cam0_x, cam0_y, cam2_x, cam2_y] m'),

        # ============================================================
        # 병합
        # ============================================================
        DeclareLaunchArgument('merge_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('camera_timeout_s', default_value='1.0'),
        DeclareLaunchArgument(
            'require_all_cameras', default_value='false',
            description='true면 한 대라도 죽으면 /parking/* 발행을 멈춘다'),
        DeclareLaunchArgument(
            'duplicate_center_gate_m', default_value='0.35',
            description='두 카메라가 본 같은 차량으로 볼 최대 중심 거리'),
        DeclareLaunchArgument('duplicate_overlap_ratio', default_value='0.30'),
        DeclareLaunchArgument('duplicate_center_blend', default_value='0.0'),
        DeclareLaunchArgument(
            'require_full_slot_coverage', default_value='false',
            description='true면 슬롯 네 모서리가 한 카메라에 다 들어와야 판정'),

        # ============================================================
        # 상판 마커 (노드 1개가 두 영상 구독)
        # ============================================================
        DeclareLaunchArgument('enable_cctv_robot_markers', default_value='true'),
        DeclareLaunchArgument('aruco_dict', default_value='DICT_4X4_50'),
        DeclareLaunchArgument('front_marker_id', default_value='10'),
        DeclareLaunchArgument('rear_marker_id', default_value='11'),
        DeclareLaunchArgument('min_marker_area_px', default_value='100.0'),
        DeclareLaunchArgument('min_marker_area_ratio', default_value='0.0003'),
        DeclareLaunchArgument('marker_size_m', default_value='0.175'),
        DeclareLaunchArgument('front_yaw_offset_deg', default_value='0.0'),
        DeclareLaunchArgument('rear_yaw_offset_deg', default_value='0.0'),
        DeclareLaunchArgument('front_marker_offset_x_m', default_value='0.0'),
        DeclareLaunchArgument('rear_marker_offset_x_m', default_value='0.0'),
        DeclareLaunchArgument('front_marker_height_m', default_value='0.0'),
        DeclareLaunchArgument('rear_marker_height_m', default_value='0.0'),
        DeclareLaunchArgument('selection_hold_s', default_value='0.30'),
        DeclareLaunchArgument('observation_timeout_s', default_value='0.30'),

        # ============================================================
        # UI / 진단
        # ============================================================
        DeclareLaunchArgument('require_ui_confirmation', default_value='true'),
        DeclareLaunchArgument('ui_request_timeout_s', default_value='10.0'),
        DeclareLaunchArgument('enable_operator_ui', default_value='true'),
        DeclareLaunchArgument('ui_status_stale_s', default_value='3.0'),
        DeclareLaunchArgument('ui_button_cooldown_s', default_value='2.0'),
        DeclareLaunchArgument(
            'enable_debug_overlay', default_value='false',
            description='진단 ArUco/FPS overlay 및 annotated topic 활성'),
        DeclareLaunchArgument(
            'debug_image_topic', default_value='/cctv0/image_rect',
            description='kiosk/진단 화면에 표시할 카메라'),
        DeclareLaunchArgument('debug_enable_aruco', default_value='true'),
        DeclareLaunchArgument('debug_web_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('debug_web_port', default_value='5000'),
        DeclareLaunchArgument('debug_jpeg_quality', default_value='70'),

        # ============================================================
        # 노드 — 카메라 0
        # ============================================================
        Node(
            package='cooperative_parking_robot',
            executable='opencv_camera',
            name='opencv_camera_node_cam0',
            condition=IfCondition(enable_cameras),
            parameters=[{
                'camera_id': _int('camera0_id'),
                'gstreamer_pipeline': LaunchConfiguration(
                    'camera0_gstreamer_pipeline'),
                'output_topic': LaunchConfiguration('cctv0_raw_topic'),
                'width': _int('camera_width_px'),
                'height': _int('camera_height_px'),
                'fps': _float('camera_fps'),
                'buffer_size': 1,
                'require_camera': True,
            }],
            output='screen'),

        Node(
            package='cooperative_parking_robot',
            executable='cctv_rectify',
            name='cctv_rectify_node_cam0',
            parameters=[{
                'input_topic': LaunchConfiguration('cctv0_raw_topic'),
                'output_topic': LaunchConfiguration('cctv0_rect_topic'),
                'camera_calib': LaunchConfiguration('cctv0_camera_calib'),
                'calibration_width_px': _int('calibration_width_px'),
                'calibration_height_px': _int('calibration_height_px'),
            }],
            output='screen'),

        Node(
            package='cooperative_parking_robot',
            executable='yolo_bev_map',
            name='yolo_bev_map_node_cam0',
            parameters=[LaunchConfiguration('layout_config'),
                        dict(common_vision_params, **{
                            'camera_id': 'cam0',
                            'detection_topic': '/cctv0/detections',
                            'image_topic': LaunchConfiguration('cctv0_rect_topic'),
                            'homography_file': LaunchConfiguration(
                                'homography_cam0_file'),
                            'camera_ground_x_m': _float('cam0_ground_x_m'),
                            'camera_ground_y_m': _float('cam0_ground_y_m'),
                            'camera_height_m': _float('cam0_height_m'),
                            'vehicle_detection_height_m': _float(
                                'vehicle_detection_height_m'),
                        })],
            output='screen'),

        # ============================================================
        # 노드 — 카메라 2
        # ============================================================
        Node(
            package='cooperative_parking_robot',
            executable='opencv_camera',
            name='opencv_camera_node_cam2',
            condition=IfCondition(enable_cameras),
            parameters=[{
                'camera_id': _int('camera2_id'),
                'gstreamer_pipeline': LaunchConfiguration(
                    'camera2_gstreamer_pipeline'),
                'output_topic': LaunchConfiguration('cctv2_raw_topic'),
                'width': _int('camera_width_px'),
                'height': _int('camera_height_px'),
                'fps': _float('camera_fps'),
                'buffer_size': 1,
                'require_camera': True,
            }],
            output='screen'),

        Node(
            package='cooperative_parking_robot',
            executable='cctv_rectify',
            name='cctv_rectify_node_cam2',
            parameters=[{
                'input_topic': LaunchConfiguration('cctv2_raw_topic'),
                'output_topic': LaunchConfiguration('cctv2_rect_topic'),
                'camera_calib': LaunchConfiguration('cctv2_camera_calib'),
                'calibration_width_px': _int('calibration_width_px'),
                'calibration_height_px': _int('calibration_height_px'),
            }],
            output='screen'),

        Node(
            package='cooperative_parking_robot',
            executable='yolo_bev_map',
            name='yolo_bev_map_node_cam2',
            parameters=[LaunchConfiguration('layout_config'),
                        dict(common_vision_params, **{
                            'camera_id': 'cam2',
                            'detection_topic': '/cctv2/detections',
                            'image_topic': LaunchConfiguration('cctv2_rect_topic'),
                            'homography_file': LaunchConfiguration(
                                'homography_cam2_file'),
                            'camera_ground_x_m': _float('cam2_ground_x_m'),
                            'camera_ground_y_m': _float('cam2_ground_y_m'),
                            'camera_height_m': _float('cam2_height_m'),
                            'vehicle_detection_height_m': _float(
                                'vehicle_detection_height_m'),
                        })],
            output='screen'),

        # ============================================================
        # 병합 노드 — /parking/* 최종 발행자
        # ============================================================
        Node(
            package='cooperative_parking_robot',
            executable='cctv_merge',
            name='cctv_merge_node',
            parameters=[LaunchConfiguration('layout_config'), {
                'detection_topics': ['/cctv0/detections', '/cctv2/detections'],
                'camera_ids': ['cam0', 'cam2'],
                'require_registered_layout': True,
                'merge_rate_hz': _float('merge_rate_hz'),
                'camera_timeout_s': _float('camera_timeout_s'),
                'require_all_cameras': _bool('require_all_cameras'),
                'duplicate_center_gate_m': _float('duplicate_center_gate_m'),
                'duplicate_overlap_ratio': _float('duplicate_overlap_ratio'),
                'duplicate_center_blend': _float('duplicate_center_blend'),
                'require_full_slot_coverage': _bool(
                    'require_full_slot_coverage'),
                'use_fixed_wheelbase': True,
                'fixed_wheelbase_m': _float('fixed_wheelbase_m'),
            }],
            output='screen'),

        # ============================================================
        # Fleet Manager — 단일 카메라 구성과 완전히 동일
        # ============================================================
        Node(
            package='cooperative_parking_robot',
            executable='fleet_manager',
            name='fleet_manager_node',
            parameters=[LaunchConfiguration('layout_config'), {
                'require_registered_layout': True,
                'require_ui_confirmation': _bool('require_ui_confirmation'),
                'ui_request_timeout_s': _float('ui_request_timeout_s'),
                'simultaneous_entry': _bool('simultaneous_entry'),
                'planning_validation_mode': LaunchConfiguration(
                    'planning_validation_mode'),
                'parking_registry_db_path': LaunchConfiguration(
                    'parking_registry_db_path'),
            }],
            output='screen'),

        # ============================================================
        # 상판 마커 — 노드 1개가 두 영상을 구독
        # ============================================================
        Node(
            package='cooperative_parking_robot',
            executable='cctv_robot_marker',
            name='cctv_robot_marker_node',
            condition=IfCondition(enable_markers),
            parameters=[{
                # 배열 대신 쉼표 구분 문자열을 쓰는 이유: launch의
                # PathJoinSubstitution 결과는 "문자열 하나"로만 전달되므로
                # 리스트 안에 넣으면 두 경로가 하나로 이어붙는다.
                'image_topics_csv': [
                    LaunchConfiguration('cctv0_rect_topic'),
                    TextSubstitution(text=','),
                    LaunchConfiguration('cctv2_rect_topic'),
                ],
                'homography_files_csv': [
                    LaunchConfiguration('homography_cam0_file'),
                    TextSubstitution(text=','),
                    LaunchConfiguration('homography_cam2_file'),
                ],
                'camera_ids_csv': 'cam0,cam2',
                'camera_ground_points': _float_array('camera_ground_points'),
                'selection_hold_s': _float('selection_hold_s'),
                'observation_timeout_s': _float('observation_timeout_s'),
                'zero_stamp_fallback_to_now': True,
                'homography_scale_to_m': _float('homography_scale_to_m'),
                'aruco_dict': LaunchConfiguration('aruco_dict'),
                'front_marker_id': _int('front_marker_id'),
                'rear_marker_id': _int('rear_marker_id'),
                'min_marker_area_px': _float('min_marker_area_px'),
                'min_marker_area_ratio': _float('min_marker_area_ratio'),
                'front_yaw_offset_deg': _float('front_yaw_offset_deg'),
                'rear_yaw_offset_deg': _float('rear_yaw_offset_deg'),
                'front_marker_offset_x_m': _float('front_marker_offset_x_m'),
                'rear_marker_offset_x_m': _float('rear_marker_offset_x_m'),
                # camera_height_m은 두 카메라 설치 높이가 같다는 전제다.
                # 다르면 노드를 카메라별 높이 배열로 확장해야 한다(문서 §9 참조).
                'camera_height_m': _float('cam0_height_m'),
                'front_marker_height_m': _float('front_marker_height_m'),
                'rear_marker_height_m': _float('rear_marker_height_m'),
            }],
            output='screen'),

        # ============================================================
        # 진단 웹 / kiosk
        # ============================================================
        Node(
            package='cooperative_parking_robot',
            executable='jetson_vision_web',
            name='jetson_vision_web_node',
            condition=IfCondition(enable_web),
            parameters=[LaunchConfiguration('layout_config'), {
                'image_topic': LaunchConfiguration('debug_image_topic'),
                'enable_aruco': _bool('debug_enable_aruco'),
                'aruco_dict': LaunchConfiguration('aruco_dict'),
                'front_marker_id': _int('front_marker_id'),
                'rear_marker_id': _int('rear_marker_id'),
                'marker_size_m': _float('marker_size_m'),
                'min_marker_area_px': _float('min_marker_area_px'),
                'min_marker_area_ratio': _float('min_marker_area_ratio'),
                'camera_calib': LaunchConfiguration('cctv0_camera_calib'),
                'calibration_width_px': _int('calibration_width_px'),
                'calibration_height_px': _int('calibration_height_px'),
                'jpeg_quality': _int('debug_jpeg_quality'),
                'web_host': LaunchConfiguration('debug_web_host'),
                'web_port': _int('debug_web_port'),
                'enable_operator_ui': _bool('enable_operator_ui'),
                'enable_debug_overlay': _bool('enable_debug_overlay'),
                'status_stale_s': _float('ui_status_stale_s'),
                'ui_button_cooldown_s': _float('ui_button_cooldown_s'),
            }],
            output='screen'),
    ])
