#!/usr/bin/env python3
"""ROS 2 Humble / Jetson ceiling-camera server launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution,
    PythonExpression,
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


def generate_launch_description():
    enable_camera = LaunchConfiguration('enable_opencv_camera')
    enable_markers = LaunchConfiguration('enable_cctv_robot_markers')
    enable_web = PythonExpression([
        "'", LaunchConfiguration('enable_operator_ui'),
        "'.lower() in ('true', '1', 'yes', 'on') or '",
        LaunchConfiguration('enable_debug_overlay'),
        "'.lower() in ('true', '1', 'yes', 'on')",
    ])
    default_cctv_calib = PathJoinSubstitution([
        FindPackageShare('cooperative_parking_robot'),
        'config', 'cctv_camera_calibration.npz'])
    default_vehicle_model = PathJoinSubstitution([
        FindPackageShare('cooperative_parking_robot'),
        'models', 'parking_vehicle_yolo11n_seg.pt'])
    runtime_config_dir = PathJoinSubstitution([
        EnvironmentVariable('HOME'), '.ros', 'adaptive_valet_bot'])
    default_homography = PathJoinSubstitution([
        runtime_config_dir, 'homography_rectified.npy'])
    default_layout = PathJoinSubstitution([
        runtime_config_dir, 'parking_layout.yaml'])
    default_registry_database = PathJoinSubstitution([
        runtime_config_dir, 'parking_registry.db'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_opencv_camera', default_value='false',
            description='true면 이 패키지가 cv2.VideoCapture로 카메라를 단독 점유'),
        DeclareLaunchArgument('camera_id', default_value='0'),
        DeclareLaunchArgument('camera_width_px', default_value='1280'),
        DeclareLaunchArgument('camera_height_px', default_value='720'),
        DeclareLaunchArgument('camera_fps', default_value='30.0'),
        DeclareLaunchArgument('camera_gstreamer_pipeline', default_value=''),
        DeclareLaunchArgument('cctv_raw_topic', default_value='/cctv/image_raw'),
        DeclareLaunchArgument('cctv_rect_topic', default_value='/cctv/image_rect'),
        DeclareLaunchArgument(
            'cctv_camera_calib', default_value=default_cctv_calib),
        DeclareLaunchArgument('calibration_width_px', default_value='1280'),
        DeclareLaunchArgument('calibration_height_px', default_value='720'),

        DeclareLaunchArgument('model_path', default_value=default_vehicle_model),
        DeclareLaunchArgument('model_mode', default_value='vehicle_seg'),
        DeclareLaunchArgument('inference_imgsz', default_value='640'),
        DeclareLaunchArgument('process_every_n', default_value='3'),
        DeclareLaunchArgument('confidence', default_value='0.4'),
        DeclareLaunchArgument('yaw_pca_min_ratio', default_value='1.25'),
        DeclareLaunchArgument('yaw_ema_alpha', default_value='0.15'),
        DeclareLaunchArgument('yaw_limit_deg', default_value='90.0'),
        DeclareLaunchArgument('classifier_path', default_value=''),
        DeclareLaunchArgument(
            'homography_file', default_value=default_homography),
        DeclareLaunchArgument(
            'homography_scale_to_m', default_value='1.0',
            description='브라우저 등록 H는 pixel->metre이므로 1.0'),
        DeclareLaunchArgument('fixed_wheelbase_m', default_value='0.785'),
        DeclareLaunchArgument(
            'layout_config', default_value=default_layout,
            description='map 크기, 대기구역, 슬롯 좌표 YAML'),
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

        DeclareLaunchArgument(
            'enable_cctv_robot_markers', default_value='true'),
        DeclareLaunchArgument('aruco_dict', default_value='DICT_4X4_50'),
        DeclareLaunchArgument('front_marker_id', default_value='2'),
        DeclareLaunchArgument('rear_marker_id', default_value='1'),
        DeclareLaunchArgument('min_marker_area_px', default_value='100.0'),
        DeclareLaunchArgument('min_marker_area_ratio', default_value='0.0003'),
        DeclareLaunchArgument('marker_size_m', default_value='0.24'),
        DeclareLaunchArgument('front_yaw_offset_deg', default_value='0.0'),
        DeclareLaunchArgument('rear_yaw_offset_deg', default_value='0.0'),
        DeclareLaunchArgument('front_marker_offset_x_m', default_value='0.0'),
        DeclareLaunchArgument('rear_marker_offset_x_m', default_value='0.0'),
        DeclareLaunchArgument('camera_ground_x_m', default_value='0.0'),
        DeclareLaunchArgument('camera_ground_y_m', default_value='0.0'),
        DeclareLaunchArgument('camera_height_m', default_value='0.0'),
        DeclareLaunchArgument('front_marker_height_m', default_value='0.0'),
        DeclareLaunchArgument('rear_marker_height_m', default_value='0.0'),
        DeclareLaunchArgument('vehicle_detection_height_m', default_value='0.0'),

        DeclareLaunchArgument(
            'require_ui_confirmation', default_value='true',
            description='터치 UI 입차 버튼 승인을 요구한다. false면 v1.9처럼 자동 시작'),
        DeclareLaunchArgument('ui_request_timeout_s', default_value='10.0'),
        DeclareLaunchArgument(
            'enable_operator_ui', default_value='true',
            description='/kiosk 운용 화면과 /api/* 엔드포인트 활성'),
        DeclareLaunchArgument('ui_status_stale_s', default_value='3.0'),
        DeclareLaunchArgument('ui_button_cooldown_s', default_value='2.0'),
        DeclareLaunchArgument(
            'enable_debug_overlay', default_value='false',
            description='진단 ArUco/FPS overlay 및 annotated topic 활성'),
        DeclareLaunchArgument('debug_enable_aruco', default_value='true'),
        DeclareLaunchArgument('debug_web_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('debug_web_port', default_value='5000'),
        DeclareLaunchArgument('debug_jpeg_quality', default_value='70'),

        Node(
            package='cooperative_parking_robot',
            executable='opencv_camera',
            name='opencv_camera_node',
            condition=IfCondition(enable_camera),
            parameters=[{
                'camera_id': _int('camera_id'),
                'gstreamer_pipeline': LaunchConfiguration(
                    'camera_gstreamer_pipeline'),
                'output_topic': LaunchConfiguration('cctv_raw_topic'),
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
            name='cctv_rectify_node',
            parameters=[{
                'input_topic': LaunchConfiguration('cctv_raw_topic'),
                'output_topic': LaunchConfiguration('cctv_rect_topic'),
                'camera_calib': LaunchConfiguration('cctv_camera_calib'),
                'calibration_width_px': _int('calibration_width_px'),
                'calibration_height_px': _int('calibration_height_px'),
            }],
            output='screen'),

        Node(
            package='cooperative_parking_robot',
            executable='yolo_bev_map',
            name='yolo_bev_map_node',
            parameters=[LaunchConfiguration('layout_config'), {
                'image_topic': LaunchConfiguration('cctv_rect_topic'),
                'model_path': LaunchConfiguration('model_path'),
                'model_mode': LaunchConfiguration('model_mode'),
                'inference_imgsz': _int('inference_imgsz'),
                'process_every_n': _int('process_every_n'),
                'confidence': _float('confidence'),
                'yaw_pca_min_ratio': _float('yaw_pca_min_ratio'),
                'yaw_ema_alpha': _float('yaw_ema_alpha'),
                'yaw_limit_deg': _float('yaw_limit_deg'),
                'classifier_path': LaunchConfiguration('classifier_path'),
                'homography_file': LaunchConfiguration('homography_file'),
                'homography_scale_to_m': _float('homography_scale_to_m'),
                'camera_ground_x_m': _float('camera_ground_x_m'),
                'camera_ground_y_m': _float('camera_ground_y_m'),
                'camera_height_m': _float('camera_height_m'),
                'vehicle_detection_height_m': _float(
                    'vehicle_detection_height_m'),
                'require_dependencies': True,
                'require_homography': True,
                'require_registered_layout': True,
                'use_fixed_wheelbase': True,
                'fixed_wheelbase_m': _float('fixed_wheelbase_m'),
            }],
            output='screen'),

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

        Node(
            package='cooperative_parking_robot',
            executable='cctv_robot_marker',
            name='cctv_robot_marker_node',
            condition=IfCondition(enable_markers),
            parameters=[{
                'image_topic': LaunchConfiguration('cctv_rect_topic'),
                'zero_stamp_fallback_to_now': True,
                'homography_file': LaunchConfiguration('homography_file'),
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
                'camera_ground_x_m': _float('camera_ground_x_m'),
                'camera_ground_y_m': _float('camera_ground_y_m'),
                'camera_height_m': _float('camera_height_m'),
                'front_marker_height_m': _float('front_marker_height_m'),
                'rear_marker_height_m': _float('rear_marker_height_m'),
            }],
            output='screen'),

        Node(
            package='cooperative_parking_robot',
            executable='jetson_vision_web',
            name='jetson_vision_web_node',
            condition=IfCondition(enable_web),
            parameters=[LaunchConfiguration('layout_config'), {
                'image_topic': LaunchConfiguration('cctv_rect_topic'),
                'enable_aruco': _bool('debug_enable_aruco'),
                'aruco_dict': LaunchConfiguration('aruco_dict'),
                'front_marker_id': _int('front_marker_id'),
                'rear_marker_id': _int('rear_marker_id'),
                'marker_size_m': _float('marker_size_m'),
                'min_marker_area_px': _float('min_marker_area_px'),
                'min_marker_area_ratio': _float('min_marker_area_ratio'),
                'camera_calib': LaunchConfiguration('cctv_camera_calib'),
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
