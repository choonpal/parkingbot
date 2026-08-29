#!/usr/bin/env python3
"""ROS 2 Humble 한-PC 구조 확인용 launch.

기본값은 카메라/시리얼을 요구하지 않는 안전한 smoke 모드다.
실차 분산 운용은 cctv_server.launch.py, front_robot.launch.py,
rear_robot.launch.py를 각 장비에서 실행한다.
"""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


FIXED_WHEELBASE = 0.785
FRONT_HOME = (3.60, 0.60)
REAR_HOME = (3.60, 0.20)
HOME_YAW_DEG = 180.0
HOME_YAW_RAD = math.pi


def _bool(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def _int(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def _float(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    default_vehicle_model = PathJoinSubstitution([
        FindPackageShare('cooperative_parking_robot'),
        'models', 'parking_vehicle_yolo11n_seg.pt'])
    default_cctv_calib = PathJoinSubstitution([
        FindPackageShare('cooperative_parking_robot'),
        'config', 'cctv_camera_calibration.npz'])
    sync_config = PathJoinSubstitution([
        FindPackageShare('cooperative_parking_robot'),
        'config', 'sync_params.yaml'])
    id0_calibration = PathJoinSubstitution([
        FindPackageShare('cooperative_parking_robot'),
        'config', 'id0_calibration.yaml'])
    runtime_config_dir = PathJoinSubstitution([
        EnvironmentVariable('HOME'), '.ros', 'adaptive_valet_bot'])
    default_homography = PathJoinSubstitution([
        runtime_config_dir, 'homography_rectified.npy'])
    default_layout_config = PathJoinSubstitution([
        runtime_config_dir, 'parking_layout.yaml'])
    default_registry_database = PathJoinSubstitution([
        runtime_config_dir, 'parking_registry.db'])
    layout_config = LaunchConfiguration('layout_config')

    enable_camera = LaunchConfiguration('enable_opencv_camera')
    enable_rectify = LaunchConfiguration('enable_cctv_rectify')
    enable_vision = LaunchConfiguration('enable_vision')
    enable_cctv_markers = LaunchConfiguration('enable_cctv_robot_markers')
    enable_rear_aruco = LaunchConfiguration('enable_rear_aruco')
    enable_rear_camera = LaunchConfiguration('enable_rear_camera')
    enable_web = PythonExpression([
        "('", LaunchConfiguration('enable_operator_ui'),
        "'.lower() in ('true', '1', 'yes', 'on') and '",
        LaunchConfiguration('enable_vision'),
        "'.lower() in ('true', '1', 'yes', 'on')) or '",
        LaunchConfiguration('enable_debug_overlay'),
        "'.lower() in ('true', '1', 'yes', 'on')",
    ])

    return LaunchDescription([
        LogInfo(msg=(
            'WARNING: full_system.launch.py is a legacy single-host smoke launch; '
            'production dual CCTV must use cctv_server_dual.launch.py plus '
            'front_robot.launch.py and rear_robot.launch.py.')),
        DeclareLaunchArgument(
            'enable_opencv_camera', default_value='false',
            description='이 launch가 cv2.VideoCapture로 천장 카메라를 점유'),
        DeclareLaunchArgument(
            'enable_cctv_rectify', default_value='false',
            description='천장 원본 영상을 /cctv/image_rect로 왜곡 보정'),
        DeclareLaunchArgument(
            'enable_vision', default_value='false',
            description='YOLO/BEV와 fleet manager를 실제 카메라로 실행'),
        DeclareLaunchArgument(
            'enable_cctv_robot_markers', default_value='false'),
        DeclareLaunchArgument('enable_rear_aruco', default_value='false'),
        DeclareLaunchArgument(
            'enable_operator_ui', default_value='true',
            description='enable_vision 운용 시 /kiosk와 /api/* 활성'),
        DeclareLaunchArgument(
            'enable_debug_overlay', default_value='false',
            description='진단 YOLO/ArUco/FPS overlay 및 annotated topic 활성'),
        DeclareLaunchArgument('enable_serial', default_value='false'),
        DeclareLaunchArgument('require_serial', default_value='false'),
        DeclareLaunchArgument('require_hardware_ready', default_value='false'),
        DeclareLaunchArgument(
            'require_ultrasonic_for_ready', default_value='false'),
        DeclareLaunchArgument('ultrasonic_frame_timeout_s', default_value='0.50'),
        DeclareLaunchArgument('ultrasonic_threshold_m', default_value='0.10'),
        DeclareLaunchArgument(
            'ultrasonic_exit_hysteresis_m', default_value='0.02'),
        DeclareLaunchArgument(
            'front_left_sensor_to_gripper_x_m', default_value='0.0'),
        DeclareLaunchArgument(
            'front_right_sensor_to_gripper_x_m', default_value='0.0'),
        DeclareLaunchArgument(
            'rear_left_sensor_to_gripper_x_m', default_value='0.0'),
        DeclareLaunchArgument(
            'rear_right_sensor_to_gripper_x_m', default_value='0.0'),
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
        DeclareLaunchArgument(
            'rear_camera_topic', default_value='/rear/marker_camera/image'),
        DeclareLaunchArgument('enable_rear_camera', default_value='true'),
        DeclareLaunchArgument('rear_camera_id', default_value='1'),
        DeclareLaunchArgument('rear_camera_device', default_value=''),
        DeclareLaunchArgument('rear_camera_gst', default_value=''),
        DeclareLaunchArgument('rear_camera_width', default_value='1280'),
        DeclareLaunchArgument('rear_camera_height', default_value='720'),
        DeclareLaunchArgument('rear_camera_fps', default_value='8.0'),
        DeclareLaunchArgument('rear_camera_capture_fps', default_value='30.0'),
        DeclareLaunchArgument('rear_camera_fourcc', default_value='MJPG'),
        DeclareLaunchArgument('rear_camera_standby_fps', default_value='1.0'),
        DeclareLaunchArgument(
            'rear_camera_activation_drop_frames', default_value='2'),
        DeclareLaunchArgument('model_path', default_value=default_vehicle_model),
        DeclareLaunchArgument('model_mode', default_value='vehicle_seg'),
        DeclareLaunchArgument('inference_imgsz', default_value='640'),
        DeclareLaunchArgument('process_every_n', default_value='3'),
        DeclareLaunchArgument('confidence', default_value='0.4'),
        DeclareLaunchArgument('yaw_pca_min_ratio', default_value='1.25'),
        DeclareLaunchArgument('yaw_ema_alpha', default_value='0.15'),
        DeclareLaunchArgument('yaw_limit_deg', default_value='90.0'),
        DeclareLaunchArgument('aruco_dict', default_value='DICT_4X4_50'),
        DeclareLaunchArgument(
            'homography_file', default_value=default_homography),
        DeclareLaunchArgument('homography_scale_to_m', default_value='1.0'),
        DeclareLaunchArgument(
            'layout_config', default_value=default_layout_config,
            description='브라우저에서 생성한 주차면/맵 YAML 경로'),
        DeclareLaunchArgument(
            'parking_registry_db_path',
            default_value=default_registry_database,
            description='Fleet Parking Registry SQLite 파일'),
        DeclareLaunchArgument('front_marker_id', default_value='2'),
        DeclareLaunchArgument('rear_marker_id', default_value='1'),
        DeclareLaunchArgument('min_marker_area_px', default_value='100.0'),
        DeclareLaunchArgument('min_marker_area_ratio', default_value='0.0003'),
        DeclareLaunchArgument('marker_size_m', default_value='0.24'),
        DeclareLaunchArgument('debug_web_port', default_value='5000'),
        DeclareLaunchArgument('camera_ground_x_m', default_value='0.0'),
        DeclareLaunchArgument('camera_ground_y_m', default_value='0.0'),
        DeclareLaunchArgument('camera_height_m', default_value='0.0'),
        DeclareLaunchArgument('vehicle_detection_height_m', default_value='0.0'),
        DeclareLaunchArgument('front_marker_height_m', default_value='0.0'),
        DeclareLaunchArgument('rear_marker_height_m', default_value='0.0'),
        DeclareLaunchArgument(
            'rear_camera_calib', default_value='rear_camera_calibration.npz'),
        DeclareLaunchArgument(
            'rear_aruco_yaw_sign', default_value='1.0'),
        DeclareLaunchArgument(
            'rear_aruco_gray_gain', default_value='1.0'),
        DeclareLaunchArgument(
            'simultaneous_entry', default_value='false'),
        DeclareLaunchArgument(
            'stop_after_align', default_value='false',
            description=(
                'Hold both robots after axle alignment; never commit LIFT')),
        DeclareLaunchArgument(
            'planning_validation_mode', default_value='warn_only',
            description=(
                'MVP Fleet preflight policy: warn_only records model-based '
                'geometry findings without suppressing an executable path')),
        DeclareLaunchArgument(
            'same_direction_exit', default_value='false'),
        DeclareLaunchArgument(
            'same_direction_exit_sign', default_value='1'),
        DeclareLaunchArgument(
            'exit_sync_gain', default_value='0.15'),
        DeclareLaunchArgument('prealign_hold_n', default_value='10'),
        DeclareLaunchArgument('cctv_marker_timeout_s', default_value='0.50'),
        DeclareLaunchArgument(
            'relative_lateral_tolerance_m', default_value='0.03'),
        DeclareLaunchArgument(
            'axle_position_tolerance_m', default_value='0.15'),
        DeclareLaunchArgument(
            'use_ultrasonic_lateral', default_value='true'),
        DeclareLaunchArgument(
            'ultrasonic_lateral_timeout_s', default_value='0.30'),
        DeclareLaunchArgument(
            'ultrasonic_lateral_yaw_gate_deg', default_value='10.0'),
        DeclareLaunchArgument(
            'lateral_deviation_limit_m', default_value='0.030'),
        DeclareLaunchArgument('lateral_deviation_n', default_value='5'),
        DeclareLaunchArgument('max_scan_retry', default_value='2'),
        DeclareLaunchArgument('lateral_median_n', default_value='3'),
        DeclareLaunchArgument(
            'lateral_pair_timeout_s', default_value='0.20'),
        DeclareLaunchArgument('front_lateral_sign', default_value='1.0'),
        DeclareLaunchArgument('rear_lateral_sign', default_value='1.0'),
        DeclareLaunchArgument('front_serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('rear_serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument(
            'front_hardware_profile', default_value='robot-2'),
        DeclareLaunchArgument(
            'rear_hardware_profile', default_value='robot-1'),

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
            condition=IfCondition(enable_rectify),
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
            condition=IfCondition(enable_vision),
            parameters=[layout_config, {
                'image_topic': LaunchConfiguration('cctv_rect_topic'),
                'model_path': LaunchConfiguration('model_path'),
                'model_mode': LaunchConfiguration('model_mode'),
                'inference_imgsz': _int('inference_imgsz'),
                'process_every_n': _int('process_every_n'),
                'confidence': _float('confidence'),
                'yaw_pca_min_ratio': _float('yaw_pca_min_ratio'),
                'yaw_ema_alpha': _float('yaw_ema_alpha'),
                'yaw_limit_deg': _float('yaw_limit_deg'),
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
                'fixed_wheelbase_m': FIXED_WHEELBASE,
            }],
            output='screen'),
        Node(
            package='cooperative_parking_robot',
            executable='fleet_manager',
            name='fleet_manager_node',
            condition=IfCondition(enable_vision),
            parameters=[layout_config, {
                'require_registered_layout': True,
                'require_valid_vehicle_spec': True,
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
            condition=IfCondition(enable_cctv_markers),
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
                'front_yaw_offset_deg': 0.0,
                'front_marker_offset_x_m': 0.0,
                'rear_yaw_offset_deg': 0.0,
                'rear_marker_offset_x_m': 0.0,
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
            parameters=[layout_config, {
                'image_topic': LaunchConfiguration('cctv_rect_topic'),
                'enable_aruco': True,
                'aruco_dict': LaunchConfiguration('aruco_dict'),
                'front_marker_id': _int('front_marker_id'),
                'rear_marker_id': _int('rear_marker_id'),
                'marker_size_m': _float('marker_size_m'),
                'min_marker_area_px': _float('min_marker_area_px'),
                'min_marker_area_ratio': _float('min_marker_area_ratio'),
                'camera_calib': LaunchConfiguration('cctv_camera_calib'),
                'calibration_width_px': _int('calibration_width_px'),
                'calibration_height_px': _int('calibration_height_px'),
                'web_port': _int('debug_web_port'),
                'enable_operator_ui': _bool('enable_operator_ui'),
                'enable_debug_overlay': _bool('enable_debug_overlay'),
            }],
            output='screen'),

        Node(
            package='cooperative_parking_robot',
            executable='rigid_body_sync',
            name='rigid_body_sync_node',
            parameters=[id0_calibration, sync_config, {
                'wheelbase': FIXED_WHEELBASE,
                'use_vehicle_spec_wheelbase': True,
                'max_speed': 0.08,
                'hold_initial_yaw': True,
                # 경로 중에는 yaw를 고정하고 슬롯 밖 staging에서만 회전한다.
                'align_to_slot_yaw': True,
                'final_approach_dist': 0.02,
                'use_aruco_distance': True,
                'cctv_marker_timeout_s': _float(
                    'cctv_marker_timeout_s'),
                'initialize_offset_from_target_pose': True,
            }],
            output='screen'),

        # P0-1: rear 전면 카메라 발행자 (ID0 관측). rear_robot.launch.py와 동일.
        Node(
            package='cooperative_parking_robot',
            executable='opencv_camera',
            name='rear_marker_camera_node',
            condition=IfCondition(enable_rear_camera),
            parameters=[{
                'camera_id': _int('rear_camera_id'),
                'camera_device': LaunchConfiguration(
                    'rear_camera_device'),
                'gstreamer_pipeline': LaunchConfiguration('rear_camera_gst'),
                'output_topic': LaunchConfiguration('rear_camera_topic'),
                'frame_id': 'rear_marker_camera',
                'width': _int('rear_camera_width'),
                'height': _int('rear_camera_height'),
                'fps': _float('rear_camera_fps'),
                'capture_fps': _float('rear_camera_capture_fps'),
                'v4l2_fourcc': LaunchConfiguration(
                    'rear_camera_fourcc'),
                'buffer_size': 1,
                'require_camera': True,
                'runtime_enable_topic': '/rear/relative_vision_enable',
                'runtime_ready_topic': '/rear/marker_camera_ready',
                'start_enabled': False,
                'standby_fps': _float('rear_camera_standby_fps'),
                'activation_drop_frames': _int(
                    'rear_camera_activation_drop_frames'),
            }],
            output='screen'),

        Node(
            package='cooperative_parking_robot',
            executable='aruco_tracker',
            name='aruco_tracker_node',
            condition=IfCondition(enable_rear_aruco),
            parameters=[id0_calibration, {
                'image_topic': LaunchConfiguration('rear_camera_topic'),
                'marker_id': 0,
                'camera_calib': LaunchConfiguration('rear_camera_calib'),
                'yaw_offset_deg': 0.0,
                'yaw_sign': _float('rear_aruco_yaw_sign'),
                'gray_gain': _float('rear_aruco_gray_gain'),
                'allow_uncalibrated': False,
                'runtime_enable_topic': '/rear/relative_vision_enable',
                'runtime_ready_topic': '/rear/relative_vision_ready',
                'start_enabled': False,
            }],
            output='screen'),

        Node(
            package='cooperative_parking_robot',
            executable='individual_move',
            name='front_individual_move',
            parameters=[id0_calibration, {
                'role': 'front',
                'simultaneous_entry': _bool('simultaneous_entry'),
                'same_direction_exit': _bool('same_direction_exit'),
                'same_direction_exit_sign': _int('same_direction_exit_sign'),
                'exit_sync_gain': _float('exit_sync_gain'),
                'prealign_hold_n': _int('prealign_hold_n'),
                'use_ultrasonic_lateral': _bool(
                    'use_ultrasonic_lateral'),
                'ultrasonic_lateral_timeout_s': _float(
                    'ultrasonic_lateral_timeout_s'),
                'ultrasonic_lateral_yaw_gate_deg': _float(
                    'ultrasonic_lateral_yaw_gate_deg'),
                'lateral_deviation_limit_m': _float(
                    'lateral_deviation_limit_m'),
                'lateral_deviation_n': _int('lateral_deviation_n'),
                'max_scan_retry': _int('max_scan_retry'),
                'default_wheelbase': FIXED_WHEELBASE,
                'use_vehicle_spec_wheelbase': True,
                'waiting_x': FRONT_HOME[0],
                'waiting_y': FRONT_HOME[1],
                'home_yaw_deg': HOME_YAW_DEG,
                'vehicle_half_length_m': 0.45,
                'vehicle_half_width_m': 0.175,
                'robot_length_m': 0.565,
                'robot_width_m': 0.420,
                'minimum_inter_robot_gap_m': 0.22,
                'entry_standoff_m': 0.85,
                'entry_side_offset_m': 0.50,
                'exit_distance_m': 0.50,
                'substate_timeout_s': 60.0,
                'cctv_marker_timeout_s': _float(
                    'cctv_marker_timeout_s'),
                'relative_lateral_tolerance_m': _float(
                    'relative_lateral_tolerance_m'),
            }],
            output='screen'),
        Node(
            package='cooperative_parking_robot',
            executable='individual_move',
            name='rear_individual_move',
            parameters=[id0_calibration, {
                'role': 'rear',
                'simultaneous_entry': _bool('simultaneous_entry'),
                'same_direction_exit': _bool('same_direction_exit'),
                'same_direction_exit_sign': _int('same_direction_exit_sign'),
                'exit_sync_gain': _float('exit_sync_gain'),
                'prealign_hold_n': _int('prealign_hold_n'),
                'use_ultrasonic_lateral': _bool(
                    'use_ultrasonic_lateral'),
                'ultrasonic_lateral_timeout_s': _float(
                    'ultrasonic_lateral_timeout_s'),
                'ultrasonic_lateral_yaw_gate_deg': _float(
                    'ultrasonic_lateral_yaw_gate_deg'),
                'lateral_deviation_limit_m': _float(
                    'lateral_deviation_limit_m'),
                'lateral_deviation_n': _int('lateral_deviation_n'),
                'max_scan_retry': _int('max_scan_retry'),
                'default_wheelbase': FIXED_WHEELBASE,
                'use_vehicle_spec_wheelbase': True,
                'waiting_x': REAR_HOME[0],
                'waiting_y': REAR_HOME[1],
                'home_yaw_deg': HOME_YAW_DEG,
                'vehicle_half_length_m': 0.45,
                'vehicle_half_width_m': 0.175,
                'robot_length_m': 0.565,
                'robot_width_m': 0.420,
                'minimum_inter_robot_gap_m': 0.22,
                'entry_standoff_m': 0.85,
                'entry_side_offset_m': 0.50,
                'exit_distance_m': 0.50,
                'substate_timeout_s': 60.0,
                'cctv_marker_timeout_s': _float(
                    'cctv_marker_timeout_s'),
                'relative_lateral_tolerance_m': _float(
                    'relative_lateral_tolerance_m'),
            }],
            output='screen'),

        Node(
            package='cooperative_parking_robot',
            executable='ultrasonic_edge',
            name='front_ultrasonic',
            parameters=[{
                'role': 'front',
                'threshold_m': _float('ultrasonic_threshold_m'),
                'exit_hysteresis_m': _float(
                    'ultrasonic_exit_hysteresis_m'),
                'sensor_timeout_s': _float('ultrasonic_frame_timeout_s'),
                'left_sensor_to_gripper_x_m': _float(
                    'front_left_sensor_to_gripper_x_m'),
                'right_sensor_to_gripper_x_m': _float(
                    'front_right_sensor_to_gripper_x_m'),
                'lateral_median_n': _int('lateral_median_n'),
                'lateral_pair_timeout_s': _float(
                    'lateral_pair_timeout_s'),
                'lateral_sign': _float('front_lateral_sign'),
                'axle_position_tolerance_m': _float(
                    'axle_position_tolerance_m'),
            }],
            output='screen'),
        Node(
            package='cooperative_parking_robot',
            executable='state_machine',
            name='front_state_machine',
            parameters=[{
                'role': 'front',
                'approach_timeout_s': 150.0,
                'align_timeout_s': 120.0,
                'drive_timeout_s': 120.0,
                'require_hardware_ready': _bool('require_hardware_ready'),
                'stop_after_align': _bool('stop_after_align'),
            }],
            output='screen'),
        Node(
            package='cooperative_parking_robot',
            executable='stm32_bridge',
            name='front_bridge',
            parameters=[{
                'role': 'front',
                'hardware_profile': LaunchConfiguration(
                    'front_hardware_profile'),
                'serial_port': LaunchConfiguration('front_serial_port'),
                'enable_serial': _bool('enable_serial'),
                'require_serial': _bool('require_serial'),
                'require_ultrasonic_for_ready': _bool(
                    'require_ultrasonic_for_ready'),
                'ultrasonic_frame_timeout_s': _float(
                    'ultrasonic_frame_timeout_s'),
            }],
            output='screen'),
        Node(
            package='cooperative_parking_robot',
            executable='pose_fusion',
            name='front_pose_fusion',
            parameters=[{
                'role': 'front',
                'init_x': FRONT_HOME[0],
                'init_y': FRONT_HOME[1],
                'init_yaw': HOME_YAW_RAD,
            }],
            output='screen'),

        Node(
            package='cooperative_parking_robot',
            executable='ultrasonic_edge',
            name='rear_ultrasonic',
            parameters=[{
                'role': 'rear',
                'threshold_m': _float('ultrasonic_threshold_m'),
                'exit_hysteresis_m': _float(
                    'ultrasonic_exit_hysteresis_m'),
                'sensor_timeout_s': _float('ultrasonic_frame_timeout_s'),
                'left_sensor_to_gripper_x_m': _float(
                    'rear_left_sensor_to_gripper_x_m'),
                'right_sensor_to_gripper_x_m': _float(
                    'rear_right_sensor_to_gripper_x_m'),
                'lateral_median_n': _int('lateral_median_n'),
                'lateral_pair_timeout_s': _float(
                    'lateral_pair_timeout_s'),
                'lateral_sign': _float('rear_lateral_sign'),
                'axle_position_tolerance_m': _float(
                    'axle_position_tolerance_m'),
            }],
            output='screen'),
        Node(
            package='cooperative_parking_robot',
            executable='state_machine',
            name='rear_state_machine',
            parameters=[{
                'role': 'rear',
                'approach_timeout_s': 150.0,
                'align_timeout_s': 120.0,
                'drive_timeout_s': 120.0,
                'require_hardware_ready': _bool('require_hardware_ready'),
                'stop_after_align': _bool('stop_after_align'),
            }],
            output='screen'),
        Node(
            package='cooperative_parking_robot',
            executable='stm32_bridge',
            name='rear_bridge',
            parameters=[{
                'role': 'rear',
                'hardware_profile': LaunchConfiguration(
                    'rear_hardware_profile'),
                'serial_port': LaunchConfiguration('rear_serial_port'),
                'enable_serial': _bool('enable_serial'),
                'require_serial': _bool('require_serial'),
                'require_ultrasonic_for_ready': _bool(
                    'require_ultrasonic_for_ready'),
                'ultrasonic_frame_timeout_s': _float(
                    'ultrasonic_frame_timeout_s'),
            }],
            output='screen'),
        Node(
            package='cooperative_parking_robot',
            executable='pose_fusion',
            name='rear_pose_fusion',
            parameters=[{
                'role': 'rear',
                'init_x': REAR_HOME[0],
                'init_y': REAR_HOME[1],
                'init_yaw': HOME_YAW_RAD,
            }],
            output='screen'),
    ])
