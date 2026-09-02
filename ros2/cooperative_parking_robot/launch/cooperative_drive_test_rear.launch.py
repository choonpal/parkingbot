#!/usr/bin/env python3
"""Rear (robot-1) camera, bridge, and dashboard for a 10 cm pair test.

The assembled robot has one shared main power switch.  Start with all wheels
secured off the floor; this launch cannot and does not isolate motor power.
Motion remains disabled until the dashboard receives ARM and START requests.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
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
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyACM0',
            description='robot-1 Rear STM32 stable /dev/serial/by-id path'),
        DeclareLaunchArgument('hardware_profile', default_value='robot-1'),
        DeclareLaunchArgument(
            'camera_device', default_value='',
            description='Stable /dev/v4l/by-path/...-video-index0 path'),
        DeclareLaunchArgument('camera_id', default_value='0'),
        DeclareLaunchArgument(
            'camera_calib',
            default_value=str(
                Path.home() / 'ov2710_calib_23mm_white.npz')),
        DeclareLaunchArgument(
            'image_topic', default_value='/rear/marker_camera/image'),
        DeclareLaunchArgument(
            'preview_image_topic',
            default_value='/rear/marker_camera/preview'),
        DeclareLaunchArgument('preview_width', default_value='640'),
        DeclareLaunchArgument('preview_height', default_value='360'),
        DeclareLaunchArgument('preview_fps', default_value='4.0'),
        DeclareLaunchArgument('width', default_value='1280'),
        DeclareLaunchArgument('height', default_value='720'),
        DeclareLaunchArgument('fps', default_value='8.0'),
        DeclareLaunchArgument(
            'marker_size_m', default_value='0.10',
            description='Front rear-face ID0 black-square side length'),
        DeclareLaunchArgument('yaw_offset_deg', default_value='0.0'),
        DeclareLaunchArgument('yaw_sign', default_value='1.0'),
        DeclareLaunchArgument('gray_gain', default_value='1.0'),
        DeclareLaunchArgument(
            'aruco_min_marker_distance_rate', default_value='0.02'),
        DeclareLaunchArgument('wheel_radius', default_value='0.05'),
        DeclareLaunchArgument('encoder_ppr', default_value='5182.0'),
        DeclareLaunchArgument('lx', default_value='0.10'),
        DeclareLaunchArgument('ly', default_value='0.10'),
        DeclareLaunchArgument(
            'require_ultrasonic_for_ready', default_value='false'),
        DeclareLaunchArgument(
            'ultrasonic_frame_timeout_s', default_value='0.50'),
        DeclareLaunchArgument('preview_port', default_value='5005'),
        DeclareLaunchArgument(
            'preview_enable_aruco', default_value='false',
            description=(
                'The control tracker already detects every frame; enable only '
                'when a duplicate diagnostic overlay is worth the CPU cost')),
        DeclareLaunchArgument('dashboard_port', default_value='5006'),
        DeclareLaunchArgument('test_speed_mps', default_value='0.0628'),
        DeclareLaunchArgument('test_distance_m', default_value='0.10'),
        DeclareLaunchArgument('max_duration_s', default_value='4.0'),
        DeclareLaunchArgument(
            'enable_drive_test_dashboard', default_value='true'),
        DeclareLaunchArgument(
            'id0_calibration', default_value=PathJoinSubstitution([
                FindPackageShare('cooperative_parking_robot'), 'config',
                'id0_calibration.yaml']),
            description='shared raw ID0 forward-offset calibration YAML'),

        Node(
            package='cooperative_parking_robot',
            executable='opencv_camera',
            name='rear_drive_test_camera',
            parameters=[{
                'camera_device': LaunchConfiguration('camera_device'),
                'camera_id': _int('camera_id'),
                'output_topic': LaunchConfiguration('image_topic'),
                'preview_topic': LaunchConfiguration('preview_image_topic'),
                'preview_width': _int('preview_width'),
                'preview_height': _int('preview_height'),
                'preview_fps': _float('preview_fps'),
                'frame_id': 'rear_marker_camera',
                'width': _int('width'),
                'height': _int('height'),
                'fps': _float('fps'),
                'buffer_size': 1,
                'require_camera': True,
            }],
            # USB UVC devices can remain busy briefly after a rapid test
            # restart. Keep strict open failure, but retry this sensor process;
            # the controller remains blocked without fresh ArUco pose.
            respawn=True,
            respawn_delay=2.0,
            output='screen'),

        Node(
            package='cooperative_parking_robot',
            executable='aruco_tracker',
            name='rear_drive_test_aruco',
            parameters=[LaunchConfiguration('id0_calibration'), {
                'image_topic': LaunchConfiguration('image_topic'),
                'marker_id': 0,
                'marker_size_m': _float('marker_size_m'),
                'camera_calib': LaunchConfiguration('camera_calib'),
                'aruco_dict': 'DICT_4X4_50',
                'yaw_offset_deg': _float('yaw_offset_deg'),
                'yaw_sign': _float('yaw_sign'),
                'gray_gain': _float('gray_gain'),
                'min_marker_distance_rate': _float(
                    'aruco_min_marker_distance_rate'),
                'allow_uncalibrated': False,
            }],
            output='screen'),

        Node(
            package='cooperative_parking_robot',
            executable='camera_preview',
            name='rear_drive_test_camera_preview',
            parameters=[{
                'image_topics_csv': LaunchConfiguration('preview_image_topic'),
                'labels_csv': 'robot-1 Rear / robot-2 Front ID0',
                'web_host': '0.0.0.0',
                'web_port': _int('preview_port'),
                'jpeg_quality': 80,
                'grid_step_px': 50,
                'calibration_width_px': _int('preview_width'),
                'calibration_height_px': _int('preview_height'),
                'stale_after_s': 1.0,
                'enable_aruco': _bool('preview_enable_aruco'),
                'aruco_dict': 'DICT_4X4_50',
                'marker_size_m': _float('marker_size_m'),
                'aruco_min_marker_distance_rate': _float(
                    'aruco_min_marker_distance_rate'),
                # The control tracker keeps all 12 Hz frames. The browser
                # overlay is diagnostic-only, so detect at 6 Hz to avoid a
                # second full-core 720p ArUco workload on Raspberry Pi 4.
                'aruco_every_n': 2,
                'relative_pose_topic': '/sync/relative_pose',
                'marker_visible_topic': '/sync/marker_visible',
                'enable_bev': False,
                'enable_yolo': False,
            }],
            output='screen'),

        Node(
            package='cooperative_parking_robot',
            executable='stm32_bridge',
            name='rear_drive_test_bridge',
            parameters=[{
                'role': 'rear',
                'hardware_profile': LaunchConfiguration('hardware_profile'),
                'serial_port': LaunchConfiguration('serial_port'),
                'serial_baud': 115200,
                'enable_serial': True,
                'require_serial': True,
                'wheel_radius': _float('wheel_radius'),
                'encoder_ppr': _float('encoder_ppr'),
                'lx': _float('lx'),
                'ly': _float('ly'),
                'max_linear_mps': 0.08,
                'max_angular_rps': 0.20,
                'require_ultrasonic_for_ready': _bool(
                    'require_ultrasonic_for_ready'),
                'ultrasonic_frame_timeout_s': _float(
                    'ultrasonic_frame_timeout_s'),
            }],
            output='screen'),

        Node(
            package='cooperative_parking_robot',
            executable='cooperative_drive_test',
            name='cooperative_drive_test_dashboard',
            condition=IfCondition(
                LaunchConfiguration('enable_drive_test_dashboard')),
            parameters=[{
                'speed_mps': _float('test_speed_mps'),
                'distance_m': _float('test_distance_m'),
                'max_duration_s': _float('max_duration_s'),
                'web_host': '0.0.0.0',
                'web_port': _int('dashboard_port'),
                'preview_port': _int('preview_port'),
                'preview_path': '/video/0',
                'min_marker_distance_m': 0.15,
                'max_marker_distance_m': 1.00,
                'initial_lateral_limit_m': 0.10,
                'initial_yaw_limit_deg': 15.0,
            }],
            output='screen'),

    ])
