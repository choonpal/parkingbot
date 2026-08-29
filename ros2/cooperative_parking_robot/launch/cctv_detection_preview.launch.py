#!/usr/bin/env python3
"""Production CCTV 토픽을 읽기만 하는 상세 웹 프리뷰.

카메라와 YOLO를 새로 열지 않는다. ``site_jetson.launch.py``가 발행하는
보정 영상과 ``/cctvN/detections`` envelope를 구독하므로 GPU 모델 중복,
카메라 busy, Production 판정과 프리뷰 판정의 불일치가 없다.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _float(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _int(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def generate_launch_description():
    runtime = Path.home() / '.ros' / 'adaptive_valet_bot'
    return LaunchDescription([
        DeclareLaunchArgument('web_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('web_port', default_value='5008'),
        DeclareLaunchArgument('marker_size_m', default_value='0.24'),
        DeclareLaunchArgument('camera_optics_csv', default_value=''),
        DeclareLaunchArgument('marker_height_m', default_value='0.0'),
        DeclareLaunchArgument('vehicle_detection_height_m', default_value='0.0'),
        DeclareLaunchArgument('stale_after_s', default_value='2.0'),
        DeclareLaunchArgument(
            'image_topics_csv',
            default_value='/cctv0/image_rect,/cctv2/image_rect'),
        DeclareLaunchArgument(
            'detection_topics_csv',
            default_value='/cctv0/detections,/cctv2/detections'),
        DeclareLaunchArgument(
            'layout_yaml',
            default_value=str(runtime / 'parking_layout.yaml')),
        Node(
            package='cooperative_parking_robot',
            executable='camera_preview',
            name='cctv_detection_preview_node',
            parameters=[{
                'image_topics_csv': LaunchConfiguration('image_topics_csv'),
                'labels_csv': 'cctv0,cctv2',
                'detection_topics_csv': LaunchConfiguration(
                    'detection_topics_csv'),
                'web_host': LaunchConfiguration('web_host'),
                'web_port': _int('web_port'),
                'stale_after_s': _float('stale_after_s'),
                'calibration_width_px': 640,
                'calibration_height_px': 360,
                'enable_aruco': True,
                'aruco_dict': 'DICT_4X4_50',
                'marker_size_m': _float('marker_size_m'),
                'camera_optics_csv': LaunchConfiguration('camera_optics_csv'),
                'marker_height_m': _float('marker_height_m'),
                'vehicle_detection_height_m': _float(
                    'vehicle_detection_height_m'),
                'aruco_every_n': 3,
                'enable_bev': True,
                'layout_yaml': LaunchConfiguration('layout_yaml'),
                # 핵심: 모델을 다시 올리지 않고 Production 결과만 표시한다.
                'enable_yolo': False,
                'yolo_switch_mode': 'off',
            }],
            output='screen'),
    ])
