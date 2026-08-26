#!/usr/bin/env python3
"""카메라 왜곡 보정 + 브라우저형 BEV/주차면 등록 전용 launch.

YOLO/Fleet/천장 마커 노드는 일부러 실행하지 않는다. Homography가 아직 없는
최초 설치 상태에서도 이 launch만으로 등록 파일을 만들 수 있어야 하기 때문이다.

천장 카메라 2대 등록 (v1.11) — 이 launch를 카메라마다 한 번씩 돌린다.

  # 1회차: cam0
  ros2 launch cooperative_parking_robot bev_layout_calibration.launch.py \\
    enable_opencv_camera:=true camera_id:=0 camera_label:=cam0 \\
    cctv_raw_topic:=/cctv0/image_raw cctv_rect_topic:=/cctv0/image_rect \\
    cctv_camera_calib:=/absolute/cctv0_camera_calibration.npz \\
    homography_output_file:=$HOME/.ros/adaptive_valet_bot/homography_cam0_rectified.npy

  # 2회차: cam2 (같은 바닥 점에 같은 실측 X,Y를 다시 입력할 것)
  ros2 launch cooperative_parking_robot bev_layout_calibration.launch.py \\
    enable_opencv_camera:=true camera_id:=2 camera_label:=cam2 \\
    cctv_raw_topic:=/cctv2/image_raw cctv_rect_topic:=/cctv2/image_rect \\
    cctv_camera_calib:=/absolute/cctv2_camera_calibration.npz \\
    homography_output_file:=$HOME/.ros/adaptive_valet_bot/homography_cam2_rectified.npy \\
    append_existing_layout:=true

layout_output_file은 두 번 모두 같은 경로(parking_layout.yaml)를 쓴다.
2회차의 append_existing_layout:=true가 1회차 슬롯을 지우지 않게 해준다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _bool(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def _float(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _int(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def generate_launch_description():
    enable_camera = LaunchConfiguration('enable_opencv_camera')
    enable_rectify = LaunchConfiguration('enable_rectify')
    default_calibration = PathJoinSubstitution([
        FindPackageShare('cooperative_parking_robot'),
        'config', 'cctv_camera_calibration.npz'])
    runtime_config_dir = PathJoinSubstitution([
        EnvironmentVariable('HOME'), '.ros', 'adaptive_valet_bot'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_opencv_camera', default_value='false',
            description='true면 이 패키지가 천장 카메라를 직접 연다'),
        DeclareLaunchArgument(
            'enable_rectify', default_value='true',
            description='raw 토픽을 렌즈 왜곡 보정해 /cctv/image_rect로 발행'),
        DeclareLaunchArgument('camera_id', default_value='0'),
        DeclareLaunchArgument('camera_width_px', default_value='1280'),
        DeclareLaunchArgument('camera_height_px', default_value='720'),
        DeclareLaunchArgument('camera_fps', default_value='30.0'),
        DeclareLaunchArgument('camera_gstreamer_pipeline', default_value=''),
        DeclareLaunchArgument('cctv_raw_topic', default_value='/cctv/image_raw'),
        DeclareLaunchArgument('cctv_rect_topic', default_value='/cctv/image_rect'),
        DeclareLaunchArgument(
            'cctv_camera_calib', default_value=default_calibration),
        DeclareLaunchArgument('calibration_width_px', default_value='1280'),
        DeclareLaunchArgument('calibration_height_px', default_value='720'),
        DeclareLaunchArgument(
            'homography_output_file',
            default_value=PathJoinSubstitution([
                runtime_config_dir, 'homography_rectified.npy']),
            description='픽셀->metre Homography 저장 경로'),
        DeclareLaunchArgument(
            'layout_output_file',
            default_value=PathJoinSubstitution([
                runtime_config_dir, 'parking_layout.yaml']),
            description='등록한 슬롯/대기영역 parameter YAML 저장 경로'),
        DeclareLaunchArgument('web_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('web_port', default_value='5001'),
        DeclareLaunchArgument('jpeg_quality', default_value='88'),
        DeclareLaunchArgument('preview_pixels_per_m', default_value='120'),
        DeclareLaunchArgument('map_origin_x_m', default_value='-0.40'),
        DeclareLaunchArgument('map_origin_y_m', default_value='-0.80'),
        DeclareLaunchArgument('map_width_m', default_value='4.80'),
        DeclareLaunchArgument('map_height_m', default_value='4.63'),
        # --- v1.11 천장 카메라 2대 ---
        DeclareLaunchArgument(
            'camera_label', default_value='cam0',
            description='지금 등록 중인 카메라 이름(로그/메타데이터 표시용)'),
        DeclareLaunchArgument(
            'append_existing_layout', default_value='false',
            description=(
                '두 번째 카메라를 등록할 때 true. 기존 parking_layout.yaml의 '
                '슬롯을 유지한 채 이번에 등록한 슬롯만 추가/갱신한다')),

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
            executable='bev_layout_calibrator',
            name='bev_layout_calibrator_node',
            parameters=[{
                'image_topic': LaunchConfiguration('cctv_rect_topic'),
                'homography_output_file': LaunchConfiguration(
                    'homography_output_file'),
                'layout_output_file': LaunchConfiguration(
                    'layout_output_file'),
                'web_host': LaunchConfiguration('web_host'),
                'web_port': _int('web_port'),
                'jpeg_quality': _int('jpeg_quality'),
                'preview_pixels_per_m': _int('preview_pixels_per_m'),
                'default_map_origin_x_m': _float('map_origin_x_m'),
                'default_map_origin_y_m': _float('map_origin_y_m'),
                'default_map_width_m': _float('map_width_m'),
                'default_map_height_m': _float('map_height_m'),
                'camera_label': LaunchConfiguration('camera_label'),
                'append_existing_layout': _bool('append_existing_layout'),
            }],
            output='screen'),
    ])
