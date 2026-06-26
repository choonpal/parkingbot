#!/usr/bin/env python3
"""
ros2 launch parking_robot system_launch.py

WBS 전체 시스템:
  [파트1] CCTV 서버: yolo_bev_map + fleet_manager
  [파트2] 로봇 두뇌: ultrasonic + aruco + rigid_body_sync + state_machine (×2)
  [파트3] STM32: 펌웨어 (별도 플래시)
  + Nav2 (별도 bringup)
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('parking_robot')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    return LaunchDescription([

        # ===== [파트1] CCTV 서버 =====
        Node(package='parking_robot', executable='yolo_bev_map',
             name='yolo_bev_map_node', output='screen'),
        Node(package='parking_robot', executable='fleet_manager',
             name='fleet_manager_node', output='screen'),

        # ===== [파트2] 로봇 두뇌 — 공유 노드 =====
        # 초음파 (front/rear 각각 실행하려면 namespace 분리)
        Node(package='parking_robot', executable='ultrasonic_edge',
             name='ultrasonic_edge_node', output='screen'),
        Node(package='parking_robot', executable='aruco_tracker',
             name='aruco_tracker_node', output='screen'),

        # 강체 동기 제어기 (Nav2 cmd_vel → 두 로봇)
        Node(package='parking_robot', executable='rigid_body_sync',
             name='rigid_body_sync_node',
             parameters=[{'wheelbase': 0.25, 'max_speed': 0.08}],
             output='screen'),

        # 상태 머신 (front)
        Node(package='parking_robot', executable='state_machine',
             name='front_state_machine',
             parameters=[{'role': 'front'}], output='screen'),
        # 상태 머신 (rear)
        Node(package='parking_robot', executable='state_machine',
             name='rear_state_machine',
             parameters=[{'role': 'rear'}], output='screen'),

        # ===== Nav2 =====
        # 실제: nav2_bringup navigation_launch.py (params=nav2_params)
    ])
