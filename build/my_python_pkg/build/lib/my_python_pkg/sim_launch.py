#!/usr/bin/env python3
"""
ros2 launch parking_sim sim_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # 앞 로봇 (마스터)
        Node(
            package='parking_sim',
            executable='robot',
            name='front_robot',
            parameters=[{
                'name': 'front',
                'start_x': 0.70,
                'start_y': 0.60,
                'start_theta': 1.5708,
                'noise': 0.0008,
                'wheel_slip': 0.015,
            }],
            output='screen',
        ),

        # 뒤 로봇 (슬레이브)
        Node(
            package='parking_sim',
            executable='robot',
            name='rear_robot',
            parameters=[{
                'name': 'rear',
                'start_x': 0.70,
                'start_y': 0.85,
                'start_theta': 1.5708,
                'noise': 0.0008,
                'wheel_slip': 0.015,
            }],
            output='screen',
        ),

        # CCTV
        Node(
            package='parking_sim',
            executable='cctv',
            name='sim_cctv',
            parameters=[{
                'occupied_slots': [1, 3],  # 1번, 3번에 차 있음
            }],
            output='screen',
        ),

        # 네비게이션 컨트롤러
        Node(
            package='parking_sim',
            executable='controller',
            name='nav_controller',
            parameters=[{
                'wheelbase': 0.25,
                'max_speed': 0.08,
                'arrival_dist': 0.03,
            }],
            output='screen',
        ),

        # 시각화
        Node(
            package='parking_sim',
            executable='visualizer',
            name='visualizer',
            output='screen',
        ),
    ])
