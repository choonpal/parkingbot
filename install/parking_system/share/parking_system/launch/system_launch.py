#!/usr/bin/env python3
"""
ros2 launch parking_system system_launch.py

전체 주차 시스템 실행:
  - 오케스트레이터 (상태 머신)
  - CCTV 노드
  - front 로봇
  - rear 로봇
  - 동기 컨트롤러 (TRANSPORT 단계용)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # 전체 지휘
        Node(
            package='parking_system',
            executable='orchestrator',
            name='orchestrator',
            output='screen',
        ),

        # CCTV
        Node(
            package='parking_system',
            executable='cctv',
            name='cctv_node',
            output='screen',
        ),

        # front 로봇
        Node(
            package='parking_system',
            executable='robot',
            name='front_robot',
            parameters=[{
                'name': 'front',
                'serial_port': '/dev/ttyUSB0',
            }],
            output='screen',
        ),

        # rear 로봇
        Node(
            package='parking_system',
            executable='robot',
            name='rear_robot',
            parameters=[{
                'name': 'rear',
                'serial_port': '/dev/ttyUSB1',
            }],
            output='screen',
        ),
    ])
