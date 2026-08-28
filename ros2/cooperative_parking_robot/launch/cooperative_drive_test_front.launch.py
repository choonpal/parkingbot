#!/usr/bin/env python3
"""Front (robot-2) half of the bounded cooperative straight-drive test.

Only the STM32 bridge is started.  There is no mission state machine,
individual motion controller, rigid-body coordinator, or automatic cmd_vel
publisher in this launch.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _float(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _bool(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyACM0',
            description='robot-2 Front STM32 stable /dev/serial/by-id path'),
        DeclareLaunchArgument('hardware_profile', default_value='robot-2'),
        DeclareLaunchArgument('wheel_radius', default_value='0.05'),
        DeclareLaunchArgument('encoder_ppr', default_value='5182.0'),
        DeclareLaunchArgument('lx', default_value='0.10'),
        DeclareLaunchArgument('ly', default_value='0.10'),
        DeclareLaunchArgument(
            'require_ultrasonic_for_ready', default_value='false'),
        DeclareLaunchArgument(
            'ultrasonic_frame_timeout_s', default_value='0.50'),

        Node(
            package='cooperative_parking_robot',
            executable='stm32_bridge',
            name='front_drive_test_bridge',
            parameters=[{
                'role': 'front',
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
    ])
