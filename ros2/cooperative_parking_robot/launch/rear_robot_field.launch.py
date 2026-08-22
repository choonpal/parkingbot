#!/usr/bin/env python3
"""Field-site wrapper for the Rear robot.

The physical HOME poses are side-by-side in the 4.40 m x 3.83 m map.  This
wrapper reuses the normal Rear launch but supplies the measured HOME centre
and the field-safe shared exit toward the aisle.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base_launch = PathJoinSubstitution([
        FindPackageShare("cooperative_parking_robot"),
        "launch",
        "rear_robot.launch.py",
    ])
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                # Registered field HOME: (3.60, 0.20), facing -X.
                # PoseFusion takes its first fresh CCTV marker pose as the
                # authoritative yaw, so only the HOME centre is supplied here.
                "waiting_x": "3.60",
                "waiting_y": "0.20",
                "simultaneous_entry": "false",
                # Keep axle-centre separation while both robots clear the
                # released vehicle toward the aisle.
                "same_direction_exit": "true",
                "same_direction_exit_sign": "-1",
                # Rear is the outer robot after the shared exit.  It rotates
                # in the aisle and returns to the lower HOME before Front.
                "exit_distance_m": "0.65",
            }.items(),
        )
    ])
