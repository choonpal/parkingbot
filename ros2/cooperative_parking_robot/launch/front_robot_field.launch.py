#!/usr/bin/env python3
"""Field-site wrapper for the Front robot.

The physical HOME poses are side-by-side in the 4.40 m x 3.83 m map.  This
wrapper reuses the normal Front launch but supplies the measured HOME centre;
the ``individual_move`` executable is already mapped to the field adapter on
this branch.
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
        "front_robot.launch.py",
    ])
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                # Registered field HOME: (3.60, 0.60), facing -X.
                # PoseFusion takes its first fresh CCTV marker pose as the
                # authoritative yaw, so only the HOME centre is supplied here.
                "waiting_x": "3.60",
                "waiting_y": "0.60",
                "simultaneous_entry": "false",
            }.items(),
        )
    ])
