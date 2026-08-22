#!/usr/bin/env python3
"""Field-site wrapper for the Front robot.

The physical HOME poses are side-by-side in the 4.40 m x 3.83 m map.  This
wrapper reuses the normal Front launch but supplies the measured HOME centre
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
                # A split nearest-end exit would drive Front into the slot's
                # 0.23m closed-end clearance.  Both robots clear toward aisle.
                "same_direction_exit": "true",
                "same_direction_exit_sign": "-1",
                # 0.65m leaves the inner Front robot far enough outside the
                # released vehicle to rotate in the open aisle.  Rear parks
                # first; Front then enters the upper HOME.
                "exit_distance_m": "0.65",
                # Front waits for Rear to park and then returns, so the former
                # 90s single-robot return timeout is too short.
                "return_timeout_s": "180.0",
            }.items(),
        )
    ])
