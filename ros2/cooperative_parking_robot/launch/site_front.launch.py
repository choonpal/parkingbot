#!/usr/bin/env python3
"""One-line Front Raspberry Pi production launch using shared site config."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from cooperative_parking_robot.site_config import (
    DEFAULT_SITE_CONFIG, load_site_config, require_site_keys,
)


def generate_launch_description():
    config = load_site_config(DEFAULT_SITE_CONFIG)
    require_site_keys(config, (
        "FRONT_SERIAL", "WHEELBASE", "FRONT_WHEEL_RADIUS",
        "FRONT_ENCODER_PPR", "FRONT_LX", "FRONT_LY",
        "FRONT_LEFT_SENSOR_X", "FRONT_RIGHT_SENSOR_X",
    ), "Front")

    args = {
        "serial_port": config["FRONT_SERIAL"],
        "hardware_profile": "robot-2",
        "enable_serial": "true",
        "require_serial": "true",
        "require_hardware_ready": "true",
        "require_ultrasonic_for_ready": "true",
        "wheelbase": config["WHEELBASE"],
        "wheel_radius": config["FRONT_WHEEL_RADIUS"],
        "encoder_ppr": config["FRONT_ENCODER_PPR"],
        "lx": config["FRONT_LX"],
        "ly": config["FRONT_LY"],
        "left_sensor_to_gripper_x_m": config["FRONT_LEFT_SENSOR_X"],
        "right_sensor_to_gripper_x_m": config["FRONT_RIGHT_SENSOR_X"],
        "use_aruco_distance": "true",
        "simultaneous_entry": "false",
    }

    source = PathJoinSubstitution([
        FindPackageShare("cooperative_parking_robot"),
        "launch", "front_robot.launch.py"])
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(source),
            launch_arguments=args.items())
    ])
