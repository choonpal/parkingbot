#!/usr/bin/env python3
"""One-line Rear Raspberry Pi production launch using shared site config."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from cooperative_parking_robot.site_config import (
    DEFAULT_SITE_CONFIG, load_site_config, require_site_keys, site_bool,
)


def generate_launch_description():
    config = load_site_config(DEFAULT_SITE_CONFIG)
    require_site_keys(config, (
        "REAR_SERIAL", "WHEELBASE", "REAR_WHEEL_RADIUS",
        "REAR_ENCODER_PPR", "REAR_LX", "REAR_LY",
        "REAR_LEFT_SENSOR_X", "REAR_RIGHT_SENSOR_X", "REAR_CALIB",
    ), "Rear")

    internal_camera = site_bool(
        config, "REAR_ENABLE_INTERNAL_CAMERA", True)
    if not internal_camera:
        require_site_keys(
            config, ("REAR_EXTERNAL_CAMERA_COMMAND",), "Rear external camera")

    args = {
        "serial_port": config["REAR_SERIAL"],
        "hardware_profile": "robot-1",
        "enable_serial": "true",
        "require_serial": "true",
        "require_hardware_ready": "true",
        "require_ultrasonic_for_ready": "true",
        "enable_rear_camera": "true" if internal_camera else "false",
        "rear_camera_topic": config.get(
            "REAR_CAMERA_TOPIC", "/rear/marker_camera/image"),
        "camera_calib": str(Path(config["REAR_CALIB"]).expanduser()),
        "wheelbase": config["WHEELBASE"],
        "wheel_radius": config["REAR_WHEEL_RADIUS"],
        "encoder_ppr": config["REAR_ENCODER_PPR"],
        "lx": config["REAR_LX"],
        "ly": config["REAR_LY"],
        "left_sensor_to_gripper_x_m": config["REAR_LEFT_SENSOR_X"],
        "right_sensor_to_gripper_x_m": config["REAR_RIGHT_SENSOR_X"],
        "simultaneous_entry": "false",
    }
    if internal_camera:
        args["rear_camera_id"] = config.get("REAR_CAMERA_ID", "0") or "0"

    source = PathJoinSubstitution([
        FindPackageShare("cooperative_parking_robot"),
        "launch", "rear_robot.launch.py"])

    actions = []
    if not internal_camera:
        actions.append(ExecuteProcess(
            cmd=["bash", "-lc", config["REAR_EXTERNAL_CAMERA_COMMAND"]],
            output="screen"))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(source),
        launch_arguments=args.items()))
    return LaunchDescription(actions)
