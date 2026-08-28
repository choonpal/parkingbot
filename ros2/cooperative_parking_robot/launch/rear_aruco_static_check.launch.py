#!/usr/bin/env python3
"""Perception-only ID0 bench check for Rear (robot-1).

This launch intentionally contains no STM32 bridge, state machine, motion
controller, or cmd_vel publisher.  It starts a browser-friendly camera preview
alongside the real tracker.  The assembled robot has one shared main power
switch, so the hardware must be secured with every wheel off the floor and the
check must run in an isolated ROS domain.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _float(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _int(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def generate_launch_description():
    id0_calibration = PathJoinSubstitution([
        FindPackageShare("cooperative_parking_robot"),
        "config", "id0_calibration.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_device", default_value="",
            description=(
                "Stable /dev/v4l/by-path/...-video-index0 path. "
                "Empty uses camera_id.")),
        DeclareLaunchArgument("camera_id", default_value="0"),
        DeclareLaunchArgument(
            "camera_calib",
            default_value=str(
                Path.home() / "ov2710_calib_23mm_white.npz"),
            description="robot-1 white OV2710 intrinsic calibration"),
        DeclareLaunchArgument("image_topic", default_value="/rear/marker_camera/image"),
        DeclareLaunchArgument("width", default_value="1280"),
        DeclareLaunchArgument("height", default_value="720"),
        DeclareLaunchArgument("fps", default_value="8.0"),
        DeclareLaunchArgument("marker_id", default_value="0"),
        DeclareLaunchArgument(
            "marker_size_m", default_value="0.10",
            description="Front rear-face ID0 black-square side length"),
        DeclareLaunchArgument("yaw_offset_deg", default_value="0.0"),
        DeclareLaunchArgument("yaw_sign", default_value="1.0"),
        DeclareLaunchArgument("gray_gain", default_value="1.0"),
        DeclareLaunchArgument(
            "aruco_min_marker_distance_rate", default_value="0.02",
            description="Keep ID0 separate from its close mounting-board edge"),
        DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("web_port", default_value="5005"),

        Node(
            package="cooperative_parking_robot",
            executable="opencv_camera",
            name="rear_marker_camera_node",
            parameters=[{
                "camera_device": LaunchConfiguration("camera_device"),
                "camera_id": _int("camera_id"),
                "output_topic": LaunchConfiguration("image_topic"),
                "frame_id": "rear_marker_camera",
                "width": _int("width"),
                "height": _int("height"),
                "fps": _float("fps"),
                "buffer_size": 1,
                "require_camera": True,
            }],
            output="screen"),

        Node(
            package="cooperative_parking_robot",
            executable="aruco_tracker",
            name="aruco_tracker_node",
            parameters=[id0_calibration, {
                "image_topic": LaunchConfiguration("image_topic"),
                "marker_id": _int("marker_id"),
                "marker_size_m": _float("marker_size_m"),
                "camera_calib": LaunchConfiguration("camera_calib"),
                "aruco_dict": "DICT_4X4_50",
                "yaw_offset_deg": _float("yaw_offset_deg"),
                "yaw_sign": _float("yaw_sign"),
                "gray_gain": _float("gray_gain"),
                "min_marker_distance_rate": _float(
                    "aruco_min_marker_distance_rate"),
                "allow_uncalibrated": False,
            }],
            output="screen"),

        Node(
            package="cooperative_parking_robot",
            executable="camera_preview",
            name="rear_aruco_camera_preview",
            parameters=[{
                "image_topics_csv": LaunchConfiguration("image_topic"),
                "labels_csv": "robot-1 white OV2710 / Front ID0",
                "web_host": LaunchConfiguration("web_host"),
                "web_port": _int("web_port"),
                "jpeg_quality": 80,
                "grid_step_px": 50,
                "calibration_width_px": _int("width"),
                "calibration_height_px": _int("height"),
                "stale_after_s": 1.0,
                "enable_aruco": True,
                "aruco_dict": "DICT_4X4_50",
                "marker_size_m": _float("marker_size_m"),
                "aruco_min_marker_distance_rate": _float(
                    "aruco_min_marker_distance_rate"),
                # Keep the real tracker at 8 Hz; the diagnostic overlay only
                # needs 4 Hz and otherwise duplicates the full 720p workload.
                "aruco_every_n": 2,
                "relative_pose_topic": "/sync/relative_pose",
                "marker_visible_topic": "/sync/marker_visible",
                "enable_bev": False,
                "enable_yolo": False,
            }],
            output="screen"),
    ])
