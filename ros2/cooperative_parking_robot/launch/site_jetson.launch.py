#!/usr/bin/env python3
"""One-line Jetson production launch using the shared site config."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from cooperative_parking_robot.site_config import (
    DEFAULT_SITE_CONFIG, load_site_config, require_site_keys,
)


CAMERA_WIDTH_PX = 640
CAMERA_HEIGHT_PX = 360
CAMERA_FPS = 30


def _mjpeg_pipeline(device: str) -> str:
    """Use the exact pixel frame used by the 2026-08-28 calibration."""
    return (
        f"v4l2src device={device} io-mode=2 ! "
        f"image/jpeg,width={CAMERA_WIDTH_PX},height={CAMERA_HEIGHT_PX},"
        f"framerate={CAMERA_FPS}/1 ! jpegdec ! videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )


def generate_launch_description():
    config = load_site_config(DEFAULT_SITE_CONFIG)
    require_site_keys(config, (
        "CAM0_DEVICE", "CAM2_DEVICE", "MODEL_PATH",
        "CAM0_GROUND_X_M", "CAM0_GROUND_Y_M", "CAM0_HEIGHT_M",
        "CAM2_GROUND_X_M", "CAM2_GROUND_Y_M", "CAM2_HEIGHT_M",
        "FRONT_MARKER_HEIGHT_M", "REAR_MARKER_HEIGHT_M",
    ), "Jetson")

    runtime = Path.home() / ".ros" / "adaptive_valet_bot"
    args = {
        "enable_opencv_camera": "true",
        "camera0_device": config["CAM0_DEVICE"],
        "camera2_device": config["CAM2_DEVICE"],
        "camera_width_px": str(CAMERA_WIDTH_PX),
        "camera_height_px": str(CAMERA_HEIGHT_PX),
        "camera_fps": str(CAMERA_FPS),
        "camera0_gstreamer_pipeline": _mjpeg_pipeline(
            config["CAM0_DEVICE"]),
        "camera2_gstreamer_pipeline": _mjpeg_pipeline(
            config["CAM2_DEVICE"]),
        "cctv0_camera_calib": str(runtime / "cctv0_camera_calibration.npz"),
        "cctv2_camera_calib": str(runtime / "cctv2_camera_calibration.npz"),
        "calibration_width_px": str(CAMERA_WIDTH_PX),
        "calibration_height_px": str(CAMERA_HEIGHT_PX),
        "homography_cam0_file": str(runtime / "homography_cam0_rectified.npy"),
        "homography_cam2_file": str(runtime / "homography_cam2_rectified.npy"),
        "layout_config": str(runtime / "parking_layout.yaml"),
        "parking_registry_db_path": str(runtime / "parking_registry.db"),
        "model_path": str(Path(config["MODEL_PATH"]).expanduser()),
        "cam0_ground_x_m": config["CAM0_GROUND_X_M"],
        "cam0_ground_y_m": config["CAM0_GROUND_Y_M"],
        "cam0_height_m": config["CAM0_HEIGHT_M"],
        "cam2_ground_x_m": config["CAM2_GROUND_X_M"],
        "cam2_ground_y_m": config["CAM2_GROUND_Y_M"],
        "cam2_height_m": config["CAM2_HEIGHT_M"],
        "camera_ground_points": "[" + ", ".join((
            config["CAM0_GROUND_X_M"], config["CAM0_GROUND_Y_M"],
            config["CAM2_GROUND_X_M"], config["CAM2_GROUND_Y_M"])) + "]",
        "front_marker_height_m": config["FRONT_MARKER_HEIGHT_M"],
        "rear_marker_height_m": config["REAR_MARKER_HEIGHT_M"],
        "enable_operator_ui": "true",
        "enable_debug_overlay": "false",
        "simultaneous_entry": "false",
        "require_all_cameras": "true",
        "camera_timeout_s": "2.0",
        "require_exact_camera_resolution": "true",
    }

    source = PathJoinSubstitution([
        FindPackageShare("cooperative_parking_robot"),
        "launch", "cctv_server_dual.launch.py"])
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(source),
            launch_arguments=args.items())
    ])
