#!/usr/bin/env python3
"""ROS 2 Humble / Rear Raspberry Pi real-robot launch."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _float(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _int(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def _bool(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def generate_launch_description():
    wheelbase = _float("wheelbase")
    waiting_x = _float("waiting_x")
    waiting_y = _float("waiting_y")
    home_yaw_deg = _float("home_yaw_deg")
    home_yaw_rad = ParameterValue(
        PythonExpression([
            "float(", LaunchConfiguration("home_yaw_deg"),
            ") * 0.017453292519943295",
        ]),
        value_type=float)
    wheel_radius = _float("wheel_radius")
    encoder_ppr = _float("encoder_ppr")
    lx = _float("lx")
    ly = _float("ly")
    marker_size = _float("marker_size_m")
    yaw_offset = _float("yaw_offset_deg")
    yaw_sign = _float("yaw_sign")
    gray_gain = _float("gray_gain")
    aruco_min_marker_distance_rate = _float(
        "aruco_min_marker_distance_rate")
    allow_uncalibrated = _bool("allow_uncalibrated")
    enable_serial = _bool("enable_serial")
    require_serial = _bool("require_serial")
    require_hardware_ready = _bool("require_hardware_ready")
    require_ultrasonic = _bool("require_ultrasonic_for_ready")
    ultrasonic_timeout = _float("ultrasonic_frame_timeout_s")
    ultrasonic_threshold = _float("ultrasonic_threshold_m")
    ultrasonic_hysteresis = _float("ultrasonic_exit_hysteresis_m")
    ultrasonic_yaw_limit = _float("max_sensor_yaw_error_deg")
    left_sensor_offset = _float("left_sensor_to_gripper_x_m")
    right_sensor_offset = _float("right_sensor_to_gripper_x_m")
    enable_aruco = LaunchConfiguration("enable_aruco_tracker")
    # P0-1: aruco_tracker_node가 구독하는 rear 전면 카메라 토픽에는 v1.9까지
    # 발행자가 없었다. 외부 카메라 드라이버가 있으면 enable_rear_camera:=false로
    # 두고, 없으면 이 패키지의 opencv_camera_node가 유일한 발행자가 된다.
    enable_rear_camera = LaunchConfiguration("enable_rear_camera")
    id0_calibration = PathJoinSubstitution([
        FindPackageShare("cooperative_parking_robot"),
        "config", "id0_calibration.yaml"])

    entry_parameters = {
        "vehicle_half_length_m": _float("vehicle_half_length_m"),
        "vehicle_half_width_m": _float("vehicle_half_width_m"),
        "robot_length_m": _float("robot_length_m"),
        "robot_width_m": _float("robot_width_m"),
        "minimum_inter_robot_gap_m": _float(
            "minimum_inter_robot_gap_m"),
        "robot_clearance_m": _float("robot_clearance_m"),
        "entry_standoff_m": _float("entry_standoff_m"),
        "entry_side_offset_m": _float("entry_side_offset_m"),
        "entry_side": _int("entry_side"),
        "exit_distance_m": _float("exit_distance_m"),
        "simultaneous_entry": _bool("simultaneous_entry"),
        "same_direction_exit": _bool("same_direction_exit"),
        "same_direction_exit_sign": _int("same_direction_exit_sign"),
        "exit_sync_gain": _float("exit_sync_gain"),
        "scan_overshoot_m": _float("scan_overshoot_m"),
        "prealign_hold_n": _int("prealign_hold_n"),
        "use_ultrasonic_lateral": _bool("use_ultrasonic_lateral"),
        "ultrasonic_lateral_timeout_s": _float(
            "ultrasonic_lateral_timeout_s"),
        "ultrasonic_lateral_yaw_gate_deg": _float(
            "ultrasonic_lateral_yaw_gate_deg"),
        "lateral_deviation_limit_m": _float(
            "lateral_deviation_limit_m"),
        "lateral_deviation_n": _int("lateral_deviation_n"),
        "max_scan_retry": _int("max_scan_retry"),
        "substate_timeout_s": _float("substate_timeout_s"),
        "target_timeout_s": _float("target_timeout_s"),
        "cctv_marker_timeout_s": _float("cctv_marker_timeout_s"),
        "relative_lateral_tolerance_m": _float(
            "relative_lateral_tolerance_m"),
        "home_yaw_deg": home_yaw_deg,
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument(
            "hardware_profile", default_value="robot-1",
            description="Physical chassis profile; independent of rear role"),
        DeclareLaunchArgument("enable_serial", default_value="true"),
        DeclareLaunchArgument("require_serial", default_value="true"),
        DeclareLaunchArgument("serial_write_timeout_s", default_value="0.05"),
        DeclareLaunchArgument("velocity_tx_rate_hz", default_value="20.0"),
        DeclareLaunchArgument(
            "require_hardware_ready", default_value="true"),
        DeclareLaunchArgument(
            "require_ultrasonic_for_ready", default_value="false"),
        DeclareLaunchArgument(
            "ultrasonic_frame_timeout_s", default_value="0.50"),
        DeclareLaunchArgument(
            "ultrasonic_threshold_m", default_value="0.10"),
        DeclareLaunchArgument(
            "ultrasonic_exit_hysteresis_m", default_value="0.02"),
        DeclareLaunchArgument(
            "max_sensor_yaw_error_deg", default_value="10.0"),
        DeclareLaunchArgument(
            "left_sensor_to_gripper_x_m", default_value="0.0"),
        DeclareLaunchArgument(
            "right_sensor_to_gripper_x_m", default_value="0.0"),
        DeclareLaunchArgument(
            "enable_aruco_tracker", default_value="true"),
        DeclareLaunchArgument(
            "rear_camera_topic",
            default_value="/rear/marker_camera/image"),
        DeclareLaunchArgument(
            "enable_rear_camera", default_value="true",
            description="이 패키지의 opencv_camera로 rear 전면 카메라를 연다. "
                        "외부 ROS 카메라 드라이버를 쓰면 false"),
        DeclareLaunchArgument(
            "rear_camera_id", default_value="0",
            description="rear 전면 USB 카메라 V4L2 index"),
        DeclareLaunchArgument(
            "rear_camera_device", default_value="",
            description="권장: rear 카메라 /dev/v4l/by-id persistent path"),
        DeclareLaunchArgument(
            "rear_camera_gst", default_value="",
            description="비우면 V4L2, 채우면 GStreamer 파이프라인 사용"),
        DeclareLaunchArgument("rear_camera_width", default_value="1280"),
        DeclareLaunchArgument("rear_camera_height", default_value="720"),
        DeclareLaunchArgument("rear_camera_fps", default_value="8.0"),
        DeclareLaunchArgument(
            "rear_camera_capture_fps", default_value="30.0",
            description="V4L2 acquisition rate; ROS output stays rear_camera_fps"),
        DeclareLaunchArgument(
            "rear_camera_fourcc", default_value="MJPG",
            description="Rear V4L2 pixel format (empty leaves backend default)"),
        DeclareLaunchArgument("rear_camera_standby_fps", default_value="1.0"),
        DeclareLaunchArgument(
            "rear_camera_activation_drop_frames", default_value="2"),
        DeclareLaunchArgument(
            "camera_calib",
            default_value=str(
                Path.home() / "ov2710_calib_23mm_white.npz")),
        DeclareLaunchArgument(
            "allow_uncalibrated", default_value="false"),
        DeclareLaunchArgument(
            "marker_size_m", default_value="0.10",
            description="Front rear-face ID0 black-square side length"),
        DeclareLaunchArgument("yaw_offset_deg", default_value="0.0"),
        DeclareLaunchArgument("yaw_sign", default_value="1.0"),
        DeclareLaunchArgument("gray_gain", default_value="1.0"),
        DeclareLaunchArgument(
            "aruco_min_marker_distance_rate", default_value="0.02"),
        DeclareLaunchArgument("wheelbase", default_value="0.785"),
        DeclareLaunchArgument(
            "vehicle_half_length_m", default_value="0.45"),
        DeclareLaunchArgument(
            "vehicle_half_width_m", default_value="0.175"),
        DeclareLaunchArgument("robot_length_m", default_value="0.565"),
        DeclareLaunchArgument("robot_width_m", default_value="0.420"),
        DeclareLaunchArgument(
            "minimum_inter_robot_gap_m", default_value="0.22"),
        DeclareLaunchArgument(
            "robot_clearance_m", default_value="0.06"),
        DeclareLaunchArgument(
            "entry_standoff_m", default_value="0.85"),
        DeclareLaunchArgument(
            "entry_side_offset_m", default_value="0.50"),
        DeclareLaunchArgument(
            "entry_side", default_value="-1"),
        DeclareLaunchArgument(
            "exit_distance_m", default_value="0.50"),
        DeclareLaunchArgument(
            "simultaneous_entry", default_value="false"),
        DeclareLaunchArgument(
            "stop_after_align", default_value="false",
            description="Hold after axle alignment and never commit LIFT"),
        DeclareLaunchArgument(
            "same_direction_exit", default_value="false"),
        DeclareLaunchArgument(
            "same_direction_exit_sign", default_value="1"),
        DeclareLaunchArgument(
            "exit_sync_gain", default_value="0.15"),
        DeclareLaunchArgument(
            "scan_overshoot_m", default_value="0.10"),
        DeclareLaunchArgument("prealign_hold_n", default_value="10"),
        DeclareLaunchArgument(
            "use_ultrasonic_lateral", default_value="true"),
        DeclareLaunchArgument(
            "ultrasonic_lateral_timeout_s", default_value="0.30"),
        DeclareLaunchArgument(
            "ultrasonic_lateral_yaw_gate_deg", default_value="10.0"),
        DeclareLaunchArgument(
            "lateral_deviation_limit_m", default_value="0.030"),
        DeclareLaunchArgument("lateral_deviation_n", default_value="5"),
        DeclareLaunchArgument("max_scan_retry", default_value="2"),
        DeclareLaunchArgument("lateral_median_n", default_value="3"),
        DeclareLaunchArgument(
            "lateral_pair_timeout_s", default_value="0.20"),
        DeclareLaunchArgument("lateral_sign", default_value="1.0"),
        DeclareLaunchArgument(
            "substate_timeout_s", default_value="60.0"),
        DeclareLaunchArgument(
            "target_timeout_s", default_value="2.0"),
        DeclareLaunchArgument("cctv_marker_timeout_s", default_value="0.50"),
        DeclareLaunchArgument(
            "relative_lateral_tolerance_m", default_value="0.03"),
        DeclareLaunchArgument(
            "axle_position_tolerance_m", default_value="0.15"),
        DeclareLaunchArgument(
            "approach_timeout_s", default_value="150.0"),
        DeclareLaunchArgument(
            "align_timeout_s", default_value="120.0"),
        DeclareLaunchArgument(
            "return_timeout_s", default_value="90.0"),
        DeclareLaunchArgument("waiting_x", default_value="3.60"),
        DeclareLaunchArgument("waiting_y", default_value="0.20"),
        DeclareLaunchArgument(
            "home_yaw_deg", default_value="180.0",
            description="Robot HOME heading in the map frame [deg]"),
        DeclareLaunchArgument("wheel_radius", default_value="0.05"),
        DeclareLaunchArgument("encoder_ppr", default_value="5182.0"),
        DeclareLaunchArgument("lx", default_value="0.10"),
        DeclareLaunchArgument("ly", default_value="0.10"),

        # P0-1: ID0 관측용 rear 전면 카메라 발행자.
        # 이 노드가 없으면 /sync/relative_pose가 영원히 발행되지 않아
        # WAIT_PEER_STAGED에서 ALIGN_TIMEOUT -> FAULT가 확정적으로 발생한다.
        # Pre-open the UVC device before the bridge session begins, but keep it
        # at a 1 Hz drain-only standby until Rear reaches READY_TO_SCAN.
        Node(
            package="cooperative_parking_robot",
            executable="opencv_camera",
            name="rear_marker_camera_node",
            condition=IfCondition(enable_rear_camera),
            parameters=[{
                "camera_id": _int("rear_camera_id"),
                "camera_device": LaunchConfiguration(
                    "rear_camera_device"),
                "gstreamer_pipeline": LaunchConfiguration("rear_camera_gst"),
                "output_topic": LaunchConfiguration("rear_camera_topic"),
                "frame_id": "rear_marker_camera",
                "width": _int("rear_camera_width"),
                "height": _int("rear_camera_height"),
                "fps": _float("rear_camera_fps"),
                "capture_fps": _float("rear_camera_capture_fps"),
                "v4l2_fourcc": LaunchConfiguration(
                    "rear_camera_fourcc"),
                "buffer_size": 1,
                "require_camera": True,
                "runtime_enable_topic": "/rear/relative_vision_enable",
                "runtime_ready_topic": "/rear/marker_camera_ready",
                "start_enabled": False,
                "standby_fps": _float("rear_camera_standby_fps"),
                "activation_drop_frames": _int(
                    "rear_camera_activation_drop_frames"),
            }],
            output="screen"),

        # Complete the heavy OpenCV/ArUco imports before opening the 300 ms
        # heartbeat session. Disabled ArUco does no per-frame CV work.
        TimerAction(period=3.0, actions=[Node(
            package="cooperative_parking_robot",
            executable="aruco_tracker",
            name="aruco_tracker_node",
            condition=IfCondition(enable_aruco),
            parameters=[id0_calibration, {
                "image_topic": LaunchConfiguration("rear_camera_topic"),
                "marker_id": 0,
                "marker_size_m": marker_size,
                "camera_calib": LaunchConfiguration("camera_calib"),
                "yaw_offset_deg": yaw_offset,
                "yaw_sign": yaw_sign,
                "gray_gain": gray_gain,
                "min_marker_distance_rate": aruco_min_marker_distance_rate,
                "allow_uncalibrated": allow_uncalibrated,
                "runtime_enable_topic": "/rear/relative_vision_enable",
                "runtime_ready_topic": "/rear/relative_vision_ready",
                "start_enabled": False,
            }],
            output="screen")]),

        TimerAction(period=16.0, actions=[Node(
            package="cooperative_parking_robot",
            executable="individual_move",
            name="rear_individual_move",
            parameters=[id0_calibration, {
                "role": "rear",
                "default_wheelbase": wheelbase,
                "use_vehicle_spec_wheelbase": True,
                "waiting_x": waiting_x,
                "waiting_y": waiting_y,
                "center_tolerance_m": 0.01,
                **entry_parameters,
            }],
            output="screen")]),

        TimerAction(period=10.0, actions=[Node(
            package="cooperative_parking_robot",
            executable="state_machine",
            name="rear_state_machine",
            parameters=[{
                "role": "rear",
                "approach_timeout_s": _float("approach_timeout_s"),
                "align_timeout_s": _float("align_timeout_s"),
                "drive_timeout_s": 120.0,
                "return_timeout_s": _float("return_timeout_s"),
                "require_hardware_ready": require_hardware_ready,
                "stop_after_align": _bool("stop_after_align"),
            }],
            output="screen")]),

        TimerAction(period=8.0, actions=[Node(
            package="cooperative_parking_robot",
            executable="stm32_bridge",
            name="rear_bridge",
            parameters=[{
                "role": "rear",
                "hardware_profile": LaunchConfiguration("hardware_profile"),
                "serial_port": LaunchConfiguration("serial_port"),
                "serial_baud": 115200,
                "enable_serial": enable_serial,
                "require_serial": require_serial,
                "wheel_radius": wheel_radius,
                "encoder_ppr": encoder_ppr,
                "lx": lx,
                "ly": ly,
                "ultrasonic_frame_timeout_s": ultrasonic_timeout,
                "require_ultrasonic_for_ready": require_ultrasonic,
                "serial_write_timeout_s": _float(
                    "serial_write_timeout_s"),
                "velocity_tx_rate_hz": _float("velocity_tx_rate_hz"),
            }],
            output="screen")]),

        TimerAction(period=12.0, actions=[Node(
            package="cooperative_parking_robot",
            executable="ultrasonic_edge",
            name="rear_ultrasonic",
            parameters=[{
                "role": "rear",
                "threshold_m": ultrasonic_threshold,
                "exit_hysteresis_m": ultrasonic_hysteresis,
                "sensor_timeout_s": ultrasonic_timeout,
                "max_sensor_yaw_error_deg": ultrasonic_yaw_limit,
                "left_sensor_to_gripper_x_m": left_sensor_offset,
                "right_sensor_to_gripper_x_m": right_sensor_offset,
                "lateral_median_n": _int("lateral_median_n"),
                "lateral_pair_timeout_s": _float(
                    "lateral_pair_timeout_s"),
                "lateral_sign": _float("lateral_sign"),
                "axle_position_tolerance_m": _float(
                    "axle_position_tolerance_m"),
            }],
            output="screen")]),

        TimerAction(period=14.0, actions=[Node(
            package="cooperative_parking_robot",
            executable="pose_fusion",
            name="rear_pose_fusion",
            parameters=[{
                "role": "rear",
                "init_x": waiting_x,
                "init_y": waiting_y,
                "init_yaw": home_yaw_rad,
            }],
            output="screen")]),
    ])
