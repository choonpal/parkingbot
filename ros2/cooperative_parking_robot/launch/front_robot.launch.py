#!/usr/bin/env python3
"""ROS 2 Humble / Front Raspberry Pi (Master) real-robot launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
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
    use_aruco_distance = _bool("use_aruco_distance")
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
    sync_config = PathJoinSubstitution([
        FindPackageShare("cooperative_parking_robot"),
        "config", "sync_params.yaml"])
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
            "serial_port", default_value="/dev/ttyACM0",
            description="Front STM32 UART; /dev/serial/by-id path recommended"),
        DeclareLaunchArgument(
            "hardware_profile", default_value="robot-2",
            description="Physical chassis profile; independent of front role"),
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
            "wheelbase", default_value="0.785",
            description="Measured target-vehicle wheelbase [m]"),
        DeclareLaunchArgument(
            "vehicle_half_length_m", default_value="0.45",
            description="Measured target body half-length [m]"),
        DeclareLaunchArgument(
            "vehicle_half_width_m", default_value="0.175",
            description="Measured target body half-width [m]"),
        DeclareLaunchArgument("robot_length_m", default_value="0.565"),
        DeclareLaunchArgument("robot_width_m", default_value="0.420"),
        DeclareLaunchArgument(
            "minimum_inter_robot_gap_m", default_value="0.22"),
        DeclareLaunchArgument(
            "robot_clearance_m", default_value="0.06",
            description="Robot envelope and route safety clearance [m]"),
        DeclareLaunchArgument(
            "entry_standoff_m", default_value="0.85",
            description="Center-to-entry-standoff distance [m]"),
        DeclareLaunchArgument(
            "entry_side_offset_m", default_value="0.50",
            description="Center-to-side-staging lane distance [m]"),
        DeclareLaunchArgument(
            "entry_side", default_value="-1",
            description="-1 vehicle right side, +1 vehicle left side"),
        DeclareLaunchArgument(
            "exit_distance_m", default_value="0.50"),
        DeclareLaunchArgument(
            "simultaneous_entry", default_value="false",
            description="Use the validated Front-first staging by default"),
        DeclareLaunchArgument(
            "stop_after_align", default_value="false",
            description="Hold after axle alignment and never commit LIFT"),
        DeclareLaunchArgument(
            "same_direction_exit", default_value="false",
            description="False keeps the validated split-nearest-end exit"),
        DeclareLaunchArgument(
            "same_direction_exit_sign", default_value="1",
            description="Shared exit direction when same_direction_exit=true"),
        DeclareLaunchArgument(
            "exit_sync_gain", default_value="0.15",
            description="Rear speed correction gain during shared exit"),
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
        DeclareLaunchArgument("waiting_y", default_value="0.60"),
        DeclareLaunchArgument(
            "home_yaw_deg", default_value="180.0",
            description="Robot HOME heading in the map frame [deg]"),
        DeclareLaunchArgument("wheel_radius", default_value="0.05"),
        DeclareLaunchArgument("encoder_ppr", default_value="5182.0"),
        DeclareLaunchArgument("lx", default_value="0.10"),
        DeclareLaunchArgument("ly", default_value="0.10"),
        DeclareLaunchArgument(
            "use_aruco_distance", default_value="true"),

        # Keep the serial safety path responsive while the remaining Python
        # processes import and join DDS.  The STM32 watchdog is 300 ms, so a
        # simultaneous cold start can otherwise starve the bridge long enough
        # to latch ESTOP before any mission is requested.
        TimerAction(period=20.0, actions=[Node(
            package="cooperative_parking_robot",
            executable="rigid_body_sync",
            name="rigid_body_sync_node",
            parameters=[id0_calibration, sync_config, {
                "wheelbase": wheelbase,
                "max_speed": 0.08,
                "max_omega": 0.30,
                "lookahead": 0.15,
                "hold_initial_yaw": True,
                # 일반 A* 구간은 yaw 유지, 슬롯 밖 staging에서만 슬롯 yaw 정렬.
                "align_to_slot_yaw": True,
                "final_approach_dist": 0.02,
                "use_vehicle_spec_wheelbase": True,
                "yaw_hold_kp": 1.0,
                "use_aruco_distance": use_aruco_distance,
                "cctv_marker_timeout_s": _float(
                    "cctv_marker_timeout_s"),
                "initialize_offset_from_target_pose": True,
                "initial_target_offset_gate_m": 0.50,
            }],
            output="screen")]),

        TimerAction(period=16.0, actions=[Node(
            package="cooperative_parking_robot",
            executable="individual_move",
            name="front_individual_move",
            parameters=[id0_calibration, {
                "role": "front",
                "default_wheelbase": wheelbase,
                "use_vehicle_spec_wheelbase": True,
                "waiting_x": waiting_x,
                "waiting_y": waiting_y,
                "center_tolerance_m": 0.01,
                **entry_parameters,
            }],
            output="screen")]),

        TimerAction(period=4.0, actions=[Node(
            package="cooperative_parking_robot",
            executable="state_machine",
            name="front_state_machine",
            parameters=[{
                "role": "front",
                "approach_timeout_s": _float("approach_timeout_s"),
                "align_timeout_s": _float("align_timeout_s"),
                "drive_timeout_s": 120.0,
                "return_timeout_s": _float("return_timeout_s"),
                "require_hardware_ready": require_hardware_ready,
                "stop_after_align": _bool("stop_after_align"),
            }],
            output="screen")]),

        Node(
            package="cooperative_parking_robot",
            executable="stm32_bridge",
            name="front_bridge",
            parameters=[{
                "role": "front",
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
            output="screen"),

        TimerAction(period=8.0, actions=[Node(
            package="cooperative_parking_robot",
            executable="ultrasonic_edge",
            name="front_ultrasonic",
            parameters=[{
                "role": "front",
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

        TimerAction(period=12.0, actions=[Node(
            package="cooperative_parking_robot",
            executable="pose_fusion",
            name="front_pose_fusion",
            parameters=[{
                "role": "front",
                "init_x": waiting_x,
                "init_y": waiting_y,
                "init_yaw": home_yaw_rad,
            }],
            output="screen")]),
    ])
