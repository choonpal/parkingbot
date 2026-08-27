#!/usr/bin/env python3
"""ROS-independent regressions for field operation tooling."""

import json
from pathlib import Path
import subprocess

import pytest

import parkingbot_ops as ops


class Result:
    def __init__(self, returncode=0, stdout="", stderr=""):
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


class FakeRunner:
    def __init__(self, topic_values=None, remote_code=0):
        self.topic_values = topic_values or {}
        self.remote_code = remote_code
        self.calls = []

    def run(self, argv, **kwargs):
        self.calls.append(argv)
        if argv and argv[0] == "ros2" and argv[-1] in self.topic_values:
            value = self.topic_values[argv[-1]]
            if value is TimeoutError:
                raise subprocess.TimeoutExpired(argv, kwargs.get("timeout", 1))
            text = json.dumps(value) if isinstance(value, (dict, bool)) else str(value)
            return Result(stdout=text + "\n")
        return Result(returncode=self.remote_code)


def valid_config():
    config = {key: "value" for key in ops.REQUIRED}
    config.update({"REAR_ENABLE_INTERNAL_CAMERA": "false",
                   "REAR_EXTERNAL_CAMERA_COMMAND": "ros2 run camera driver",
                   "REAR_CAMERA_TOPIC": "/rear/marker_camera/image"})
    return config


def test_config_reports_every_missing_required_value(tmp_path):
    path = tmp_path / "site.env"
    path.write_text("ROS_DOMAIN_ID=42\n")
    missing = ops.missing_config(ops.load_env(path))
    assert "JETSON_HOST" in missing
    assert "FRONT_SERIAL" in missing


def test_missing_host_is_a_hard_configuration_failure():
    config = valid_config()
    config["REAR_HOST"] = ""
    assert "REAR_HOST" in ops.missing_config(config)


def test_remote_workspace_paths_must_be_absolute():
    config = valid_config()
    config["FRONT_WORKSPACE"] = "~/parkingbot_ws"
    assert "FRONT_WORKSPACE" in ops.invalid_paths(config)


def test_external_rear_camera_requires_an_authoritative_start_command():
    config = valid_config()
    config["REAR_EXTERNAL_CAMERA_COMMAND"] = ""
    assert ops.conditional_config_errors(config)


def test_remote_ssh_is_batch_and_fail_fast():
    argv = ops.remote_argv("rear.example", "true")
    assert argv[:2] == ["ssh", "-o"]
    assert "BatchMode=yes" in argv
    assert "ConnectTimeout=5" in argv


def test_duplicate_session_detection_contract():
    runner = FakeRunner(remote_code=0)
    result = ops.remote_run(runner, "rear.example", "tmux has-session")
    assert result.returncode == 0


def test_log_directory_and_latest_link(tmp_path):
    run = ops.create_run_dir(tmp_path, "20260827_200000")
    assert (run / "incidents").is_dir()
    assert (tmp_path / "latest").resolve() == run


def test_missing_topics_timeout_is_bounded_and_becomes_blocker():
    runner = FakeRunner({
        topic: TimeoutError for topic in
        (*ops.TOPICS.values(), *ops.PRESENCE_TOPICS.values())})
    data = ops.snapshot(runner=runner, timeout=0.01)
    assert data["overall"] == "NOT READY"
    assert "FLEET STATE UNAVAILABLE" in data["blockers"]


def test_status_format_uses_real_safety_fields():
    values = {
        "/fleet/state": {"state": "WAIT_TARGET", "mission_id": "",
                          "empty_count": 3, "has_map": True,
                          "vehicle_spec_ready": True},
        "/parking/target_ready": True,
        "/parking/vehicle_spec": {"dimension_valid": True},
        "/front/robot_state": "IDLE", "/rear/robot_state": "IDLE",
        "/front/hardware_ready": True, "/rear/hardware_ready": True,
        "/front/odom": "header: ok", "/rear/odom": "header: ok",
        "/parking/map": "header: ok",
        "/front/motion_fault": "", "/rear/motion_fault": "",
        "/front/localization_status": {"state": "OK"},
        "/rear/localization_status": {"state": "OK"},
        "/front/cctv_marker_visible": True,
        "/rear/cctv_marker_visible": True,
        "/sync/marker_visible": True,
        "/sync/error_state": {"error": "OK", "reference_state": "REFERENCE_READY"},
        "/cctv/merge_status": {
            "cameras": {"cam0": {"alive": True}, "cam2": {"alive": True}}},
    }
    data = ops.snapshot(FakeRunner(values))
    text = ops.format_snapshot(data)
    assert "Fleet        WAIT_TARGET" in text
    assert "Vehicle Spec VALID" in text
    assert "Sync Error   OK" in text


def test_incident_snapshot_contains_state_and_log_tails(tmp_path):
    run = ops.create_run_dir(tmp_path, "20260827_200001")
    (run / "front" / "front_robot.log").write_text("before\nfault\n")
    state = {"timestamp": "now", "topics": {
                 key: None for key in (*ops.TOPICS, *ops.PRESENCE_TOPICS)},
             "fleet_state": "FAULT", "mission_id": "m1", "empty_slots": 0,
             "blockers": ["FRONT ROBOT FAULT"], "overall": "NOT READY"}
    target = ops.incident_snapshot(run, "front robot fault", state, FakeRunner())
    assert (target / "state.json").exists()
    assert "fault" in (target / "front_tail.log").read_text()
    assert (target / "ros_topic_list.txt").exists()


def test_launch_commands_use_documented_production_launches_and_no_force():
    config = valid_config()
    config.update({"ROS_SETUP": "/opt/ros/humble/setup.bash",
                   "ROS_LOCALHOST_ONLY": "0",
                   "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
                   "REAR_ENABLE_INTERNAL_CAMERA": "false",
                   "REAR_CAMERA_TOPIC": "/rear/marker_camera/image"})
    assert "cctv_server_dual.launch.py" in ops.launch_command(config, "jetson")
    assert "front_robot.launch.py" in ops.launch_command(config, "front")
    assert "rear_robot.launch.py" in ops.launch_command(config, "rear")
    assert "--force" not in " ".join(
        ops.launch_command(config, role) for role in ops.ROLES)
