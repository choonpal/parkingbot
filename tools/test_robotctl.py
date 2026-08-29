#!/usr/bin/env python3
"""ROS-independent regressions for field operation tooling."""

import json
import os
from pathlib import Path
import runpy
import shutil
import subprocess
import time

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
        command = argv[-1] if argv else ""
        if "parkingbot_ros_snapshot.py" in command:
            values = {key: None for key in ops.TOPICS}
            values.update({key: False for key in ops.PRESENCE_TOPICS})
            for key, topic in ops.TOPICS.items():
                value = self.topic_values.get(topic)
                values[key] = None if value is TimeoutError else value
            for key, topic in ops.PRESENCE_TOPICS.items():
                value = self.topic_values.get(topic)
                values[key] = value not in (None, TimeoutError, False)
            return Result(stdout=json.dumps({"topics": values}) + "\n")
        return Result(returncode=self.remote_code)


def valid_config():
    config = {key: "value" for key in ops.REQUIRED}
    config.update({"ROS_SETUP": "/opt/ros/humble/setup.bash",
                   "ROS_LOCALHOST_ONLY": "0",
                   "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
                   "REAR_ENABLE_INTERNAL_CAMERA": "false",
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
    data = ops.snapshot(valid_config(), runner=runner, timeout=0.01)
    assert data["overall"] == "NOT READY"
    assert "FLEET STATE UNAVAILABLE" in data["blockers"]
    assert len(runner.calls) == 1
    assert "parkingbot_ros_snapshot.py" in runner.calls[0][-1]
    assert data["observer"]["observer_ok"] is True


def test_observer_process_failures_are_not_reported_as_missing_topics():
    exited = ops.parse_observer_output(
        "partial output", returncode=1,
        stderr="ModuleNotFoundError: No module named 'rclpy'")
    timed_out = ops.parse_observer_output(
        "partial output", returncode=124, stderr="", timed_out=True)
    malformed = ops.parse_observer_output("not-json", returncode=0)

    assert exited["observer_ok"] is False
    assert exited["observer_error_type"] == "process_exit"
    assert exited["observer_returncode"] == 1
    assert "rclpy" in exited["observer_stderr"]
    assert timed_out["observer_error_type"] == "timeout"
    assert timed_out["observer_timed_out"] is True
    assert malformed["observer_error_type"] == "malformed_output"


def test_snapshot_preserves_observer_failure_in_status():
    observer = ops.parse_observer_output(
        "", returncode=1, stderr="rclpy import failed")
    data = ops.snapshot(valid_config(), observer_result=observer)
    assert data["observer"]["observer_ok"] is False
    assert data["blockers"][0].startswith("OBSERVER FAILURE:")
    assert "FAIL" in ops.format_snapshot(data)


def test_observer_prerequisites_report_missing_setup_and_helper(tmp_path):
    config = valid_config()
    config["ROS_SETUP"] = str(tmp_path / "missing-ros.bash")
    config["OBSERVER_PYTHON"] = "/usr/bin/python3"
    errors = ops.observer_prerequisite_errors(
        config, helper=tmp_path / "missing-helper.py")
    assert any("ROS setup missing" in error for error in errors)
    assert any("snapshot helper missing" in error for error in errors)


def test_observer_prerequisite_reports_python_import_or_rmw_failure(tmp_path):
    setup = tmp_path / "setup.bash"
    helper = tmp_path / "parkingbot_ros_snapshot.py"
    support = tmp_path / "parkingbot_ops.py"
    python = tmp_path / "python3"
    for path in (setup, helper, support, python):
        path.write_text("# test\n")
    python.chmod(0o755)
    config = valid_config()
    config.update({"ROS_SETUP": str(setup),
                   "OBSERVER_PYTHON": str(python)})
    runner = StaticRunner(Result(
        returncode=1,
        stderr="ModuleNotFoundError: No module named 'rclpy'"))
    errors = ops.observer_prerequisite_errors(
        config, runner=runner, helper=helper)
    assert len(errors) == 1
    assert "preflight failed rc=1" in errors[0]
    assert "rclpy" in errors[0]


def test_observer_environment_uses_underlay_without_control_overlay():
    config = valid_config()
    config.update({"ROS_SETUP": "/opt/ros/humble/setup.bash",
                   "CONTROL_WORKSPACE": "/srv/parkingbot_ws",
                   "OBSERVER_PYTHON": "/usr/bin/python3"})
    command = ops.observer_argv(
        config, [config["OBSERVER_PYTHON"], "snapshot.py"])[2]
    assert "source /opt/ros/humble/setup.bash" in command
    assert "/srv/parkingbot_ws/install/setup.bash" not in command
    assert "exec /usr/bin/python3 snapshot.py" in command


def test_startup_completion_ignores_non_startup_diagnostic_topics():
    received = set(ops.STARTUP_REQUIRED_TOPIC_KEYS)
    assert ops.observer_complete(received, "startup") is True
    assert ops.observer_complete(received, "full") is False
    received.remove("rear_hw")
    assert ops.observer_complete(received, "startup") is False


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
    data = ops.snapshot(valid_config(), FakeRunner(values))
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
    target = ops.incident_snapshot(
        run, "front robot fault", state, valid_config(), FakeRunner(),
        {"role": "front", "process_alive": True})
    assert (target / "state.json").exists()
    assert "fault" in (target / "front_tail.log").read_text()
    assert (target / "ros_topic_list.txt").exists()
    assert json.loads((target / "incident.json").read_text())["role"] == "front"


def test_launch_commands_use_documented_production_launches_and_no_force():
    config = valid_config()
    config.update({"ROS_SETUP": "/opt/ros/humble/setup.bash",
                   "ROS_LOCALHOST_ONLY": "0",
                   "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
                   "REAR_ENABLE_INTERNAL_CAMERA": "false",
                   "REAR_CAMERA_TOPIC": "/rear/marker_camera/image"})
    assert "cctv_server_dual.launch.py" in ops.launch_command(config, "jetson")
    jetson = ops.launch_command(config, "jetson")
    assert "camera_width_px:=640" in jetson
    assert "camera_height_px:=360" in jetson
    assert "calibration_width_px:=640" in jetson
    assert "calibration_height_px:=360" in jetson
    assert "image/jpeg,width=640,height=360,framerate=30/1" in jetson
    assert "front_robot.launch.py" in ops.launch_command(config, "front")
    assert "rear_robot.launch.py" in ops.launch_command(config, "rear")
    assert "--force" not in " ".join(
        ops.launch_command(config, role) for role in ops.ROLES)


def test_stop_after_align_is_forwarded_to_both_robot_launches():
    config = valid_config()
    config["STOP_AFTER_ALIGN"] = "true"

    assert "stop_after_align:=true" in ops.launch_command(config, "front")
    assert "stop_after_align:=true" in ops.launch_command(config, "rear")
    assert "stop_after_align" not in ops.launch_command(config, "jetson")


def test_internal_rear_camera_prefers_persistent_device_path():
    config = valid_config()
    config.update({
        "REAR_ENABLE_INTERNAL_CAMERA": "true",
        "REAR_CAMERA_ID": "0",
        "REAR_CAMERA_DEVICE": "/dev/v4l/by-id/rear-camera",
    })

    command = ops.launch_command(config, "rear")
    assert "rear_camera_id:=0" in command
    assert "rear_camera_device:=/dev/v4l/by-id/rear-camera" in command


def test_local_ros_commands_always_source_underlay_and_control_overlay():
    config = valid_config()
    config.update({
        "ROS_SETUP": "/opt/ros/humble/setup.bash",
        "CONTROL_WORKSPACE": "/srv/parkingbot_ws",
        "ROS_LOCALHOST_ONLY": "0",
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
    })
    argv = ops.local_ros_argv(config, ["ros2", "node", "list"])
    assert argv[:2] == ["bash", "-lc"]
    command = argv[2]
    assert "source /opt/ros/humble/setup.bash" in command
    assert "source /srv/parkingbot_ws/install/setup.bash" in command
    assert "export ROS_DOMAIN_ID=value" in command
    assert "exec ros2 node list" in command


def test_release_protocol_sources_are_consistent():
    repository = Path(__file__).resolve().parents[1]
    assert ops.EXPECTED_UART_PROTOCOL_VERSION == 2
    assert ops.EXPECTED_UART_BAUD_RATE == 115200
    assert ops.protocol_consistency_errors(repository) == []


def test_release_protocol_check_fails_closed_on_mixed_source(tmp_path):
    firmware = tmp_path / 'stm32/parking_robot/Core/Src'
    package = tmp_path / (
        'ros2/cooperative_parking_robot/cooperative_parking_robot')
    firmware.mkdir(parents=True)
    package.mkdir(parents=True)
    (firmware / 'parking_robot_firmware.c').write_text(
        '#define UART_PROTOCOL_VERSION 2U\n'
        '#define UART_BAUD_RATE 115200U\n"HELLO:%u:%s"\n'
        'protocol_session_active\n')
    (package / 'uart_protocol.py').write_text(
        'PROTOCOL_VERSION = 2\nUART_BAUD_RATE = 115200\n'
        'def encode_hello(): pass\n'
        'def encode_zero_velocity(): pass\n')
    (package / 'stm32_bridge_node.py').write_text(
        'hello_acknowledged = False\nzero_command_acknowledged = False\n')
    errors = ops.protocol_consistency_errors(tmp_path)
    assert errors
    assert any('bridge' in error and 'encode_hello' in error
               for error in errors)


def test_remote_protocol_doctor_checks_deployed_ros_sources():
    package = '/srv/parkingbot_ws/src/cooperative_parking_robot'
    command = ops.protocol_source_check_command(package)
    assert 'parking_robot_firmware.c' not in command
    assert 'uart_protocol.py' in command
    assert 'stm32_bridge_node.py' in command
    assert 'UART_PROTOCOL_VERSION' not in command
    assert 'UART_BAUD_RATE' in command
    assert package in command
    assert package + '/ros2/cooperative_parking_robot/' in command
    assert 'if test -f' in command


@pytest.mark.parametrize("arguments", (
    ["node", "list"],
    ["topic", "list", "-t"],
    ["topic", "echo", "--once", "/test"],
))
def test_ros_cli_helper_works_from_clean_noninteractive_environment(
        tmp_path, arguments):
    underlay = tmp_path / "underlay.bash"
    workspace = tmp_path / "ws"
    overlay = workspace / "install/setup.bash"
    binary_dir = tmp_path / "bin"
    overlay.parent.mkdir(parents=True)
    binary_dir.mkdir()
    fake_ros2 = binary_dir / "ros2"
    underlay.write_text("export PARKINGBOT_TEST_UNDERLAY=1\n")
    overlay.write_text(
        f"export PATH={binary_dir}:$PATH\n"
        "export PARKINGBOT_TEST_OVERLAY=1\n")
    fake_ros2.write_text(
        "#!/usr/bin/env bash\n"
        "test \"${PARKINGBOT_TEST_UNDERLAY:-}\" = 1\n"
        "test \"${PARKINGBOT_TEST_OVERLAY:-}\" = 1\n"
        "printf '%s\\n' \"$*\"\n")
    fake_ros2.chmod(0o755)
    config = valid_config()
    config.update({
        "ROS_SETUP": str(underlay),
        "CONTROL_WORKSPACE": str(workspace),
        "ROS_LOCALHOST_ONLY": "0",
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
    })
    clean_env = {"PATH": "/usr/bin:/bin"}
    result = subprocess.run(
        ops.local_ros_argv(config, ["ros2", *arguments]),
        text=True, capture_output=True, env=clean_env)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == " ".join(arguments)


class StaticRunner:
    def __init__(self, result):
        self.result = result

    def run(self, _argv, **_kwargs):
        return self.result


def test_process_probe_distinguishes_exit_from_ssh_failure_and_missing_session():
    running = ops.stack_process_status(
        StaticRunner(Result(stdout="123|0||bash\n")), "robot")
    exited = ops.stack_process_status(
        StaticRunner(Result(stdout="123|1|17|bash\n")), "robot")
    unreachable = ops.stack_process_status(
        StaticRunner(Result(returncode=255, stderr="ssh timeout")), "robot")
    missing = ops.stack_process_status(
        StaticRunner(Result(returncode=3, stdout="MISSING\n")), "robot")

    assert running["state"] == "RUNNING" and running["process_alive"] is True
    assert exited["state"] == "EXITED" and exited["returncode"] == 17
    assert unreachable["state"] == "UNKNOWN"
    assert unreachable["process_alive"] is None
    assert missing["state"] == "SESSION_MISSING"


def test_tmux_launch_retains_exit_status_and_startup_reports_each_gate():
    robotctl = runpy.run_path(str(Path(__file__).with_name("robotctl")))
    command = robotctl["stack_launch_command"](
        "source /opt/ros/humble/setup.bash", "ros2 launch pkg file.py",
        "$HOME/logs/run/front", "front.log")
    assert "remain-on-exit on" in command
    assert "respawn-pane" in command
    assert "PIPESTATUS[0]" in command
    assert "stack_exit.env" in command

    values = {key: None for key in ops.TOPICS}
    values.update({key: False for key in ops.PRESENCE_TOPICS})
    values.update({
        "fleet": {"state": "WAIT_TARGET"},
        "front_state": "IDLE", "rear_state": "IDLE",
        "front_hw": True, "rear_hw": False,
        "merge": {}, "id0_marker": False,
    })
    _, conditions = robotctl["report_startup_progress"](
        {"topics": values}, {})
    assert conditions["Front hardware_ready"] is True
    assert conditions["Rear hardware_ready"] is False

    metadata = robotctl["incident_metadata"](
        "front", "FRONT HARDWARE NOT READY", {"timestamp": "now",
        "topics": {"front_hw": False, "front_state": "IDLE",
                   "front_fault": ""}},
        {"state": "RUNNING", "pid": 123, "process_alive": True,
         "returncode": None, "launch_command": "ros2 launch ..."})
    assert metadata["reason"] == "FRONT HARDWARE NOT READY"
    assert metadata["process_alive"] is True
    assert metadata["returncode"] is None


def test_startup_detects_process_that_exits_during_graph_wait():
    robotctl = runpy.run_path(str(Path(__file__).with_name("robotctl")))
    processes = {
        "jetson": {"state": "RUNNING"},
        "rear": {"state": "EXITED", "returncode": 1},
        "front": {"state": "RUNNING"},
    }
    role, status = robotctl["startup_process_failure"](processes)
    assert role == "rear"
    assert status["returncode"] == 1


def test_robot_restart_stops_without_publishing_a_new_estop():
    robotctl = runpy.run_path(str(Path(__file__).with_name("robotctl")))
    calls = []

    def fake_stop(args):
        calls.append(("stop", args.no_estop))
        return 0

    def fake_start(_args):
        calls.append(("start", None))
        return 0

    restart = robotctl["restart"]
    restart.__globals__["stop"] = fake_stop
    restart.__globals__["start"] = fake_start
    args = type("Args", (), {})()
    assert restart(args) == 0
    assert calls == [("stop", True), ("start", None)]


@pytest.mark.skipif(shutil.which("tmux") is None, reason="tmux unavailable")
def test_detached_stack_survives_starting_shell_return(tmp_path):
    robotctl = runpy.run_path(str(Path(__file__).with_name("robotctl")))
    function = robotctl["stack_launch_command"]
    session = f"parkingbot-lifecycle-test-{os.getpid()}"
    function.__globals__["SESSION"] = session
    command = function(
        "true", "bash -c 'sleep 1; exit 7'", str(tmp_path), "stack.log")
    started = time.monotonic()
    try:
        result = subprocess.run(
            ["bash", "-lc", command], text=True, capture_output=True,
            timeout=2.0)
        if (result.returncode != 0 and
                "Operation not permitted" in result.stderr):
            pytest.skip("sandbox does not permit access to the tmux socket")
        assert result.returncode == 0, result.stderr
        assert time.monotonic() - started < 2.0
        probe = subprocess.run([
            "tmux", "display-message", "-p", "-t", f"{session}:stack.0",
            "#{pane_dead}|#{pane_pid}"], text=True, capture_output=True)
        assert probe.returncode == 0, probe.stderr
        dead, pid = probe.stdout.strip().split("|")
        assert dead == "0"
        assert int(pid) > 1

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            probe = subprocess.run([
                "tmux", "display-message", "-p", "-t",
                f"{session}:stack.0",
                "#{pane_dead}|#{pane_dead_status}"],
                text=True, capture_output=True)
            if probe.stdout.strip().startswith("1|"):
                break
            time.sleep(0.05)
        assert probe.stdout.strip() == "1|7"
    finally:
        subprocess.run(
            ["tmux", "kill-session", "-t", session], capture_output=True)
