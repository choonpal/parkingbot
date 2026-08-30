"""Regression tests for distributed startup and stationary ID0 yaw checks."""

import math
from pathlib import Path
import runpy

import id0_yaw_preflight as yaw_gate
import production_preflight as production_gate


VALID_SHA = "a" * 40
OTHER_SHA = "b" * 40


def test_matching_revisions_pass():
    heads = {role: VALID_SHA for role in production_gate.ROLES}
    assert production_gate.evaluate_revisions(VALID_SHA, heads) == []


def test_any_revision_mismatch_blocks_start():
    heads = {role: VALID_SHA for role in production_gate.ROLES}
    heads["rear"] = OTHER_SHA
    errors = production_gate.evaluate_revisions(VALID_SHA, heads)
    assert len(errors) == 1
    assert "rear" in errors[0]
    assert "differs" in errors[0]


def test_missing_revision_blocks_start():
    heads = {role: VALID_SHA for role in production_gate.ROLES}
    heads["front"] = ""
    errors = production_gate.evaluate_revisions(VALID_SHA, heads)
    assert "front: git HEAD unavailable" in errors


def test_package_only_workspace_revision_marker_is_supported(tmp_path):
    (tmp_path / ".parkingbot_revision").write_text(VALID_SHA + "\n")
    assert production_gate.local_workspace_revision(tmp_path) == VALID_SHA


def test_circular_wrap_is_not_reported_as_yaw_jump():
    samples = [
        math.radians(value)
        for value in (179.0, -179.0, 178.5, -178.5) * 8]
    result = yaw_gate.evaluate_yaw_stability(
        samples, visibility_count=40, visible_count=40,
        min_samples=30, min_visible_ratio=0.8,
        max_std_deg=2.0, max_deviation_deg=5.0, max_step_deg=5.0)
    assert result.passed
    assert result.max_deviation_deg < 2.0


def test_known_stationary_yaw_spike_is_blocked():
    samples = [0.0] * 30 + [math.radians(8.65)]
    result = yaw_gate.evaluate_yaw_stability(
        samples, visibility_count=40, visible_count=40,
        min_samples=30, min_visible_ratio=0.8,
        max_std_deg=2.0, max_deviation_deg=5.0, max_step_deg=5.0)
    assert not result.passed
    assert result.reason in {
        "YAW_STD_EXCEEDED", "YAW_DEVIATION_EXCEEDED",
        "YAW_STEP_EXCEEDED"}


def test_visibility_and_sample_count_fail_closed():
    result = yaw_gate.evaluate_yaw_stability(
        [], visibility_count=0, visible_count=0,
        min_samples=30, min_visible_ratio=0.8,
        max_std_deg=2.0, max_deviation_deg=5.0, max_step_deg=5.0)
    assert not result.passed
    assert result.reason == "NO_MARKER_VISIBILITY_MESSAGES"


def test_rear_launch_defaults_match_latest_verified_1280x720_runtime():
    root = Path(__file__).resolve().parents[1]
    launch = (root / "ros2/cooperative_parking_robot/launch/"
              "rear_robot.launch.py").read_text()
    assert '"rear_camera_width", default_value="1280"' in launch
    assert '"rear_camera_height", default_value="720"' in launch
    assert '"rear_camera_fps", default_value="8.0"' in launch


def test_installer_deploys_guard_core_and_explicit_id0_command():
    root = Path(__file__).resolve().parents[1]
    installer = (root / "tools/install_robot_commands.sh").read_text()
    assert '"$tool_dir/robotctl_core"' in installer
    assert '"$tool_dir/production_preflight.py"' in installer
    assert '"$tool_dir/id0_yaw_preflight.py"' in installer
    assert "doctor id0_check" in installer


def test_robotctl_guard_preserves_core_api_and_id0_command():
    namespace = runpy.run_path(str(Path(__file__).with_name("robotctl")))
    assert callable(namespace["stack_launch_command"])
    assert callable(namespace["report_startup_progress"])
    assert namespace["selected_command"](
        ["--config", "/tmp/site.env", "start"]) == "start"
    assert namespace["selected_command"](["id0-check"]) == "id0-check"
    assert {"id0-check", "id0_check"} == namespace["ID0_COMMANDS"]
