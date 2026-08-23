import math
from pathlib import Path

import pytest

from cooperative_parking_robot.individual_move_node import IndividualMoveNode
from cooperative_parking_robot.retrieval_planning import (
    sequential_routes_clear,
)
from cooperative_parking_robot.vehicle_entry import (
    approach_longitudinal,
    vehicle_to_world,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class _RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _returning_node(route_complete):
    node = IndividualMoveNode.__new__(IndividualMoveNode)
    node.phase = "RETURN_HOME"
    node.max_speed = 0.06
    node.home_yaw = math.pi
    node.route = [(3.60, 0.60)]
    node.return_sent = False
    node.pub_return_done = _RecordingPublisher()
    node.phase_timed_out = lambda: False
    node.stop = lambda: None
    node.transitions = []
    node.set_phase = node.transitions.append
    node.advance_calls = []

    def advance(speed, goal_yaw=None):
        node.advance_calls.append((speed, goal_yaw))
        return route_complete

    node.advance_route = advance
    return node


def test_return_home_tracks_configured_home_yaw_before_return_done():
    node = _returning_node(route_complete=False)

    node.run_return()

    assert node.advance_calls == [(0.06, pytest.approx(math.pi))]
    assert node.pub_return_done.messages == []
    assert node.transitions == []


def test_return_done_is_published_only_after_home_pose_is_complete():
    node = _returning_node(route_complete=True)

    node.run_return()

    assert node.advance_calls == [(0.06, pytest.approx(math.pi))]
    assert node.transitions == ["RETURNED"]
    assert len(node.pub_return_done.messages) == 1
    assert node.pub_return_done.messages[0].data is True


@pytest.mark.parametrize(
    ("simultaneous_entry", "expected_goal_yaw"),
    ((False, None), (True, math.pi / 2.0)),
)
def test_staging_translation_defers_yaw_only_for_front_first_entry(
        simultaneous_entry, expected_goal_yaw):
    node = IndividualMoveNode.__new__(IndividualMoveNode)
    node.phase = "TO_REAR_STAGING"
    node.active_target = (2.0, 2.2, math.pi / 2.0)
    node.centerline_speed = 0.035
    node.simultaneous_entry = simultaneous_entry
    node.phase_timed_out = lambda: False
    node.advance_calls = []

    def advance(speed, goal_yaw=None):
        node.advance_calls.append((speed, goal_yaw))
        return False

    node.advance_route = advance

    node.run_approach()

    assert node.advance_calls == [(0.035, expected_goal_yaw)]


def test_p2_front_first_clearance_requires_original_home_axis():
    # Gazebo evidence from the rejected retrieve request: RETURN_HOME had
    # reached the HOME area while both bodies still faced the slot axis.
    # Restoring the configured HOME axis removes that immediate overlap.
    front_route = ((3.598, 0.624), (1.991, 1.334))
    rear_route = ((3.586, 0.059), (1.991, 0.634))
    common = (front_route, rear_route, 0.035, 0.565, 0.275, 0.10)

    assert not sequential_routes_clear(
        *common,
        front_yaw_rad=math.pi / 2.0,
        rear_yaw_rad=math.pi / 2.0,
        front_goal_yaw_rad=math.pi / 2.0,
        rear_goal_yaw_rad=math.pi / 2.0,
    )
    assert sequential_routes_clear(
        *common,
        front_yaw_rad=math.pi,
        rear_yaw_rad=math.pi,
        front_goal_yaw_rad=math.pi / 2.0,
        rear_goal_yaw_rad=math.pi / 2.0,
    )


@pytest.mark.parametrize("slot_x", (1.20, 2.00, 2.80, 3.60))
def test_registered_demo_home_pose_allows_front_first_retrieve(slot_x):
    """The agreed demo HOME layout must admit sequential P1-P4 retrieval."""
    target_yaw = math.pi / 2.0
    front_goal = vehicle_to_world(
        approach_longitudinal("front", 0.85, 0.70),
        0.0, slot_x, 2.20, target_yaw)
    rear_goal = vehicle_to_world(
        approach_longitudinal("rear", 0.85, 0.70),
        0.0, slot_x, 2.20, target_yaw)

    assert sequential_routes_clear(
        ((3.60, 0.60), front_goal),
        ((3.60, 0.20), rear_goal),
        speed_mps=0.035,
        robot_length_m=0.565,
        robot_width_m=0.275,
        minimum_gap_m=0.10,
        front_yaw_rad=math.pi,
        rear_yaw_rad=math.pi,
        front_goal_yaw_rad=target_yaw,
        rear_goal_yaw_rad=target_yaw,
        yaw_gain=1.5,
        max_yaw_rate=0.15,
    ), slot_x


@pytest.mark.parametrize(
    "launch_name, home_x, home_y",
    (
        ("front_robot.launch.py", "3.60", "0.60"),
        ("rear_robot.launch.py", "3.60", "0.20"),
    ),
)
def test_real_robot_launch_uses_registered_home_pose(
        launch_name, home_x, home_y):
    source = (PACKAGE_ROOT / "launch" / launch_name).read_text(
        encoding="utf-8")

    assert (
        f'DeclareLaunchArgument("waiting_x", default_value="{home_x}")'
        in source)
    assert (
        f'DeclareLaunchArgument("waiting_y", default_value="{home_y}")'
        in source)
    assert '"home_yaw_deg", default_value="180.0"' in source
    assert '"home_yaw_deg": home_yaw_deg' in source
    assert '"init_yaw": home_yaw_rad' in source


def test_full_system_uses_registered_home_pose_and_heading():
    source = (PACKAGE_ROOT / "launch" / "full_system.launch.py").read_text(
        encoding="utf-8")

    assert "FRONT_HOME = (3.60, 0.60)" in source
    assert "REAR_HOME = (3.60, 0.20)" in source
    assert "HOME_YAW_DEG = 180.0" in source
    assert "HOME_YAW_RAD = math.pi" in source
    assert source.count("'home_yaw_deg': HOME_YAW_DEG") == 2
    assert source.count("'init_yaw': HOME_YAW_RAD") == 2
