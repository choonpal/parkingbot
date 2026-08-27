"""Regression tests for the automatic velocity-command transport contract."""

import ast
from pathlib import Path

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    ReliabilityPolicy,
)

from cooperative_parking_robot.command_qos import CMD_VEL_QOS
from cooperative_parking_robot.freshness import NSEC_PER_SEC, StampGate


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "cooperative_parking_robot"


def _source_tree(filename):
    return ast.parse((PACKAGE / filename).read_text())


def _automatic_cmd_vel_qos_arguments(filename):
    """Return QoS arguments for automatic cmd_vel endpoint declarations."""
    arguments = []
    for node in ast.walk(_source_tree(filename)):
        if not isinstance(node, ast.Call) or not isinstance(
                node.func, ast.Attribute):
            continue
        endpoint = node.func.attr
        if endpoint not in ("create_publisher", "create_subscription"):
            continue
        if len(node.args) < 3:
            continue
        topic = ast.unparse(node.args[1])
        if "cmd_vel" not in topic or "manual_cmd_vel" in topic:
            continue
        qos_index = 2 if endpoint == "create_publisher" else 3
        arguments.append(ast.unparse(node.args[qos_index]))
    return arguments


def test_cmd_vel_qos_is_latest_only_best_effort_and_volatile():
    assert CMD_VEL_QOS.history == HistoryPolicy.KEEP_LAST
    assert CMD_VEL_QOS.depth == 1
    assert CMD_VEL_QOS.reliability == ReliabilityPolicy.BEST_EFFORT
    assert CMD_VEL_QOS.durability == DurabilityPolicy.VOLATILE


def test_all_automatic_cmd_vel_endpoints_share_the_qos_contract():
    assert _automatic_cmd_vel_qos_arguments(
        "rigid_body_sync_node.py") == ["CMD_VEL_QOS", "CMD_VEL_QOS"]
    assert _automatic_cmd_vel_qos_arguments(
        "individual_move_node.py") == ["CMD_VEL_QOS"]
    assert _automatic_cmd_vel_qos_arguments(
        "stm32_bridge_node.py") == ["CMD_VEL_QOS"]


def test_bridge_command_timeout_default_remains_250_ms():
    defaults = []
    for node in ast.walk(_source_tree("stm32_bridge_node.py")):
        if not isinstance(node, ast.Call) or not isinstance(
                node.func, ast.Attribute):
            continue
        if node.func.attr != "declare_parameter" or len(node.args) < 2:
            continue
        if (isinstance(node.args[0], ast.Constant) and
                node.args[0].value == "command_source_timeout_s"):
            defaults.append(ast.literal_eval(node.args[1]))
    assert defaults == [0.25]


def test_stamp_gate_still_rejects_commands_older_than_timeout():
    now_ns = 10 * NSEC_PER_SEC
    gate = StampGate(max_age_s=0.25, future_tolerance_s=0.10)

    assert gate.accept(now_ns - 250_000_001, now_ns) == (
        False, "STALE_STAMP")
