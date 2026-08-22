"""Static wiring checks for the field-specific branch adapters.

These tests deliberately avoid importing rclpy/launch so they also run in the
lightweight unit-test environment used by the repository.
"""

from __future__ import annotations

import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PYTHON_PACKAGE = PACKAGE_ROOT / "cooperative_parking_robot"
LAUNCH_DIR = PACKAGE_ROOT / "launch"
CONFIG_DIR = PACKAGE_ROOT / "config"


def _source(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _assert_parses(path: Path) -> None:
    ast.parse(_source(path), filename=str(path))


def test_field_python_modules_parse():
    for name in (
            "field_geometry_policy.py",
            "field_fleet_manager_node.py",
            "field_individual_move_node.py",
            "field_pose_fusion_node.py"):
        _assert_parses(PYTHON_PACKAGE / name)


def test_field_launch_wrappers_parse_and_use_measured_homes():
    front = LAUNCH_DIR / "front_robot_field.launch.py"
    rear = LAUNCH_DIR / "rear_robot_field.launch.py"
    _assert_parses(front)
    _assert_parses(rear)

    front_text = _source(front)
    rear_text = _source(rear)
    assert '"waiting_x": "3.60"' in front_text
    assert '"waiting_y": "0.60"' in front_text
    assert '"waiting_x": "3.60"' in rear_text
    assert '"waiting_y": "0.20"' in rear_text
    assert '"simultaneous_entry": "false"' in front_text
    assert '"simultaneous_entry": "false"' in rear_text


def test_setup_points_default_executables_to_field_adapters():
    setup_text = _source(PACKAGE_ROOT / "setup.py")
    assert (
        "fleet_manager = cooperative_parking_robot."
        "field_fleet_manager_node:main" in setup_text)
    assert (
        "individual_move = cooperative_parking_robot."
        "field_individual_move_node:main" in setup_text)
    assert (
        "pose_fusion = cooperative_parking_robot."
        "field_pose_fusion_node:main" in setup_text)
    assert "fleet_manager_legacy" in setup_text
    assert "individual_move_legacy" in setup_text
    assert "pose_fusion_legacy" in setup_text


def test_field_layout_declares_vehicle_only_slot_policy():
    layout = _source(CONFIG_DIR / "parking_layout.yaml")
    assert "map_width_m: 4.400000" in layout
    assert "map_height_m: 3.830000" in layout
    assert "slot_back_clearance_m: 0.230000" in layout
    assert "slot_back_clearance_reserve_m: 0.030000" in layout
    assert "vehicle_slot_longitudinal_margin_m: 0.050000" in layout
    assert "vehicle_slot_lateral_margin_m: 0.050000" in layout
    assert "simultaneous_entry: false" in layout
