from pathlib import Path
import re

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_id0_aligned_pose_has_one_config_source():
    config = yaml.safe_load(
        (ROOT / 'config/id0_calibration.yaml').read_text())
    calibration = config['/**']['ros__parameters']
    assert calibration['aruco_distance_offset_m'] == pytest.approx(
        0.569069426)
    assert 0.215930574 + calibration['aruco_distance_offset_m'] == \
        pytest.approx(0.785)
    assert 0.012879378 + calibration['aruco_lateral_offset_m'] == \
        pytest.approx(0.0)
    assert -3.155371 + calibration['aruco_yaw_offset_deg'] == \
        pytest.approx(0.0)


@pytest.mark.parametrize(
    ('filename', 'node_names'),
    (
        ('front_robot.launch.py',
         ('rigid_body_sync_node', 'front_individual_move')),
        ('rear_robot.launch.py', ('rear_individual_move',)),
        ('full_system.launch.py',
         ('rigid_body_sync_node', 'front_individual_move',
          'rear_individual_move')),
    ),
)
def test_production_launches_load_id0_calibration_file(filename, node_names):
    source = (ROOT / 'launch' / filename).read_text()
    assert 'id0_calibration.yaml' in source
    assert 'aruco_distance_offset_m' not in source
    for node_name in node_names:
        pattern = (
            rf"name\s*=\s*['\"]{node_name}['\"]"
            rf"[\s\S]{{0,180}}parameters=\[id0_calibration,")
        assert re.search(pattern, source), node_name


@pytest.mark.parametrize(
    'filename', ('rear_robot.launch.py', 'full_system.launch.py'))
def test_id0_calibration_is_not_passed_to_camera_node(filename):
    source = (ROOT / 'launch' / filename).read_text()
    pattern = (
        r"name\s*=\s*['\"]rear_marker_camera_node['\"]"
        r"[\s\S]{0,220}parameters=\[id0_calibration,")
    assert not re.search(pattern, source)


@pytest.mark.parametrize(
    'filename',
    ('rear_robot.launch.py', 'full_system.launch.py'),
)
def test_id0_alignment_is_passed_to_aruco_tracker(filename):
    source = (ROOT / 'launch' / filename).read_text()
    pattern = (
        r"executable\s*=\s*['\"]aruco_tracker['\"]"
        r"[\s\S]{0,180}parameters=\[id0_calibration,")
    assert re.search(pattern, source)


def test_python_nodes_do_not_embed_measured_id0_offset():
    for filename in ('individual_move_node.py', 'rigid_body_sync_node.py'):
        source = (ROOT / 'cooperative_parking_robot' / filename).read_text()
        assert '0.570' not in source
        assert "declare_parameter('aruco_distance_offset_m', 0.0)" in \
            source or \
            'declare_parameter("aruco_distance_offset_m", 0.0)' in source
