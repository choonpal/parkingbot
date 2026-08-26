import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from cooperative_parking_robot.bev_fusion_core import CameraDetection
from cooperative_parking_robot.mvp_integration_nodes import (
    HomeAwareIndividualMoveNode,
    _OriginPublisher,
    _origin_aware_calibrator_html,
    _shift_detection,
    _shift_point,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class _RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_negative_map_origin_translates_world_coordinates_to_local_grid():
    assert _shift_point((-0.40, -0.80), -0.40, -0.80) == (0.0, 0.0)
    assert _shift_point((4.40, 3.83), -0.40, -0.80) == pytest.approx(
        (4.80, 4.63))

    detection = CameraDetection(
        camera_id='cam0',
        center=(0.60, 0.40),
        polygon=[(0.0, 0.0), (1.2, 0.0), (1.2, 0.8)],
        yaw=math.pi,
    )
    shifted = _shift_detection(detection, -0.40, -0.80)
    assert shifted.center == pytest.approx((1.0, 1.2))
    assert shifted.polygon[0] == pytest.approx((0.4, 0.8))
    assert shifted.yaw == pytest.approx(math.pi)


def test_origin_publisher_preserves_delegate_and_sets_metadata():
    delegate = _RecordingPublisher()
    owner = SimpleNamespace(map_origin_x_m=-0.4, map_origin_y_m=-0.8)
    message = SimpleNamespace(info=SimpleNamespace(
        origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0))))

    _OriginPublisher(delegate, owner).publish(message)

    assert delegate.messages == [message]
    assert message.info.origin.position.x == pytest.approx(-0.4)
    assert message.info.origin.position.y == pytest.approx(-0.8)


def test_origin_aware_calibrator_ui_exposes_registered_map_geometry():
    html = _origin_aware_calibrator_html()

    assert 'id="mapOriginX"' in html
    assert 'id="mapOriginY"' in html
    assert 'value="4.80"' in html
    assert 'value="4.63"' in html
    assert 'map_origin_x_m:' in html
    assert 'map_origin_y_m:' in html


def test_calibrator_wrapper_saves_and_previews_registered_map_origin():
    source = (PACKAGE_ROOT / 'cooperative_parking_robot' /
              'mvp_integration_nodes.py').read_text(encoding='utf-8')

    assert 'class OriginAwareBevLayoutCalibratorNode' in source
    assert "kwargs['map_origin_x_m'] = origin_x" in source
    assert "kwargs['map_origin_y_m'] = origin_y" in source
    assert '(point[0] - self.map_origin_x_m)' in source
    assert '(point[1] - self.map_origin_y_m)' in source


def test_layout_calibration_launch_uses_expanded_map_defaults():
    source = (PACKAGE_ROOT / 'launch' /
              'bev_layout_calibration.launch.py').read_text(encoding='utf-8')

    assert "'map_origin_x_m', default_value='-0.40'" in source
    assert "'map_origin_y_m', default_value='-0.80'" in source
    assert "'map_width_m', default_value='4.80'" in source
    assert "'map_height_m', default_value='4.63'" in source
    assert "'default_map_origin_x_m': _float('map_origin_x_m')" in source
    assert "'default_map_origin_y_m': _float('map_origin_y_m')" in source


def _returning_node(route_complete):
    node = HomeAwareIndividualMoveNode.__new__(HomeAwareIndividualMoveNode)
    node.phase = 'RETURN_HOME'
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


def test_home_return_requires_registered_heading_before_return_done():
    node = _returning_node(route_complete=True)

    node.run_return()

    assert node.advance_calls == [(0.06, pytest.approx(math.pi))]
    assert node.transitions == ['RETURNED']
    assert len(node.pub_return_done.messages) == 1
    assert node.pub_return_done.messages[0].data is True


@pytest.mark.parametrize(
    ('simultaneous_entry', 'expected_goal_yaw'),
    ((False, None), (True, math.pi / 2.0)),
)
def test_front_first_staging_defers_yaw_until_prealign(
        simultaneous_entry, expected_goal_yaw):
    node = HomeAwareIndividualMoveNode.__new__(HomeAwareIndividualMoveNode)
    node.phase = 'TO_REAR_STAGING'
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
