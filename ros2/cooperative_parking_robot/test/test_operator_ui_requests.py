import json
import queue
import threading
import time
from types import SimpleNamespace

from cooperative_parking_robot.jetson_vision_web_node import (
    JetsonVisionWebNode,
    KIOSK_PAGE,
    MAP_PAGE,
    render_occupancy_map,
)


def web_harness():
    node = JetsonVisionWebNode.__new__(JetsonVisionWebNode)
    node._ui_queue = queue.Queue(maxsize=16)
    node._ui_sequence = 0
    node._ui_client_id = 'web-session'
    node._last_park_publish = 0.0
    node._last_retrieve_publish = 0.0
    node.ui_button_cooldown = 0.0
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=123456789))
    node.build_status = lambda: {
        'park_enabled': True,
        'retrieve_enabled': True,
        'banner': '준비',
        'parking_slots': [
            {'slot_id': 'A1', 'lifecycle': 'EMPTY',
             'retrievable': False, 'retrieve_enabled': False},
            {'slot_id': 'A2', 'lifecycle': 'OCCUPIED',
             'retrievable': True, 'retrieve_enabled': True},
        ],
    }
    return node


def queued_payload(node):
    kind, raw = node._ui_queue.get_nowait()
    assert kind == 'mission'
    return json.loads(raw)


def test_web_park_submits_vehicle_password_and_selected_empty_slot():
    node = web_harness()

    submitted, _, _, request_id = node.request_park(
        '12가 3456', '2468', 'A1')

    assert submitted
    payload = queued_payload(node)
    assert payload == {
        'type': 'park',
        'vehicle_number': '12가3456',
        'password': '2468',
        'destination_slot_id': 'A1',
        'request_id': request_id,
        'client_id': 'web-session',
        'sequence': 1,
        'stamp_ns': 123456789,
    }


def test_web_retrieve_submits_credentials_without_source_slot():
    node = web_harness()

    submitted, _, _, request_id = node.request_retrieve(
        '12가 3456', '2468')

    assert submitted
    payload = queued_payload(node)
    assert payload['vehicle_number'] == '12가3456'
    assert payload['password'] == '2468'
    assert payload['request_id'] == request_id
    assert 'source_slot_id' not in payload


def test_kiosk_uses_password_fields_and_does_not_select_retrieve_slot():
    assert 'id="parkVehicle"' in KIOSK_PAGE
    assert 'id="parkPassword" type="password"' in KIOSK_PAGE
    assert 'id="parkSlot"' in KIOSK_PAGE
    assert 'id="retrieveVehicle"' in KIOSK_PAGE
    assert 'id="retrievePassword" type="password"' in KIOSK_PAGE
    assert 'submitRetrieve(s.slot_id)' not in KIOSK_PAGE


def test_kiosk_layout_supports_seven_inch_touch_viewports():
    assert 'width=device-width' in KIOSK_PAGE
    assert '--touch-target:44px' in KIOSK_PAGE
    assert 'min-height:var(--touch-target)' in KIOSK_PAGE
    assert '@media (max-width:900px)' in KIOSK_PAGE
    assert '@media (max-height:520px)' in KIOSK_PAGE
    assert 'body{width:1024px;height:600px' not in KIOSK_PAGE


def test_live_map_page_uses_the_occupancy_grid_stream():
    assert 'src="/map_feed"' in MAP_PAGE
    assert '/api/map_status' in MAP_PAGE
    assert 'WAITING' in MAP_PAGE
    assert 'P1–P4' in MAP_PAGE
    assert 'ROBOT START' in MAP_PAGE


def test_map_renderer_keeps_ros_positive_y_pointing_up():
    # OccupancyGrid index 0 is the lower-left cell in map coordinates.
    data = [100] + [0] * 11
    image = render_occupancy_map(
        data, width=4, height=3, resolution=0.1,
        origin_x=0.0, origin_y=0.0,
        waiting_polygon=[(-2.0, -2.0), (-1.9, -2.0),
                         (-1.9, -1.9), (-2.0, -1.9)],
        slots=[],
        robot_start_polygon=[(-2.0, -2.0), (-1.9, -2.0),
                             (-1.9, -1.9), (-2.0, -1.9)],
        robot_starts=[], pixels_per_m=100)

    # plot margins are left=58/top=42; the occupied y=0 cell must be near
    # the bottom, while the corresponding top cell remains free/dark.
    bottom_left = image[67, 63]
    top_left = image[47, 63]
    assert int(bottom_left[2]) > 150
    assert int(top_left[2]) < 100


def test_mission_publish_log_excludes_vehicle_number_and_password():
    node = web_harness()
    payload = json.dumps({
        'type': 'retrieve', 'request_id': 'ui-secret',
        'vehicle_number': '12가3456', 'password': '2468'})
    node._ui_queue.put_nowait(('mission', payload))
    published = []
    logs = []
    node.pub_ui_request = SimpleNamespace(
        publish=lambda message: published.append(message.data))
    node.get_logger = lambda: SimpleNamespace(
        info=lambda message: logs.append(message),
        error=lambda message: logs.append(message))

    node._drain_ui_queue()

    assert json.loads(published[0])['password'] == '2468'
    assert '2468' not in logs[0]
    assert '12가3456' not in logs[0]
    assert 'ui-secret' in logs[0]


def test_retrieve_ready_banner_does_not_require_waiting_target_freshness():
    node = JetsonVisionWebNode.__new__(JetsonVisionWebNode)
    now = time.monotonic()
    node._status_lock = threading.Lock()
    node.status_stale_s = 3.0
    node.localization_warning_streak = 5
    node._localization_reject_streak = {'front': 0, 'rear': 0}
    node._status = {
        'fleet': (json.dumps({
            'state': 'WAIT_TARGET', 'mission_id': '', 'empty_count': 1,
            'parking_slots': [{
                'slot_id': 'A1', 'lifecycle': 'OCCUPIED',
                'retrievable': True}],
        }), now),
        'front_state': ('IDLE', now),
        'rear_state': ('IDLE', now),
        'front_phase': ('IDLE', now),
        'rear_phase': ('IDLE', now),
    }

    status = node.build_status()

    assert status['retrieve_enabled']
    assert status['banner'] == '출차 가능 — 차량번호와 비밀번호를 입력하세요'
