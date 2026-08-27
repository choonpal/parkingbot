import json
import queue
import threading
import time
from types import SimpleNamespace

from cooperative_parking_robot.jetson_vision_web_node import (
    JetsonVisionWebNode,
    KIOSK_CSS,
    KIOSK_JS,
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
        'parking_spaces': [
            {'slot_id': 'A1', 'display_number': 1, 'available': True},
            {'slot_id': 'A2', 'display_number': 2, 'available': False},
        ],
    }
    return node


def queued_payload(node):
    kind, raw = node._ui_queue.get_nowait()
    assert kind == 'mission'
    return json.loads(raw)


def test_web_park_submits_credentials_for_automatic_slot_assignment():
    node = web_harness()

    submitted, _, _, request_id = node.request_park('12가 3456', '2468')

    assert submitted
    payload = queued_payload(node)
    assert payload == {
        'type': 'park',
        'vehicle_number': '12가3456',
        'password': '2468',
        'request_id': request_id,
        'client_id': 'web-session',
        'sequence': 1,
        'stamp_ns': 123456789,
    }


def test_web_park_keeps_explicit_slot_compatibility_for_operator_clients():
    node = web_harness()

    submitted, _, _, _ = node.request_park('12가 3456', '2468', 'A1')

    assert submitted
    assert queued_payload(node)['destination_slot_id'] == 'A1'


def test_web_park_rejects_explicit_slot_that_perception_marks_unavailable():
    node = web_harness()

    submitted, message, _, _ = node.request_park(
        '12가 3456', '2468', 'A2')

    assert not submitted
    assert message == '선택한 주차면을 사용할 수 없습니다'
    assert node._ui_queue.empty()


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


def test_kiosk_keeps_customer_auto_assignment_and_gates_developer_selection():
    assert 'id="parkVehicle"' in KIOSK_PAGE
    assert 'id="parkPassword" type="password"' in KIOSK_PAGE
    assert 'id="parkSlot"' not in KIOSK_PAGE
    assert 'id="retrieveVehicle"' in KIOSK_PAGE
    assert 'id="retrievePassword" type="password"' in KIOSK_PAGE
    assert 'id="sitePlan"' in KIOSK_PAGE
    assert "get('dev') === '1'" in KIOSK_JS
    assert 'if (developerMode && developerSlot.value)' in KIOSK_JS
    assert 'payload.destination_slot_id = developerSlot.value' in KIOSK_JS
    assert 'id="developerControls"' in KIOSK_PAGE
    assert 'id="developerSlot"' in KIOSK_PAGE
    assert '/api/estop' not in KIOSK_PAGE
    assert '/api/estop' not in KIOSK_JS
    assert "labels[state] || '시스템 준비 중'" in KIOSK_JS
    assert '입차 구역' in KIOSK_JS
    assert "DETECTING: '차량 감지 중'" in KIOSK_JS
    assert "READY: '정차 확인'" in KIOSK_JS
    assert "ABSENT: '차량 없음'" in KIOSK_JS
    assert '출발 위치' in KIOSK_JS
    assert '.site-zone-label' in KIOSK_CSS
    assert '.site-zone-sub' in KIOSK_CSS
    assert 'url(#entry-zone-hatch)' in KIOSK_CSS
    assert 'layout-legend .entry' in KIOSK_CSS


def test_customer_kiosk_server_has_no_estop_route():
    node = JetsonVisionWebNode.__new__(JetsonVisionWebNode)
    client = node._make_flask_app().test_client()

    assert client.post('/api/estop').status_code == 404


def test_kiosk_layout_supports_seven_inch_touch_viewports():
    assert 'width=device-width' in KIOSK_PAGE
    assert '--touch: 48px' in KIOSK_CSS
    assert 'min-height: 52px' in KIOSK_CSS
    assert '@media (max-height: 520px)' in KIOSK_CSS
    assert 'grid-template-columns: minmax(0, 61%)' in KIOSK_CSS


def test_site_plan_uses_camera_orientation_and_perception_availability():
    node = JetsonVisionWebNode.__new__(JetsonVisionWebNode)
    node.map_width_m = 4.0
    node.map_height_m = 4.0
    node.map_slots = [
        ('P1', [(0.0, 2.0), (1.0, 2.0), (1.0, 3.0)]),
        ('P2', [(1.0, 2.0), (2.0, 2.0), (2.0, 3.0)]),
    ]
    node.waiting_polygon = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    node.robot_start_polygon = [(3.0, 0.0), (4.0, 0.0), (4.0, 1.0)]

    spaces, layout = node._site_layout_payload({
        'available_slot_ids': ['P2'],
        'active_destination_slot_id': 'P2',
    }, [
        {'slot_id': 'P1', 'lifecycle': 'EMPTY'},
        {'slot_id': 'P2', 'lifecycle': 'EMPTY'},
    ])

    assert not spaces[0]['available']
    assert spaces[1]['available']
    assert spaces[1]['assigned']
    assert [space['display_number'] for space in spaces] == [1, 2]
    waiting_x = sum(point[0] for point in layout['waiting_polygon']) / len(
        layout['waiting_polygon'])
    robot_x = sum(point[0] for point in layout['robot_start_polygon']) / len(
        layout['robot_start_polygon'])
    assert waiting_x > robot_x
    # High map-y parking spaces appear above the low map-y waiting area.
    assert max(point[1] for point in spaces[0]['polygon']) < min(
        point[1] for point in layout['waiting_polygon'])


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


def fleet_status_harness(fleet_payload):
    node = JetsonVisionWebNode.__new__(JetsonVisionWebNode)
    now = time.monotonic()
    node._status_lock = threading.Lock()
    node.status_stale_s = 3.0
    node.localization_warning_streak = 5
    node._localization_reject_streak = {'front': 0, 'rear': 0}
    node._status = {
        'fleet': (json.dumps(fleet_payload), now),
        'front_state': ('DRIVE', now),
        'rear_state': ('DRIVE', now),
        'front_phase': ('DRIVE', now),
        'rear_phase': ('DRIVE', now),
    }
    return node


def test_operator_ui_shows_astar_blocker_instead_of_generic_waiting_text():
    node = fleet_status_harness({
        'state': 'PLAN_PATH',
        'mission_id': 'park-1',
        'empty_count': 1,
        'planning_validation_mode': 'warn_only',
        'validation_warnings': [],
        'planning_blocker': {
            'code': 'ASTAR_NO_PATH',
            'mission_phase': 'PLAN_PATH',
        },
    })

    status = node.build_status()

    assert status['planning_blocker']['code'] == 'ASTAR_NO_PATH'
    assert status['banner'] == '경로 생성 불가: ASTAR_NO_PATH'


def test_operator_ui_shows_warn_only_findings_while_mission_continues():
    node = fleet_status_harness({
        'state': 'NAVIGATING',
        'mission_id': 'park-1',
        'empty_count': 1,
        'planning_validation_mode': 'warn_only',
        'validation_warnings': [{
            'code': 'SLOT_TOO_SHORT',
            'mission_phase': 'PLAN_PATH',
        }],
        'planning_blocker': None,
    })

    status = node.build_status()

    assert status['planning_warning']
    assert status['banner'] == '경고 운행 중: SLOT_TOO_SHORT'


def test_operator_ui_distinguishes_detecting_vehicle_from_ready_gate():
    node = fleet_status_harness({
        'state': 'WAIT_TARGET',
        'mission_id': '',
        'empty_count': 1,
        'parking_slots': [],
    })
    now = time.monotonic()
    node._status['target_ready'] = (False, now)
    node._status['target_status'] = (json.dumps({
        'version': 1,
        'state': 'DETECTING',
        'observed_recently': True,
        'ready': False,
    }), now)

    status = node.build_status()

    assert status['target_state'] == 'DETECTING'
    assert status['site_layout']['vehicle_present']
    assert not status['target_ready']
    assert not status['park_enabled']
    assert status['banner'] == '차량 감지 중 — 정차 확인까지 잠시 기다려 주세요'
