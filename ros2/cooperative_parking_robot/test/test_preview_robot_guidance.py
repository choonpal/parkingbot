"""ArUco 로봇 마커 -> 입·출차 이동 방향 안내 검증.

천장에 보이는 두 마커가 Front/Rear 주차로봇이다. 두 로봇이 차량을 앞뒤에서
들어 올리므로 **두 마커의 중점**을 차량 중심으로 보고, 거기서 목적지까지가
지금 가야 할 방향이다.

  입차(park)     : 배정된 빈 슬롯으로
  출차(retrieve) : 대기영역으로
"""

import ast
import importlib
import math
import os
import threading

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SOURCE = os.path.join(HERE, os.pardir, 'cooperative_parking_robot',
                      'camera_preview_node.py')

FUNCTIONS = {'parse_robot_markers'}
CONSTANTS = {'MISSION_PARK', 'MISSION_RETRIEVE', 'ROBOT_ROLES'}
METHODS = {'_robot_marker_world', '_slot_centroid', '_guidance_goal',
           '_guidance', 'empty_slot_ids'}


def _load():
    try:
        fusion = importlib.import_module(
            'cooperative_parking_robot.bev_fusion_core')
    except ImportError as exc:
        pytest.skip(f'의존성 없음: {exc}')

    with open(SOURCE, encoding='utf-8') as handle:
        tree = ast.parse(handle.read())

    namespace = {'math': math, 'polygon_centroid': fusion.polygon_centroid}
    methods = []
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in FUNCTIONS:
            exec(compile(ast.Module([node], []), SOURCE, 'exec'), namespace)
        elif (isinstance(node, ast.Assign)
              and any(getattr(t, 'id', None) in CONSTANTS
                      for t in node.targets)):
            exec(compile(ast.Module([node], []), SOURCE, 'exec'), namespace)
        elif (isinstance(node, ast.ClassDef)
              and node.name == 'CameraPreviewNode'):
            methods = [n for n in node.body
                       if isinstance(n, ast.FunctionDef) and n.name in METHODS]
    missing = METHODS - {m.name for m in methods}
    assert not missing, f'camera_preview_node 에서 못 찾은 메서드: {missing}'
    holder = ast.ClassDef(name='Preview', bases=[], keywords=[],
                          body=methods, decorator_list=[])
    module = ast.Module([holder], [])
    ast.fix_missing_locations(module)
    exec(compile(module, SOURCE, 'exec'), namespace)
    return namespace


NS = _load()
parse_robot_markers = NS['parse_robot_markers']
PARK = NS['MISSION_PARK']
RETRIEVE = NS['MISSION_RETRIEVE']

P1 = [(0.4, 1.0), (1.6, 1.0), (1.6, 3.4), (0.4, 3.4)]     # 중심 (1.0, 2.2)
P2 = [(3.2, 1.0), (4.4, 1.0), (4.4, 3.4), (3.2, 3.4)]     # 중심 (3.8, 2.2)
WAIT = [(0.0, 4.0), (4.8, 4.0), (4.8, 4.6), (0.0, 4.6)]   # 중심 (2.4, 4.3)

NOW = 100.0


def _preview(mission='', markers=None, marker_wall=NOW,
             empty=('P1', 'P2'), waiting=None):
    preview = NS['Preview']()
    preview._lock = threading.Lock()
    preview.slots = [('P1', P1), ('P2', P2)]
    preview.waiting = waiting
    preview.robot_marker_ids = {'front': 2, 'rear': 1}
    preview.robot_marker_stale_s = 2.0
    preview.production_marker_visible_stale_s = 1.0
    preview.guidance_forced_mission = ''
    preview._mission_type = mission
    preview._destination_slot_id = ''
    preview._source_slot_id = ''
    preview.slot_state = {
        slot_id: {'observed': True, 'occupied': slot_id not in empty}
        for slot_id, _ in preview.slots
    }
    markers_by_id = {marker.get('id'): marker for marker in (markers or [])}
    preview.production_marker_visibility = {}
    for role, marker_id in preview.robot_marker_ids.items():
        marker = markers_by_id.get(marker_id)
        world = None if marker is None else marker.get('world')
        preview.production_marker_visibility[role] = {
            'visible': marker is not None,
            'wall': marker_wall,
            'pose': (None if world is None else {
                'x_m': world[0], 'y_m': world[1], 'yaw_deg': 0.0,
                'frame_id': 'map',
            }),
            'pose_wall': marker_wall if world is not None else 0.0,
        }
    # A local preview detector may still have these rows, but guidance must
    # use only the production pose/visible contract above.
    preview.cameras = [{'label': 'cctv0', 'markers': list(markers or []),
                        'marker_wall': marker_wall}]
    return preview


def _marker(marker_id, x, y):
    return {'id': marker_id, 'world': (x, y)}


BOTH = [_marker(2, 1.0, 0.5), _marker(1, 2.0, 0.5)]   # 중점 (1.5, 0.5)


# ------------------------------------------------------------------- 파서
def test_parse_robot_markers():
    assert parse_robot_markers('front:2, rear:1') == {'front': 2, 'rear': 1}
    assert parse_robot_markers(' FRONT : 10 ') == {'front': 10}


@pytest.mark.parametrize('text', [
    'front 2',          # ':' 없음
    'left:2',           # 역할 이름 아님
    'front:abc',        # 정수 아님
    'front:-1',         # 음수
    'front:2, rear:2',  # 같은 ID
])
def test_parse_robot_markers_rejects_bad_input(text):
    with pytest.raises(ValueError):
        parse_robot_markers(text)


# --------------------------------------------------------------- 마커 수집
def test_robot_markers_are_collected_by_role():
    preview = _preview(markers=BOTH)
    assert preview._robot_marker_world(NOW) == {
        'front': (1.0, 0.5), 'rear': (2.0, 0.5)}


def test_stale_markers_are_ignored():
    """오래된 마커를 계속 믿으면 로봇이 없는 자리에 화살표가 남는다."""
    preview = _preview(markers=BOTH, marker_wall=NOW - 5.0)
    assert preview._robot_marker_world(NOW) == {}


def test_unrelated_marker_ids_are_ignored():
    preview = _preview(markers=[_marker(7, 1.0, 1.0)])
    assert preview._robot_marker_world(NOW) == {}


def test_marker_without_world_coordinate_is_ignored():
    preview = _preview(markers=[{'id': 2, 'world': None}])
    assert preview._robot_marker_world(NOW) == {}


def test_preview_detection_cannot_override_production_visibility():
    preview = _preview(markers=BOTH)
    preview.production_marker_visibility['front']['visible'] = False
    assert preview._robot_marker_world(NOW) == {'rear': (2.0, 0.5)}


# ------------------------------------------------------------------ 입차
def test_park_points_at_the_assigned_slot():
    preview = _preview(mission=PARK, markers=BOTH)
    preview._destination_slot_id = 'P2'
    guidance = preview._guidance(NOW)
    assert guidance['goal'] == 'P2'
    assert guidance['from'] == [1.5, 0.5]                 # 두 마커의 중점
    assert guidance['to'] == pytest.approx([3.8, 2.2])
    assert guidance['distance_m'] == pytest.approx(
        math.hypot(3.8 - 1.5, 2.2 - 0.5), abs=1e-3)


def test_park_falls_back_to_a_confirmed_empty_slot():
    """fleet 이 아직 슬롯을 안 정했어도 방향은 보여준다."""
    preview = _preview(mission=PARK, markers=BOTH, empty=('P2',))
    guidance = preview._guidance(NOW)
    assert guidance['goal'] == 'P2'


def test_park_without_any_empty_slot_says_so():
    preview = _preview(mission=PARK, markers=BOTH, empty=())
    guidance = preview._guidance(NOW)
    assert guidance['to'] is None
    assert '빈자리' in guidance['reason']


def test_park_ignores_a_destination_that_is_not_registered():
    """layout 에 없는 슬롯 ID 가 와도 죽지 않고 빈자리로 폴백한다."""
    preview = _preview(mission=PARK, markers=BOTH, empty=('P1',))
    preview._destination_slot_id = 'P9'
    assert preview._guidance(NOW)['goal'] == 'P1'


# ------------------------------------------------------------------ 출차
def test_retrieve_points_at_the_waiting_area():
    preview = _preview(mission=RETRIEVE, markers=BOTH, waiting=WAIT)
    guidance = preview._guidance(NOW)
    assert guidance['goal'] == 'WAIT'
    assert guidance['to'] == pytest.approx([2.4, 4.3])


def test_retrieve_without_waiting_area_says_so():
    preview = _preview(mission=RETRIEVE, markers=BOTH, waiting=None)
    guidance = preview._guidance(NOW)
    assert guidance['to'] is None
    assert '대기영역' in guidance['reason']


def test_park_and_retrieve_point_in_opposite_directions():
    """같은 자리에서 입차와 출차의 방향이 같으면 표시가 무의미하다."""
    park = _preview(mission=PARK, markers=BOTH, waiting=WAIT)
    park._destination_slot_id = 'P1'
    exit_ = _preview(mission=RETRIEVE, markers=BOTH, waiting=WAIT)
    assert park._guidance(NOW)['goal'] != exit_._guidance(NOW)['goal']


# ------------------------------------------------------------- 부분 관측
def test_single_marker_still_gives_a_direction_but_warns():
    """한 대만 보여도 방향은 쓸모 있다. 다만 중점이 아니라고 알려야 한다."""
    preview = _preview(mission=PARK, markers=[_marker(2, 1.0, 0.5)])
    preview._destination_slot_id = 'P1'
    guidance = preview._guidance(NOW)
    assert guidance['from'] == [1.0, 0.5]
    assert 'front' in guidance['reason']


def test_no_marker_means_no_arrow():
    preview = _preview(mission=PARK, markers=[])
    guidance = preview._guidance(NOW)
    assert guidance['from'] is None
    assert 'Production' in guidance['reason']
    assert 'pose' in guidance['reason']


def test_no_mission_means_no_arrow():
    preview = _preview(mission='', markers=BOTH)
    guidance = preview._guidance(NOW)
    assert guidance['from'] is None
    assert guidance['mission'] == ''


def test_forced_mission_overrides_fleet():
    """fleet 없이 확인할 때 손으로 미션을 고정할 수 있어야 한다."""
    preview = _preview(mission='', markers=BOTH)
    preview.guidance_forced_mission = RETRIEVE
    preview.waiting = WAIT
    guidance = preview._guidance(NOW)
    assert guidance['mission'] == RETRIEVE
    assert guidance['forced'] is True
    assert guidance['goal'] == 'WAIT'


# ------------------------------------------------------------------- 방향
@pytest.mark.parametrize('goal,expected_deg', [
    ((1.5, 3.5), 90.0),      # 위
    ((4.5, 0.5), 0.0),       # 오른쪽
    ((0.5, 0.5), 180.0),     # 왼쪽
])
def test_heading_matches_the_goal_direction(goal, expected_deg):
    preview = _preview(mission=RETRIEVE, markers=BOTH,
                       waiting=[(goal[0] - 0.1, goal[1] - 0.1),
                                (goal[0] + 0.1, goal[1] - 0.1),
                                (goal[0] + 0.1, goal[1] + 0.1),
                                (goal[0] - 0.1, goal[1] + 0.1)])
    guidance = preview._guidance(NOW)
    assert guidance['heading_deg'] == pytest.approx(expected_deg, abs=0.5)
