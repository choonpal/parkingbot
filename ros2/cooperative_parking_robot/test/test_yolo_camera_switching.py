"""구역 기준 YOLO 카메라 전환 검증.

``camera_preview_node`` 는 rclpy/cv2/flask 를 import 하므로 CI 에서 그대로
불러올 수 없다. 그래서 소스를 AST 로 읽어 **전환 판단에 관여하는 함수만**
꺼내 실행한다. 이렇게 하면 ROS 없이도 실제 코드를 그대로 검증한다.
"""

import ast
import json
import math
import os
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SOURCE = os.path.join(HERE, os.pardir, 'cooperative_parking_robot',
                      'camera_preview_node.py')

FUNCTIONS = {'parse_yolo_regions', 'format_yolo_regions',
             'parse_mission_cameras'}
CONSTANTS = {'MISSION_PARK', 'MISSION_RETRIEVE', 'MISSION_LABELS_KO'}
METHODS = {'_yolo_should_run', '_yolo_pick_active', '_set_yolo_active',
           '_region_owner', '_in_region', '_note_yolo_target',
           'set_yolo_region', 'clear_yolo_region', 'save_yolo_regions',
           'set_yolo_switch_mode', '_world_to_pixel', '_pixel_to_world',
           '_scan_active', '_mission_camera', 'fleet_state_cb'}


def _load():
    with open(SOURCE, encoding='utf-8') as handle:
        tree = ast.parse(handle.read())
    # 꺼낸 함수들이 쓰는 표준 모듈만 넣어준다.
    namespace = {'math': math, 'os': os, 'json': json, 'time': time}
    try:
        import numpy
        namespace['np'] = numpy
    except ImportError:  # numpy 없는 환경에서는 관련 테스트만 건너뛴다
        namespace['np'] = None
    methods = []
    for node in tree.body:
        if (isinstance(node, ast.Assign)
                and any(getattr(t, 'id', None) in CONSTANTS
                        for t in node.targets)):
            exec(compile(ast.Module([node], []), SOURCE, 'exec'), namespace)
        if isinstance(node, ast.FunctionDef) and node.name in FUNCTIONS:
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
parse_yolo_regions = NS['parse_yolo_regions']
format_yolo_regions = NS['format_yolo_regions']
parse_mission_cameras = NS['parse_mission_cameras']


class _Logger:
    """rclpy 로거 흉내. throttle_duration_sec 같은 인자도 받아준다."""

    def info(self, message, **kwargs):
        pass

    def warn(self, message, **kwargs):
        pass

    def error(self, message, **kwargs):
        pass


def _preview(mode='region',
             regions='cctv0:0,0,2.2,3.83; cctv2:2.2,0,4.4,3.83'):
    preview = NS['Preview']()
    preview.yolo = object()
    preview._yolo_labels = ['cctv0', 'cctv2']
    preview.yolo_switch_mode = mode
    preview.yolo_regions = parse_yolo_regions(regions)
    preview.yolo_switch_margin_m = 0.30
    preview.yolo_target_timeout_s = 2.0
    preview.yolo_scan_period_s = 1.0
    preview._yolo_target = None
    preview._yolo_target_wall = 0.0
    preview._yolo_active = None
    preview._yolo_scanning = True
    preview.get_logger = lambda: _Logger()
    preview.yolo_regions_file = ''
    preview._regions_source = None
    preview.mission_cameras = parse_mission_cameras('park:cctv0, retrieve:cctv2')
    preview.fleet_state_topic = '/fleet/state'
    preview.fleet_state_timeout_s = 5.0
    preview._mission_type = ''
    preview._mission_state = ''
    preview._mission_wall = 0.0
    return preview


def _seen_at(preview, x, y, when):
    preview._note_yolo_target(
        [{'world': (x, y), 'confidence': 0.9, 'tracked': True}], when)


# ---------------------------------------------------------------- 파서
def test_regions_parse_two_cameras():
    regions = parse_yolo_regions('cctv0:0,0,2.2,3.83; cctv2:2.2,0,4.4,3.83')
    assert regions == {'cctv0': (0.0, 0.0, 2.2, 3.83),
                       'cctv2': (2.2, 0.0, 4.4, 3.83)}


def test_regions_normalise_reversed_corners():
    """min/max 를 거꾸로 적어도 같은 사각형이 나와야 한다."""
    assert (parse_yolo_regions('cctv2: 4.4 , 3.83 , 2.2 , 0')['cctv2']
            == (2.2, 0.0, 4.4, 3.83))


def test_regions_empty_string_is_no_regions():
    assert parse_yolo_regions('') == {}
    assert parse_yolo_regions('   ;  ') == {}


@pytest.mark.parametrize('text', [
    'cctv0 0,0,1,1',      # ':' 없음
    'cctv0:1,2,3',        # 좌표 부족
    'cctv0:1,2,3,4,5',    # 좌표 초과
    ':1,2,3,4',           # 라벨 없음
    'cctv0:a,b,c,d',      # 숫자 아님
])
def test_regions_reject_malformed(text):
    with pytest.raises(ValueError):
        parse_yolo_regions(text)


# ------------------------------------------------------------ 구역 인계
def test_only_the_owning_camera_runs_inference():
    preview = _preview()
    _seen_at(preview, 1.0, 1.9, 100.0)
    assert preview._yolo_should_run('cctv0', 100.0) is True
    assert preview._yolo_should_run('cctv2', 100.0) is False


def test_handover_when_vehicle_crosses_the_boundary():
    preview = _preview()
    _seen_at(preview, 1.0, 1.9, 100.0)
    assert preview._yolo_pick_active(100.0) == 'cctv0'
    _seen_at(preview, 2.6, 1.9, 100.2)          # 경계 +0.40 m
    assert preview._yolo_pick_active(100.2) == 'cctv2'


def test_margin_prevents_chattering_at_the_boundary():
    """경계 근처에서 매 프레임 담당이 뒤집히면 추적이 끊긴다."""
    preview = _preview()
    _seen_at(preview, 1.0, 1.9, 100.0)
    assert preview._yolo_pick_active(100.0) == 'cctv0'
    # 경계를 0.15 m 넘었지만 여유(0.30 m) 안이라 넘기지 않는다.
    _seen_at(preview, 2.35, 1.9, 100.1)
    assert preview._yolo_pick_active(100.1) == 'cctv0'
    # 확실히 넘어가면 인계.
    _seen_at(preview, 2.6, 1.9, 100.2)
    assert preview._yolo_pick_active(100.2) == 'cctv2'
    # 돌아올 때도 같은 여유가 걸린다.
    _seen_at(preview, 2.05, 1.9, 100.3)
    assert preview._yolo_pick_active(100.3) == 'cctv2'
    _seen_at(preview, 1.5, 1.9, 100.4)
    assert preview._yolo_pick_active(100.4) == 'cctv0'


def test_position_outside_every_region_keeps_current_camera():
    preview = _preview()
    _seen_at(preview, 1.0, 1.9, 100.0)
    assert preview._yolo_pick_active(100.0) == 'cctv0'
    _seen_at(preview, 9.0, 9.0, 100.1)          # 맵 밖
    assert preview._yolo_pick_active(100.1) == 'cctv0'


# ------------------------------------------------------------ 스캔 폴백
def test_stale_target_falls_back_to_alternating_scan():
    """차량 위치를 모르면 두 카메라를 번갈아 봐야 반대편 차를 찾는다."""
    preview = _preview()
    _seen_at(preview, 1.0, 1.9, 100.0)
    assert preview._yolo_pick_active(101.9) == 'cctv0'   # 아직 유효
    first = preview._yolo_pick_active(103.0)
    second = preview._yolo_pick_active(104.0)
    assert preview._yolo_scanning is True
    assert {first, second} == {'cctv0', 'cctv2'}


def test_missed_frames_do_not_erase_the_last_known_position():
    """한 프레임 놓쳤다고 스캔으로 떨어지면 담당이 계속 튄다."""
    preview = _preview()
    _seen_at(preview, 1.0, 1.9, 100.0)
    preview._note_yolo_target([], 100.5)
    preview._note_yolo_target([{'world': None, 'confidence': 0.9}], 100.6)
    assert preview._yolo_target == [1.0, 1.9]
    assert preview._yolo_target_wall == 100.0


def test_tracked_detection_wins_over_higher_confidence():
    preview = _preview()
    preview._note_yolo_target(
        [{'world': (3.0, 1.0), 'confidence': 0.99},
         {'world': (1.0, 1.0), 'confidence': 0.50, 'tracked': True}], 100.0)
    assert preview._yolo_target == [1.0, 1.0]


# ---------------------------------------------------------------- off 모드
def test_off_mode_keeps_running_every_camera():
    preview = _preview(mode='off', regions='')
    assert preview._yolo_should_run('cctv0', 1.0) is True
    assert preview._yolo_should_run('cctv2', 1.0) is True


def test_no_model_means_no_inference_anywhere():
    preview = _preview(mode='off', regions='')
    preview.yolo = None
    assert preview._yolo_should_run('cctv0', 1.0) is False


def test_camera_excluded_by_yolo_cameras_csv_never_runs():
    preview = _preview()
    preview._yolo_labels = ['cctv0']
    assert preview._yolo_should_run('cctv2', 100.0) is False


# ------------------------------------------------------- 화면에서 구역 편집
def test_set_region_normalises_drag_direction():
    """오른쪽 아래에서 왼쪽 위로 끌어도 같은 사각형이어야 한다."""
    preview = _preview()
    forward = preview.set_yolo_region('cctv0', 0.5, 0.5, 2.0, 3.0)
    backward = preview.set_yolo_region('cctv0', 2.0, 3.0, 0.5, 0.5)
    assert forward == backward == (0.5, 0.5, 2.0, 3.0)


def test_set_region_rejects_unknown_camera():
    preview = _preview()
    with pytest.raises(ValueError):
        preview.set_yolo_region('cctv9', 0.0, 0.0, 1.0, 1.0)


def test_set_region_rejects_accidental_click():
    """드래그 없이 딸깍 눌렀을 때 0 크기 구역이 생기면 안 된다."""
    preview = _preview()
    with pytest.raises(ValueError):
        preview.set_yolo_region('cctv0', 1.0, 1.0, 1.01, 1.01)


def test_set_region_rejects_non_finite():
    preview = _preview()
    with pytest.raises(ValueError):
        preview.set_yolo_region('cctv0', 0.0, 0.0, float('nan'), 1.0)


def test_set_region_replaces_the_dict_instead_of_mutating():
    """추론 스레드가 반쯤 바뀐 표를 읽으면 안 된다."""
    preview = _preview()
    before = preview.yolo_regions
    preview.set_yolo_region('cctv0', 0.0, 0.0, 1.0, 1.0)
    assert preview.yolo_regions is not before
    assert before['cctv0'] == (0.0, 0.0, 2.2, 3.83)   # 옛 표는 그대로


def test_clearing_the_last_region_falls_back_to_off():
    """구역이 없으면 담당을 못 정해 조용히 멈춘다. off 로 되돌려야 한다."""
    preview = _preview()
    preview.clear_yolo_region('cctv0')
    assert preview.yolo_switch_mode == 'region'
    preview.clear_yolo_region('cctv2')
    assert preview.yolo_switch_mode == 'off'
    assert preview._yolo_should_run('cctv0', 1.0) is True


def test_switch_mode_needs_at_least_one_region():
    preview = _preview(mode='off', regions='')
    with pytest.raises(ValueError):
        preview.set_yolo_switch_mode('region')
    preview.set_yolo_region('cctv0', 0.0, 0.0, 2.2, 3.83)
    assert preview.set_yolo_switch_mode('region') == 'region'


def test_switch_mode_rejects_unknown_value():
    preview = _preview()
    with pytest.raises(ValueError):
        preview.set_yolo_switch_mode('auto')


# ------------------------------------------------------------- 저장 / 복원
def test_saved_file_round_trips_through_the_parser(tmp_path):
    """저장한 파일을 다음 실행에서 그대로 읽어야 한다."""
    preview = _preview()
    preview.yolo_regions_file = str(tmp_path / 'sub' / 'yolo_regions.csv')
    preview.set_yolo_region('cctv0', 0.0, 0.0, 2.2, 3.83)
    result = preview.save_yolo_regions()

    assert os.path.isfile(result['path'])
    with open(result['path'], encoding='utf-8') as handle:
        body = ''.join(line for line in handle
                       if not line.lstrip().startswith('#'))
    assert parse_yolo_regions(body) == preview.yolo_regions


def test_format_is_readable_back():
    regions = parse_yolo_regions('cctv0:0,0,2.2,3.83; cctv2:2.2,0,4.4,3.83')
    assert parse_yolo_regions(format_yolo_regions(regions)) == regions


# ------------------------------------------------------------- 내장 JS 문법
def test_embedded_page_javascript_parses():
    """웹 페이지 JS 에 문법 오류가 있으면 화면이 통째로 안 뜬다.

    py_compile 은 이걸 못 잡는다. 실제로 ``const mb`` 를 두 번 선언한 적이
    있었고, 그때 스크립트 전체가 파싱에 실패해 카메라 화면까지 사라졌다.
    node 가 없는 환경에서는 건너뛴다.
    """
    import re
    import shutil
    import subprocess
    import tempfile

    node = shutil.which('node') or shutil.which('nodejs')
    if node is None:
        pytest.skip('node 없음')

    with open(SOURCE, encoding='utf-8') as handle:
        text = handle.read()
    match = re.search(r'<script>(.*?)</script>', text, re.S)
    assert match, 'camera_preview_node 에서 <script> 블록을 못 찾았습니다'

    with tempfile.NamedTemporaryFile('w', suffix='.js', encoding='utf-8',
                                     delete=False) as handle:
        handle.write(match.group(1))
        js_path = handle.name
    try:
        done = subprocess.run([node, '--check', js_path],
                              capture_output=True, text=True)
    finally:
        os.unlink(js_path)
    assert done.returncode == 0, f'내장 JS 문법 오류:\n{done.stderr}'


# ------------------------------------------------- 구역을 카메라 화면에 되짚기
def _with_homography():
    np = NS['np']
    if np is None:
        pytest.skip('numpy 없음')
    preview = _preview()
    # 기울어진 천장 카메라와 비슷한 픽셀->미터 호모그래피
    preview.pixel_to_world_H = {'cctv0': np.array([
        [0.0065, 0.0008, -1.9],
        [0.0004, 0.0071, -1.3],
        [0.00002, 0.00009, 1.0]])}
    preview._world_to_pixel_H = {}
    return preview


@pytest.mark.parametrize('px,py', [(50, 40), (320, 240), (600, 450),
                                   (10, 470), (630, 10)])
def test_world_to_pixel_is_the_inverse_of_pixel_to_world(px, py):
    """구역을 카메라 화면에 되짚어 그리려면 역변환이 정확해야 한다."""
    preview = _with_homography()
    world = preview._pixel_to_world('cctv0', px, py)
    back = preview._world_to_pixel('cctv0', *world)
    # _pixel_to_world 가 mm 단위로 반올림하므로 그만큼의 여유를 둔다.
    assert back[0] == pytest.approx(px, abs=0.2)
    assert back[1] == pytest.approx(py, abs=0.2)


def test_world_to_pixel_reuses_the_cached_inverse():
    preview = _with_homography()
    preview._world_to_pixel('cctv0', 1.0, 1.0)
    assert 'cctv0' in preview._world_to_pixel_H
    cached = preview._world_to_pixel_H['cctv0']
    preview._world_to_pixel('cctv0', 2.0, 2.0)
    assert preview._world_to_pixel_H['cctv0'] is cached


def test_world_to_pixel_without_homography_returns_none():
    preview = _with_homography()
    assert preview._world_to_pixel('cctv2', 1.0, 1.0) is None


# ------------------------------------------------- 미션(입차/출차) 기준 전환
class _Msg:
    def __init__(self, data):
        self.data = data


def _fleet(mission, state='NAVIGATING'):
    return _Msg(json.dumps({'mission_type': mission, 'state': state,
                            'sequence': 1}))


def _mission_preview(now=1000.0):
    preview = _preview(mode='mission')
    preview.fleet_state_cb(_fleet(''))
    preview._mission_wall = now
    return preview


def test_mission_cameras_parse():
    assert parse_mission_cameras('park:cctv0, retrieve:cctv2') == {
        'park': 'cctv0', 'retrieve': 'cctv2'}
    assert parse_mission_cameras(' PARK : cctv0 ') == {'park': 'cctv0'}


@pytest.mark.parametrize('text', [
    'park cctv0',        # ':' 없음
    'towing:cctv0',      # 미션 이름이 아님
    'park:',             # 라벨 없음
])
def test_mission_cameras_reject_malformed(text):
    with pytest.raises(ValueError):
        parse_mission_cameras(text)


def test_park_mission_uses_cam0_only():
    preview = _mission_preview()
    preview.fleet_state_cb(_fleet('park'))
    preview._mission_wall = 1000.0
    assert preview._yolo_should_run('cctv0', 1000.0) is True
    assert preview._yolo_should_run('cctv2', 1000.0) is False


def test_retrieve_mission_uses_cam2_only():
    preview = _mission_preview()
    preview.fleet_state_cb(_fleet('retrieve'))
    preview._mission_wall = 1000.0
    assert preview._yolo_should_run('cctv2', 1000.0) is True
    assert preview._yolo_should_run('cctv0', 1000.0) is False


def test_switching_missions_moves_the_camera():
    preview = _mission_preview()
    preview.fleet_state_cb(_fleet('park'))
    preview._mission_wall = 1000.0
    assert preview._yolo_pick_active(1000.0) == 'cctv0'
    preview.fleet_state_cb(_fleet('retrieve'))
    preview._mission_wall = 1000.5
    assert preview._yolo_pick_active(1000.5) == 'cctv2'
    assert preview._yolo_scanning is False


def test_idle_between_missions_falls_back_to_scan():
    """미션이 없을 때 한 대만 보면 반대편 차를 못 찾는다."""
    preview = _mission_preview()
    preview.fleet_state_cb(_fleet(''))
    preview._mission_wall = 1000.0
    first = preview._yolo_pick_active(1000.0)
    second = preview._yolo_pick_active(1001.0)
    assert preview._yolo_scanning is True
    assert {first, second} == {'cctv0', 'cctv2'}


def test_stale_fleet_state_falls_back_to_scan():
    """fleet_manager 가 죽었는데 마지막 미션을 계속 믿으면 안 된다."""
    preview = _mission_preview()
    preview.fleet_state_cb(_fleet('park'))
    preview._mission_wall = 1000.0
    assert preview._yolo_pick_active(1004.0) == 'cctv0'      # 아직 유효
    preview._yolo_pick_active(1010.0)                        # 5 s 초과
    assert preview._yolo_scanning is True


def test_unparsable_fleet_state_is_ignored():
    """JSON 이 깨져도 노드가 죽거나 미션이 뒤집히면 안 된다."""
    preview = _mission_preview()
    preview.fleet_state_cb(_fleet('park'))
    preview._mission_wall = 1000.0
    preview.fleet_state_cb(_Msg('{not json'))
    preview.fleet_state_cb(_Msg('[]'))
    assert preview._mission_type == 'park'


def test_mission_mode_needs_a_mapping():
    preview = _preview(mode='off', regions='')
    preview.mission_cameras = {}
    with pytest.raises(ValueError):
        preview.set_yolo_switch_mode('mission')


def test_switch_mode_accepts_mission():
    preview = _mission_preview()
    assert preview.set_yolo_switch_mode('mission') == 'mission'
