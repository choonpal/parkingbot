"""프리뷰의 빈자리 표시 검증.

``camera_preview_node`` 는 rclpy/cv2/flask 를 import 하므로 그대로 못 불러온다.
표시 판단에 관여하는 함수만 AST 로 꺼내 실행한다.

핵심 성질은 하나다 — **보는 카메라가 없는 칸을 빈자리로 말하지 않는다.**
차 있는 칸을 비었다고 하면 로봇이 그리로 출발한다.
"""

import ast
import importlib
import os
import threading
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SOURCE = os.path.join(HERE, os.pardir, 'cooperative_parking_robot',
                      'camera_preview_node.py')

METHODS = {'_update_slots', '_slot_appearance', 'empty_slot_ids',
           '_detection_world_polygon', '_pixel_to_world'}
CLASSES = {'SlotDetection'}
CONSTANTS = {'SLOT_FREE_COLOUR', 'SLOT_BUSY_COLOUR', 'SLOT_UNKNOWN_COLOUR'}


def _load():
    try:
        np = importlib.import_module('numpy')
        fusion = importlib.import_module(
            'cooperative_parking_robot.bev_fusion_core')
    except ImportError as exc:            # numpy 없는 최소 환경
        pytest.skip(f'의존성 없음: {exc}')

    with open(SOURCE, encoding='utf-8') as handle:
        tree = ast.parse(handle.read())

    namespace = {
        'np': np, 'time': time, 'os': os,
        'SlotOccupancyTracker': fusion.SlotOccupancyTracker,
        'slot_observability': fusion.slot_observability,
        'image_corner_coverage': fusion.image_corner_coverage,
    }
    methods = []
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name in CLASSES:
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


class _Logger:
    def info(self, message, **kwargs):
        pass

    def warn(self, message, **kwargs):
        pass


# 4.8 x 4.63 m 맵 위의 슬롯 두 개. P1 은 왼쪽, P2 는 오른쪽.
P1 = [(0.4, 1.0), (1.6, 1.0), (1.6, 3.4), (0.4, 3.4)]
P2 = [(3.2, 1.0), (4.4, 1.0), (4.4, 3.4), (3.2, 3.4)]

# 왼쪽만 덮는 카메라 / 오른쪽만 덮는 카메라
COVER_LEFT = [(0.0, 0.0), (2.4, 0.0), (2.4, 4.63), (0.0, 4.63)]
COVER_RIGHT = [(2.4, 0.0), (4.8, 0.0), (4.8, 4.63), (2.4, 4.63)]


def _preview(now=100.0, confirm_frames=2):
    preview = NS['Preview']()
    preview.get_logger = lambda: _Logger()
    # 픽셀->월드 변환 대신 테스트가 미리 넣어둔 월드 폴리곤을 쓴다.
    # (인스턴스 속성이 클래스 메서드보다 먼저 잡힌다)
    preview._detection_world_polygon = (
        lambda label, detection: detection.get('_world_polygon'))
    preview._lock = threading.Lock()
    preview.slots = [('P1', P1), ('P2', P2)]
    preview.camera_coverage = {'cctv0': COVER_LEFT, 'cctv2': COVER_RIGHT}
    preview.slot_detection_stale_s = 1.5
    preview.slot_overlap_threshold = 0.10
    preview.slot_empty_confirm_frames = confirm_frames
    preview.slot_occupied_hold_s = 0.0
    preview.slot_state = {}
    preview.pixel_to_world_H = {}
    preview.slot_tracker = NS['SlotOccupancyTracker'](
        ['P1', 'P2'], overlap_threshold=0.10,
        empty_confirm_frames=confirm_frames,
        occupied_hold_s=0.0, now=now)
    preview.cameras = [
        {'label': 'cctv0', 'detections': [], 'detection_wall': now,
         'inference_wall': now},
        {'label': 'cctv2', 'detections': [], 'detection_wall': now,
         'inference_wall': now},
    ]
    return preview


def _car_at(x, y, half=0.5):
    """슬롯을 확실히 덮는 정사각형 검출 하나."""
    return {'world': (x, y),
            'geometry': {'corners': [(0, 0), (1, 0), (1, 1), (0, 1)]},
            '_world_polygon': [(x - half, y - half), (x + half, y - half),
                               (x + half, y + half), (x - half, y + half)]}


def _tick(preview, now, live=('cctv0', 'cctv2'), detections=None):
    """한 검출 주기를 흉내낸다.

    ``live`` 에 든 카메라만 방금 추론을 돌렸다고 표시한다. 빠진 카메라는
    inference_wall 이 그대로라 곧 stale 로 판정된다.

    관측 가능 판단은 detection_wall(뭔가 찾은 시각)이 아니라
    inference_wall(추론이 돈 시각)을 쓴다. 빈 결과도 유효한 관측이다.
    """
    for camera in preview.cameras:
        if camera['label'] in live:
            camera['inference_wall'] = now
            camera['detections'] = list(
                (detections or {}).get(camera['label'], []))
            if camera['detections']:
                camera['detection_wall'] = now
    preview._update_slots(now)


def _settle(preview, frames, now=100.0, step=0.5, live=('cctv0', 'cctv2')):
    for i in range(frames):
        _tick(preview, now + i * step, live=live)


# --------------------------------------------------------------- 기본 동작
def test_starts_occupied_before_anything_is_seen():
    """아직 아무것도 못 본 시점에 빈자리로 말하면 안 된다."""
    preview = _preview()
    assert preview.empty_slot_ids() == []


def test_slot_becomes_free_after_confirm_frames():
    preview = _preview(confirm_frames=2)
    _tick(preview, 100.0)
    assert preview.empty_slot_ids() == []      # 아직 1프레임
    _tick(preview, 100.5)
    assert preview.empty_slot_ids() == ['P1', 'P2']


def test_car_in_slot_marks_it_occupied_immediately():
    preview = _preview(confirm_frames=2)
    _settle(preview, 3)
    assert 'P1' in preview.empty_slot_ids()
    _tick(preview, 101.5, detections={'cctv0': [_car_at(1.0, 2.2)]})
    assert preview.empty_slot_ids() == ['P2']
    assert preview.slot_state['P1']['occupied'] is True


# ------------------------------------------------------- 미관측 안전성 (핵심)
def test_slot_with_no_live_camera_is_never_reported_empty():
    """cam2 가 멈추면 P2 는 '빈자리'가 아니라 '모름'이 되어야 한다."""
    preview = _preview(confirm_frames=2)
    _settle(preview, 3)
    assert preview.empty_slot_ids() == ['P1', 'P2']

    # cctv0 만 계속 살아 있다 -> P2 를 보는 카메라가 없다
    _tick(preview, 105.0, live=('cctv0',))

    assert preview.slot_state['P2']['observed'] is False
    assert preview.empty_slot_ids() == ['P1']


def test_unobserved_slot_keeps_its_last_verdict():
    """관측 불가 동안에는 직전 판정을 유지하고 목록에서만 빠진다.

    주의: 빈자리였던 칸이 안 보이다가 돌아오면 **곧바로 다시 빈자리**로
    나온다. 안 보이는 사이에 차가 들어왔다면 다음 검출 주기에야 잡힌다.
    런타임(cctv_merge)의 SlotOccupancyTracker 와 같은 동작이라 프리뷰만
    다르게 만들지 않았다.
    """
    preview = _preview(confirm_frames=3)
    _settle(preview, 4)
    assert 'P2' in preview.empty_slot_ids()

    _tick(preview, 105.0, live=('cctv0',))                # cctv2 멈춤
    assert preview.slot_state['P2']['observed'] is False
    assert 'P2' not in preview.empty_slot_ids()           # 목록에서는 빠짐
    assert preview.slot_state['P2']['occupied'] is False  # 판정은 유지

    _tick(preview, 106.0)                                 # cctv2 복귀
    assert preview.slot_state['P2']['observed'] is True
    assert 'P2' in preview.empty_slot_ids()


def test_occupied_slot_must_refill_the_confirm_count_after_returning():
    """차 있던 칸은 복귀 후 확인 프레임을 처음부터 다시 채워야 한다."""
    preview = _preview(confirm_frames=3)
    _tick(preview, 100.0, detections={'cctv2': [_car_at(3.8, 2.2)]})
    assert preview.slot_state['P2']['occupied'] is True

    # stale 판정(1.5 s)을 넘기려면 충분히 시간을 띄워야 한다.
    _tick(preview, 103.0, live=('cctv0',))                # cctv2 멈춤
    assert preview.slot_state['P2']['observed'] is False

    # 복귀 후 차가 빠졌어도 3프레임을 채우기 전까지는 점유로 남는다.
    for offset, expected in ((0.0, True), (0.5, True), (1.0, False)):
        _tick(preview, 104.0 + offset)
        assert preview.slot_state['P2']['occupied'] is expected


def test_yolo_switched_off_on_one_camera_does_not_free_its_slots():
    """구역/미션 전환으로 cam2 YOLO 가 꺼져도 P2 가 비었다고 하면 안 된다.

    검출이 안 오는 것과 '차가 없는 것'은 다르다.
    """
    preview = _preview(confirm_frames=2)
    _settle(preview, 5, live=('cctv0',))                  # cctv2 는 추론 안 함
    assert preview.slot_state['P2']['observed'] is False
    assert 'P2' not in preview.empty_slot_ids()


# ------------------------------------------------------------------ 표시 색
def test_appearance_colours_match_state():
    preview = _preview(confirm_frames=1)
    _settle(preview, 2)
    free_colour, free_tag, _ = preview._slot_appearance('P1')
    assert free_colour == NS['SLOT_FREE_COLOUR']
    assert free_tag == 'FREE'

    _tick(preview, 101.0, detections={'cctv0': [_car_at(1.0, 2.2)]})
    busy_colour, busy_tag, _ = preview._slot_appearance('P1')
    assert busy_colour == NS['SLOT_BUSY_COLOUR']
    assert busy_tag == 'BUSY'


def test_unknown_slot_is_grey_not_green():
    """모르는 칸을 초록으로 그리면 화면만 보고 빈자리로 착각한다."""
    preview = _preview()
    colour, tag, _ = preview._slot_appearance('P1')
    assert colour == NS['SLOT_UNKNOWN_COLOUR']
    assert tag == '?'
    assert colour != NS['SLOT_FREE_COLOUR']


def test_appearance_of_unregistered_slot_is_unknown():
    preview = _preview()
    colour, tag, _ = preview._slot_appearance('P9')
    assert colour == NS['SLOT_UNKNOWN_COLOUR']
    assert tag == '?'
