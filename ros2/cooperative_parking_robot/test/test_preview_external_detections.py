"""Production 검출 envelope를 프리뷰에 연결하는 경로의 ROS 없는 검증."""

import ast
import math
import os
import threading
import time
from types import SimpleNamespace

import pytest


HERE = os.path.dirname(os.path.abspath(__file__))
SOURCE = os.path.join(HERE, os.pardir, 'cooperative_parking_robot',
                      'camera_preview_node.py')
FUNCTIONS = {
    '_camera_key', 'camera_ids_match', 'parse_detection_topics',
    'production_detection_item',
}
METHODS = {
    '_project_production_detection', 'external_detection_cb',
    '_apply_detection_result',
}


def _load():
    with open(SOURCE, encoding='utf-8') as handle:
        tree = ast.parse(handle.read())
    namespace = {
        'math': math,
        'time': time,
        # callback 단위 테스트에서는 이미 검증된 envelope dict를 직접 넘긴다.
        'decode_detection_envelope': lambda payload: payload,
    }
    methods = []
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in FUNCTIONS:
            exec(compile(ast.Module([node], []), SOURCE, 'exec'), namespace)
        elif (isinstance(node, ast.ClassDef)
              and node.name == 'CameraPreviewNode'):
            methods = [item for item in node.body
                       if isinstance(item, ast.FunctionDef)
                       and item.name in METHODS]
    missing_functions = FUNCTIONS - set(namespace)
    missing_methods = METHODS - {method.name for method in methods}
    assert not missing_functions
    assert not missing_methods
    holder = ast.ClassDef(name='Preview', bases=[], keywords=[], body=methods,
                          decorator_list=[])
    module = ast.Module([holder], [])
    ast.fix_missing_locations(module)
    exec(compile(module, SOURCE, 'exec'), namespace)
    return namespace


NS = _load()
parse_detection_topics = NS['parse_detection_topics']
production_detection_item = NS['production_detection_item']
camera_ids_match = NS['camera_ids_match']


def _detection(camera_id='cam0'):
    return SimpleNamespace(
        camera_id=camera_id,
        center=(1.2345, 2.3456),
        polygon=((1.0, 2.0), (1.5, 2.0), (1.5, 2.7), (1.0, 2.7)),
        yaw=math.pi / 4.0,
        length_m=0.78,
        width_m=0.46,
        in_waiting=True,
        confidence=0.9134,
        axis_dist_m=1.2222,
        vehicle_class='compact',
        classified_wheelbase_m=0.51,
    )


def _envelope(sequence=7, stamp_ns=1_900_000_000, detections=None,
              camera_id='cam0'):
    return {
        'camera_id': camera_id,
        'stamp_ns': stamp_ns,
        'sequence': sequence,
        'homography_ok': True,
        'coverage_polygon': ((0.0, 0.0), (4.0, 0.0),
                             (4.0, 3.0), (0.0, 3.0)),
        'detections': (
            [_detection(camera_id)] if detections is None else detections),
    }


class _Logger:
    def warn(self, _message, **_kwargs):
        pass


class _Clock:
    def __init__(self, nanoseconds=2_000_000_000):
        self._now = SimpleNamespace(nanoseconds=nanoseconds)

    def now(self):
        return self._now


def _state():
    return {
        'label': 'cctv0',
        'detection_topic': '/cctv0/detections',
        'detections': [], 'slot_detections': [],
        'detection_wall': 0.0, 'inference_wall': 0.0, 'held': False,
        'detection_camera_id': '', 'detection_sequence': 0,
        'detection_stamp_ns': 0, 'detection_messages': 0,
        'detection_invalid': 0, 'detection_dropped': 0,
        'detection_rate_hz': 0.0, 'detection_rate_count': 0,
        'detection_rate_wall': time.monotonic() - 2.0,
        'transport_age_s': None, 'homography_ok': None,
        'source_coverage': None, 'detection_error': '', 'infer_ms': 0.0,
    }


def _preview():
    preview = NS['Preview']()
    preview._lock = threading.Lock()
    preview.detection_hold_s = 0.6
    preview._world_to_pixel = lambda _label, x, y: (x * 100.0, y * 100.0)
    preview._update_tracks = lambda _label, _items: None
    preview._note_yolo_target = lambda _items, _now: None
    preview.get_clock = lambda: _Clock()
    preview.get_logger = lambda: _Logger()
    preview.slot_updates = []
    preview._update_slots = lambda now: preview.slot_updates.append(now)
    return preview


def test_detection_topics_are_mapped_one_to_one():
    assert parse_detection_topics(
        '/cctv0/detections,/cctv2/detections', ['cctv0', 'cctv2']) == {
            'cctv0': '/cctv0/detections',
            'cctv2': '/cctv2/detections',
        }
    assert parse_detection_topics('', ['rear']) == {}


@pytest.mark.parametrize('topics', [
    '/cctv0/detections',
    '/same,/same',
    '/cctv0/detections,',
])
def test_detection_topics_reject_ambiguous_mapping(topics):
    with pytest.raises(ValueError):
        parse_detection_topics(topics, ['cctv0', 'cctv2'])


def test_cam_and_cctv_aliases_match_but_different_numbers_do_not():
    assert camera_ids_match('cctv0', 'cam0')
    assert camera_ids_match('/cctv2/image_rect', 'cam2')
    assert not camera_ids_match('cctv0', 'cam2')


def test_production_detection_preserves_detailed_metrics():
    item = production_detection_item(_detection())
    assert item['source'] == 'production'
    assert item['world'] == [1.234, 2.346]
    assert item['confidence'] == 0.913
    assert item['yaw_deg'] == 45.0
    assert item['length_m'] == 0.78
    assert item['width_m'] == 0.46
    assert item['in_waiting'] is True
    assert item['axis_dist_m'] == 1.222
    assert item['vehicle_class'] == 'compact'
    assert item['classified_wheelbase_m'] == 0.51


def test_external_callback_updates_overlay_health_and_slot_inputs():
    preview, state = _preview(), _state()
    preview.external_detection_cb(state, SimpleNamespace(data=_envelope()))

    assert state['detection_camera_id'] == 'cam0'
    assert state['detection_sequence'] == 7
    assert state['detection_messages'] == 1
    assert state['homography_ok'] is True
    assert state['transport_age_s'] == pytest.approx(0.1)
    assert state['inference_wall'] > 0.0
    assert state['detections'][0]['source'] == 'production'
    assert state['detections'][0]['center_px'] == [123.4, 234.6]
    assert len(state['detections'][0]['pixel_polygon']) == 4
    assert state['slot_detections'] == state['detections']
    assert len(preview.slot_updates) == 1


def test_empty_external_result_is_still_a_fresh_observation():
    preview, state = _preview(), _state()
    preview.external_detection_cb(
        state, SimpleNamespace(data=_envelope(detections=[])))
    assert state['detections'] == []
    assert state['slot_detections'] == []
    assert state['inference_wall'] > 0.0
    assert state['detection_messages'] == 1


def test_wrong_camera_and_out_of_order_envelopes_are_not_displayed():
    preview, state = _preview(), _state()
    preview.external_detection_cb(
        state, SimpleNamespace(data=_envelope(camera_id='cam2')))
    assert state['detection_invalid'] == 1
    assert state['detection_messages'] == 0

    preview.external_detection_cb(state, SimpleNamespace(data=_envelope()))
    preview.external_detection_cb(
        state, SimpleNamespace(data=_envelope(sequence=6,
                                              stamp_ns=1_800_000_000)))
    assert state['detection_messages'] == 1
    assert state['detection_dropped'] == 1


def test_dedicated_launch_never_starts_a_camera_or_second_yolo():
    launch_path = os.path.join(
        HERE, os.pardir, 'launch', 'cctv_detection_preview.launch.py')
    with open(launch_path, encoding='utf-8') as handle:
        source = handle.read()
    assert "executable='camera_preview'" in source
    assert "'enable_yolo': False" in source
    assert "'detection_topics_csv'" in source
    assert "'calibration_height_px': 360" in source
    assert "executable='opencv_camera'" not in source
    assert "executable='yolo_bev_map'" not in source
