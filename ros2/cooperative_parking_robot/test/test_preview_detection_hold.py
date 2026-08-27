"""검출 깜빡임 완화 검증.

추론 한 번이 아무것도 못 찾았다고 곧바로 박스를 지우면, 다음 추론까지
(yolo_every_n / fps) 초 동안 화면이 빈다. 신뢰도가 문턱 근처에서 흔들리면
이게 깜빡임으로 보인다. 짧은 유지 시간을 두되, 유지 중임을 숨기지 않는다.
"""

import ast
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SOURCE = os.path.join(HERE, os.pardir, 'cooperative_parking_robot',
                      'camera_preview_node.py')

METHODS = {'_apply_detection_result'}


def _load():
    with open(SOURCE, encoding='utf-8') as handle:
        tree = ast.parse(handle.read())
    namespace = {}
    methods = []
    for node in tree.body:
        if (isinstance(node, ast.ClassDef)
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

CAR = [{'world': (1.0, 2.0), 'confidence': 0.7}]


def _preview(hold_s=0.6):
    preview = NS['Preview']()
    preview.detection_hold_s = hold_s
    return preview


def _state():
    return {'label': 'cctv0', 'detections': [], 'detection_wall': 0.0,
            'inference_wall': 0.0, 'held': False}


def test_new_detection_replaces_and_clears_hold():
    preview, state = _preview(), _state()
    preview._apply_detection_result(state, CAR, 100.0)
    assert state['detections'] == CAR
    assert state['detection_wall'] == 100.0
    assert state['held'] is False


def test_single_miss_keeps_the_previous_box():
    """이게 깜빡임의 원인이었다."""
    preview, state = _preview(hold_s=0.6), _state()
    preview._apply_detection_result(state, CAR, 100.0)
    preview._apply_detection_result(state, [], 100.3)
    assert state['detections'] == CAR
    assert state['held'] is True


def test_hold_expires_and_the_box_disappears():
    """계속 못 찾으면 결국 지워야 한다. 없는 차를 그리면 안 된다."""
    preview, state = _preview(hold_s=0.6), _state()
    preview._apply_detection_result(state, CAR, 100.0)
    preview._apply_detection_result(state, [], 100.3)
    assert state['held'] is True
    preview._apply_detection_result(state, [], 100.9)
    assert state['detections'] == []
    assert state['held'] is False


def test_hold_window_is_measured_from_the_last_real_detection():
    """유지 중에 또 놓쳐도 창이 연장되면 안 된다."""
    preview, state = _preview(hold_s=0.6), _state()
    preview._apply_detection_result(state, CAR, 100.0)
    for when in (100.2, 100.4, 100.55):
        preview._apply_detection_result(state, [], when)
        assert state['held'] is True
    preview._apply_detection_result(state, [], 100.7)   # 0.6 s 초과
    assert state['detections'] == []


def test_empty_with_no_previous_detection_stays_empty():
    preview, state = _preview(), _state()
    preview._apply_detection_result(state, [], 100.0)
    assert state['detections'] == []
    assert state['held'] is False


def test_hold_disabled_clears_immediately():
    preview, state = _preview(hold_s=0.0), _state()
    preview._apply_detection_result(state, CAR, 100.0)
    preview._apply_detection_result(state, [], 100.01)
    assert state['detections'] == []
    assert state['held'] is False


@pytest.mark.parametrize('result', [CAR, []])
def test_inference_wall_updates_even_when_nothing_is_found(result):
    """빈 결과도 '이 카메라가 지금 보고 있다'는 유효한 관측이다.

    이 값으로 슬롯 관측 가능 여부를 판단하므로, 검출이 없다고 갱신을
    빠뜨리면 멀쩡한 카메라의 슬롯이 '미관측'으로 떨어진다.
    """
    preview, state = _preview(), _state()
    preview._apply_detection_result(state, result, 123.0)
    assert state['inference_wall'] == 123.0


def test_recovering_detection_clears_the_hold_flag():
    preview, state = _preview(hold_s=0.6), _state()
    preview._apply_detection_result(state, CAR, 100.0)
    preview._apply_detection_result(state, [], 100.3)
    assert state['held'] is True
    preview._apply_detection_result(state, CAR, 100.5)
    assert state['held'] is False
    assert state['detection_wall'] == 100.5
