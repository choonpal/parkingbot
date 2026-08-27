"""한 차량이 여러 번 잡히는 것을 합치는지 검증.

NMS 를 통과하고도 중복이 남는다. segmentation 모델이 한 차량을 앞뒤로
나눠 잡거나, 신뢰도 문턱을 낮추면 특히 자주 생긴다. 판정 규칙은 런타임
``merge_detections`` 와 같아야 한다 — 프리뷰만 다르게 합치면 화면과 실제
발행값이 어긋난다.
"""

import ast
import importlib
import math
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SOURCE = os.path.join(HERE, os.pardir, 'cooperative_parking_robot',
                      'camera_preview_node.py')

FUNCTIONS = {'dedupe_detections', 'box_iou'}


def _load():
    try:
        fusion = importlib.import_module(
            'cooperative_parking_robot.bev_fusion_core')
    except ImportError as exc:
        pytest.skip(f'의존성 없음: {exc}')

    with open(SOURCE, encoding='utf-8') as handle:
        tree = ast.parse(handle.read())
    namespace = {'math': math, '_mutual_overlap': fusion._mutual_overlap}
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in FUNCTIONS:
            exec(compile(ast.Module([node], []), SOURCE, 'exec'), namespace)
    missing = FUNCTIONS - set(namespace)
    assert not missing, f'camera_preview_node 에서 못 찾은 함수: {missing}'
    return namespace


NS = _load()
dedupe_detections = NS['dedupe_detections']
box_iou = NS['box_iou']


def _det(confidence, world=None, polygon=None, box=(0, 0, 10, 10)):
    item = {'confidence': confidence, 'box': list(box)}
    if world is not None:
        item['world'] = world
    if polygon is not None:
        item['world_polygon'] = polygon
    return item


def _square(cx, cy, half=0.6):
    return [(cx - half, cy - half), (cx + half, cy - half),
            (cx + half, cy + half), (cx - half, cy + half)]


# ------------------------------------------------------------- 중심 거리
def test_two_boxes_on_the_same_car_become_one():
    found = dedupe_detections([
        _det(0.8, world=(1.00, 2.00)),
        _det(0.6, world=(1.10, 2.05)),      # 11 cm — 같은 차
    ], center_gate_m=0.35, overlap_ratio=0.30)
    assert len(found) == 1
    assert found[0]['confidence'] == 0.8    # 신뢰도 높은 쪽이 남는다
    assert found[0]['merged_count'] == 2


def test_two_real_cars_stay_separate():
    """차량 두 대의 중심이 35 cm 안에 들어올 수는 없다."""
    found = dedupe_detections([
        _det(0.8, world=(1.0, 2.0)),
        _det(0.7, world=(3.0, 2.0)),
    ], center_gate_m=0.35, overlap_ratio=0.30)
    assert len(found) == 2
    assert all(item['merged_count'] == 1 for item in found)


def test_gate_boundary_is_inclusive():
    found = dedupe_detections([
        _det(0.9, world=(0.0, 0.0)),
        _det(0.5, world=(0.35, 0.0)),
    ], center_gate_m=0.35, overlap_ratio=1.0)
    assert len(found) == 1


def test_just_outside_the_gate_is_kept():
    found = dedupe_detections([
        _det(0.9, world=(0.0, 0.0)),
        _det(0.5, world=(0.36, 0.0)),
    ], center_gate_m=0.35, overlap_ratio=1.0)
    assert len(found) == 2


# --------------------------------------------------------------- 겹침률
def test_overlapping_polygons_merge_even_beyond_the_gate():
    """세그멘테이션이 한 차를 앞뒤로 나눠 잡으면 중심은 멀어도 겹친다."""
    found = dedupe_detections([
        _det(0.8, world=(1.0, 2.0), polygon=_square(1.0, 2.0)),
        _det(0.6, world=(1.5, 2.0), polygon=_square(1.4, 2.0)),
    ], center_gate_m=0.20, overlap_ratio=0.30)
    assert len(found) == 1
    assert found[0]['merged_count'] == 2


def test_separate_polygons_are_not_merged():
    found = dedupe_detections([
        _det(0.8, world=(1.0, 2.0), polygon=_square(1.0, 2.0)),
        _det(0.6, world=(4.0, 2.0), polygon=_square(4.0, 2.0)),
    ], center_gate_m=0.35, overlap_ratio=0.30)
    assert len(found) == 2


# --------------------------------------------------- world 없을 때 대비책
def test_pixel_iou_is_used_when_world_is_missing():
    """homography 미등록이어도 눈에 보이는 중복은 합쳐야 한다."""
    found = dedupe_detections([
        _det(0.8, box=(100, 100, 200, 200)),
        _det(0.6, box=(105, 105, 205, 205)),
    ], center_gate_m=0.35, overlap_ratio=0.30)
    assert len(found) == 1


def test_pixel_iou_keeps_distant_boxes():
    found = dedupe_detections([
        _det(0.8, box=(100, 100, 200, 200)),
        _det(0.6, box=(400, 400, 500, 500)),
    ], center_gate_m=0.35, overlap_ratio=0.30)
    assert len(found) == 2


def test_box_iou_values():
    assert box_iou((0, 0, 10, 10), (0, 0, 10, 10)) == pytest.approx(1.0)
    assert box_iou((0, 0, 10, 10), (20, 20, 30, 30)) == 0.0
    assert box_iou((0, 0, 10, 10), (5, 0, 15, 10)) == pytest.approx(1 / 3)


# ----------------------------------------------------------------- 기타
def test_three_way_duplicate_collapses_to_one():
    found = dedupe_detections([
        _det(0.5, world=(1.02, 2.00)),
        _det(0.9, world=(1.00, 2.00)),
        _det(0.7, world=(1.05, 2.02)),
    ], center_gate_m=0.35, overlap_ratio=0.30)
    assert len(found) == 1
    assert found[0]['confidence'] == 0.9
    assert found[0]['merged_count'] == 3


def test_empty_input():
    assert dedupe_detections([]) == []


def test_merged_count_is_always_present():
    """합쳐진 사실을 숨기면 모델이 잘 맞추는 것처럼 보인다."""
    found = dedupe_detections([_det(0.8, world=(1.0, 1.0))])
    assert found[0]['merged_count'] == 1


@pytest.mark.parametrize('gate,ratio', [(-0.1, 0.3), (0.35, 1.5), (0.35, -0.1)])
def test_invalid_thresholds_are_rejected(gate, ratio):
    with pytest.raises(ValueError):
        dedupe_detections([], center_gate_m=gate, overlap_ratio=ratio)
