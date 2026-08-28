"""카메라 원근 왜곡과 실제 주행 마커 gate를 혼동하지 않는지 검증한다."""

import ast
import math
from pathlib import Path

import pytest


SOURCE = (Path(__file__).resolve().parents[1]
          / 'cooperative_parking_robot' / 'camera_preview_node.py')
FUNCTIONS = {'marker_metrics', 'marker_readiness'}


def _load_functions():
    tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
    namespace = {'math': math}
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in FUNCTIONS:
            exec(compile(ast.Module([node], []), str(SOURCE), 'exec'), namespace)
    assert FUNCTIONS <= namespace.keys()
    return namespace


NS = _load_functions()
marker_metrics = NS['marker_metrics']
marker_readiness = NS['marker_readiness']


def _stable_marker(**overrides):
    marker = {
        'min_side_px': 24.0,
        'area_px': 850.0,
        'edge_margin_px': 42.0,
        'history_hits': 15,
        'history_samples': 15,
        'detection_ratio': 1.0,
    }
    marker.update(overrides)
    return marker


def test_perspective_trapezoid_is_not_a_driving_failure():
    # 강한 사다리꼴이라 raw 변 편차는 크지만, 크기와 경계 여유는 충분하다.
    metrics = marker_metrics(
        [(60, 210), (92, 207), (99, 239), (72, 232)],
        0.24, 640, 360)
    metrics.update({
        'history_hits': 15,
        'history_samples': 15,
        'detection_ratio': 1.0,
    })
    assert metrics['side_spread'] > 0.12
    result = marker_readiness(metrics, 'front', True, True)
    assert result['drive_status'] == 'CCTV pose 입력 정상'
    assert result['drive_class'] == 'ok'
    assert result['drive_ready'] is True


def test_missing_production_topic_is_unknown_not_bad_marker():
    result = marker_readiness(_stable_marker(), 'rear', None, False)
    assert result['drive_status'] == 'Production 미수신'
    assert result['drive_class'] == 'warn'
    assert result['drive_ready'] is None
    assert '/rear/cctv_marker_visible' in result['drive_reason']


def test_production_false_means_pose_is_not_available_for_driving():
    result = marker_readiness(_stable_marker(), 'front', False, True)
    assert result['drive_status'] == 'CCTV pose 입력 중단'
    assert result['drive_class'] == 'err'
    assert result['drive_ready'] is False
    assert '절대 pose' in result['drive_reason']


def test_small_or_intermittent_marker_is_explained_as_warning():
    result = marker_readiness(
        _stable_marker(min_side_px=16.0, history_hits=9,
                       detection_ratio=0.6),
        'front', True, True)
    assert result['drive_status'] == 'CCTV pose 입력 정상(주의)'
    assert '최소 변이 작음' in result['drive_reason']
    assert '간헐적' in result['drive_reason']


def test_marker_geometry_reports_actual_resolution_and_edge_margin():
    metrics = marker_metrics(
        [(10, 20), (30, 20), (30, 40), (10, 40)],
        0.24, 100, 80)
    assert metrics['min_side_px'] == pytest.approx(20.0)
    assert metrics['area_px'] == pytest.approx(400.0)
    assert metrics['edge_margin_px'] == pytest.approx(10.0)
    assert metrics['mm_per_px'] == pytest.approx(12.0)


def test_non_driving_marker_id_is_labelled_as_reference_only():
    result = marker_readiness(_stable_marker(), '', None, False)
    assert result['drive_status'] == '참고용 ID'
    assert result['drive_ready'] is None


def test_preview_subscribes_to_actual_role_visibility_topics():
    source = SOURCE.read_text(encoding='utf-8')
    assert "f'/{role}/cctv_marker_visible'" in source
    assert 'production_marker_visible_cb' in source
