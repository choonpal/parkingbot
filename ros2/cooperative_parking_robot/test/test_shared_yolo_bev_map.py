"""Regressions for the one-engine, dual-camera production runtime."""

from types import SimpleNamespace

import pytest

from cooperative_parking_robot import yolo_bev_map_node as baseline_module
from cooperative_parking_robot.shared_yolo_bev_map_node import (
    RoundRobinCameraSelector,
    SharedYoloModelProvider,
)


class RecordingYoloFactory:
    def __init__(self):
        self.calls = []

    def __call__(self, path, **kwargs):
        model = SimpleNamespace(path=path, kwargs=kwargs)
        self.calls.append(model)
        return model


def test_shared_provider_constructs_model_once(tmp_path):
    model_path = tmp_path / 'vehicle.engine'
    model_path.write_bytes(b'fake-engine')
    factory = RecordingYoloFactory()
    provider = SharedYoloModelProvider()

    first, first_task, first_created = provider.acquire(
        factory, str(model_path), 'vehicle_seg')
    second, second_task, second_created = provider.acquire(
        factory, str(model_path), 'vehicle_seg')

    assert first is second
    assert first_task == second_task == 'segment'
    assert first_created is True
    assert second_created is False
    assert provider.load_count == 1
    assert len(factory.calls) == 1


def test_shared_provider_rejects_different_model(tmp_path):
    first_path = tmp_path / 'first.engine'
    second_path = tmp_path / 'second.engine'
    first_path.write_bytes(b'first')
    second_path.write_bytes(b'second')
    provider = SharedYoloModelProvider()
    factory = RecordingYoloFactory()
    provider.acquire(factory, str(first_path), 'vehicle_seg')

    with pytest.raises(ValueError, match='same model and mode'):
        provider.acquire(factory, str(second_path), 'vehicle_seg')


def test_round_robin_selector_is_fair_and_skips_missing_camera():
    selector = RoundRobinCameraSelector(2)
    assert [selector.choose([True, True]) for _ in range(4)] == [0, 1, 0, 1]
    assert selector.choose([False, True]) == 1
    assert selector.choose([False, False]) is None
    assert selector.choose([True, False]) == 0


def test_shared_suspend_pauses_without_reloading_model():
    node = baseline_module.YoloBevMapNode.__new__(
        baseline_module.YoloBevMapNode)
    node.detector_suspended = False
    node.model = object()
    original_model = node.model
    node.classifier = object()
    node.camera_id = 'cam0'
    node._shared_model_provider = SimpleNamespace(retain_on_suspend=True)
    logs = []
    node.get_logger = lambda: SimpleNamespace(
        info=lambda message: logs.append(message),
        error=lambda message: logs.append(message))
    node._load_models = lambda: pytest.fail('shared model must not reload')

    baseline_module.YoloBevMapNode.perception_suspend_cb(
        node, SimpleNamespace(data=True))
    assert node.detector_suspended is True
    assert node.model is original_model

    baseline_module.YoloBevMapNode.perception_suspend_cb(
        node, SimpleNamespace(data=False))
    assert node.detector_suspended is False
    assert node.model is original_model
    assert any('paused' in message for message in logs)
    assert any('resumed' in message for message in logs)
