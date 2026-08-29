"""Host-side resource regressions for dual-camera production inference."""

from pathlib import Path
import threading
import time

import numpy as np

from cooperative_parking_robot import yolo_bev_map_production_node as module
from cooperative_parking_robot import yolo_bev_map_node as baseline_module


class Delegate:
    def __init__(self, counters, lock):
        self.counters = counters
        self.lock = lock

    def __call__(self, _frame, *_args, **_kwargs):
        with self.lock:
            self.counters['active'] += 1
            self.counters['maximum'] = max(
                self.counters['maximum'], self.counters['active'])
        time.sleep(0.05)
        with self.lock:
            self.counters['active'] -= 1
        return []


def test_two_camera_inference_sections_are_serialized(monkeypatch, tmp_path):
    monkeypatch.setattr(
        module.GPU_INFERENCE_GUARD, 'lock_path',
        str(tmp_path / 'gpu.lock'))
    # This test exercises serialization, not the host CUDA runtime.  Avoid
    # probing CUDA on CPU-only CI hosts when the wrapper releases its result.
    monkeypatch.setattr(module, 'release_unused_cuda_cache', lambda: None)
    counters = {'active': 0, 'maximum': 0}
    lock = threading.Lock()
    frame = np.zeros((32, 32, 3), dtype=np.uint8)
    models = [
        module._MaskCenteredYoloModel(Delegate(counters, lock)),
        module._MaskCenteredYoloModel(Delegate(counters, lock)),
    ]
    threads = [threading.Thread(target=model, args=(frame,))
               for model in models]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=2.0)
        assert not thread.is_alive()
    assert counters['maximum'] == 1


def test_production_guards_both_model_load_and_inference():
    source = Path(module.__file__).read_text(encoding='utf-8')
    assert 'def _load_models(self):' in source
    assert source.count('with GPU_INFERENCE_GUARD.hold():') >= 2
    assert 'result.cpu()' in source


def test_detector_suspend_unloads_and_reload_restores_model(monkeypatch):
    node = baseline_module.YoloBevMapNode.__new__(
        baseline_module.YoloBevMapNode)
    node.detector_suspended = False
    node.model = object()
    node.classifier = object()
    node.camera_id = 'cam0'
    logs = []
    node.get_logger = lambda: type('Logger', (), {
        'info': lambda _self, message: logs.append(message),
        'error': lambda _self, message: logs.append(message),
    })()
    monkeypatch.setattr(baseline_module.gc, 'collect', lambda: None)
    monkeypatch.setattr(
        baseline_module, 'release_unused_cuda_cache', lambda: None)

    baseline_module.YoloBevMapNode.perception_suspend_cb(
        node, type('Msg', (), {'data': True})())
    assert node.detector_suspended is True
    assert node.model is None
    assert node.classifier is None

    sentinel = object()
    node._load_models = lambda: setattr(node, 'model', sentinel)
    baseline_module.YoloBevMapNode.perception_suspend_cb(
        node, type('Msg', (), {'data': False})())
    assert node.detector_suspended is False
    assert node.model is sentinel
