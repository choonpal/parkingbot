import math
from types import SimpleNamespace

import numpy as np

from cooperative_parking_robot.vehicle_mask_center import (
    mask_min_area_rect_center,
    recenter_vehicle_result_boxes,
    recentered_xyxy,
)


def test_mask_center_matches_min_area_rect_center():
    mask = np.asarray([
        [20.0, 10.0], [40.0, 10.0], [40.0, 30.0], [20.0, 30.0],
    ], dtype=np.float32)
    center = mask_min_area_rect_center(mask)
    assert center is not None
    assert center[0] == 30.0
    assert center[1] == 20.0


def test_recentered_box_uses_mask_center_and_stays_inside_frame():
    mask = np.asarray([
        [20.0, 10.0], [40.0, 10.0], [40.0, 30.0], [20.0, 30.0],
    ], dtype=np.float32)
    box, used = recentered_xyxy(mask, [0.0, 0.0, 80.0, 60.0], 100, 80)
    assert used
    assert (box[0] + box[2]) * 0.5 == 30.0
    assert (box[1] + box[3]) * 0.5 == 20.0
    assert 0.0 <= box[0] < box[2] <= 99.0
    assert 0.0 <= box[1] < box[3] <= 79.0


def test_result_adapter_changes_vehicle_only_and_preserves_conf_cls():
    class Boxes:
        def __init__(self):
            self.data = np.asarray([
                [0.0, 0.0, 80.0, 60.0, 0.91, 0.0],
                [10.0, 10.0, 50.0, 50.0, 0.77, 1.0],
            ], dtype=np.float32)

        @property
        def cls(self):
            return self.data[:, 5]

        def __len__(self):
            return len(self.data)

    masks = SimpleNamespace(xy=[
        np.asarray([[20, 10], [40, 10], [40, 30], [20, 30]], dtype=np.float32),
        np.asarray([[15, 15], [45, 15], [45, 45], [15, 45]], dtype=np.float32),
    ])
    result = SimpleNamespace(boxes=Boxes(), masks=masks)
    original_non_vehicle = result.boxes.data[1].copy()
    changed = recenter_vehicle_result_boxes([result], 100, 80, vehicle_class_id=0)
    assert changed == 1
    vehicle = result.boxes.data[0]
    assert math.isclose(float((vehicle[0] + vehicle[2]) * 0.5), 30.0)
    assert math.isclose(float((vehicle[1] + vehicle[3]) * 0.5), 20.0)
    assert math.isclose(float(vehicle[4]), 0.91, rel_tol=1e-6)
    assert int(vehicle[5]) == 0
    assert np.array_equal(result.boxes.data[1], original_non_vehicle)


def test_invalid_mask_keeps_bbox_fallback():
    original = (5.0, 6.0, 25.0, 26.0)
    box, used = recentered_xyxy(None, original, 100, 80)
    assert not used
    assert box == original


def test_inference_tensor_is_never_mutated_in_place():
    torch = __import__('pytest').importorskip('torch')

    class Boxes:
        def __init__(self, data):
            self.data = data

        @property
        def cls(self):
            return self.data[:, 5]

        def __len__(self):
            return len(self.data)

    with torch.inference_mode():
        inference_data = torch.tensor(
            [[0.0, 0.0, 80.0, 60.0, 0.91, 0.0]],
            dtype=torch.float32)
    original = inference_data.cpu().numpy().copy()
    masks = SimpleNamespace(xy=[np.asarray(
        [[20, 10], [40, 10], [40, 30], [20, 30]], dtype=np.float32)])
    result = SimpleNamespace(boxes=Boxes(inference_data), masks=masks)

    assert recenter_vehicle_result_boxes(
        [result], 100, 80, vehicle_class_id=0) == 1
    assert np.array_equal(inference_data.cpu().numpy(), original)
    assert result.boxes.data is not inference_data
    if hasattr(torch, 'is_inference'):
        assert not torch.is_inference(result.boxes.data)
    vehicle = result.boxes.data[0].cpu().numpy()
    assert math.isclose(float((vehicle[0] + vehicle[2]) * 0.5), 30.0)
    assert math.isclose(float(vehicle[4]), 0.91, rel_tol=1e-6)
