#!/usr/bin/env python3
"""BEV Homography/주차면 등록 도구가 공통으로 사용하는 순수 계산 함수.

이 모듈은 ROS 노드와 Flask 서버에 의존하지 않는다. 브라우저에서 받은 픽셀
좌표를 검증하고, Homography 출력 좌표(m)로 바꾸며, 실제 주행 노드가 읽을
``parking_layout.yaml``을 만든다.

중요한 단위 규칙
------------------
새 등록 도구가 만드는 Homography는 **픽셀을 metre로 직접 변환**한다. 따라서
주행 launch의 ``homography_scale_to_m``은 반드시 ``1.0``이어야 한다.

천장 카메라 2대 (v1.11)
-----------------------
카메라가 늘어도 이 모듈이 다루는 좌표계는 하나다. 각 카메라에서 등록할 때
**같은 바닥 점에 같은 실측 (X,Y)m를 입력**하면 H0와 H2의 출력이 자동으로
같은 map frame이 되기 때문이다. 카메라 간 변환 행렬을 따로 만들 필요가 없다.

다만 주차면은 카메라마다 보이는 것이 다르므로, 두 번째 카메라에서 등록할 때
기존 ``parking_layout.yaml``의 슬롯을 읽어와 합쳐야 한다. ``load_layout_yaml``과
``merge_layout_registrations``가 그 역할을 한다.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Iterable, Mapping, Sequence

from cooperative_parking_robot.parking_geometry import (
    ParkingSlot,
    parse_registered_slots,
    slot_polygon,
)


def validate_reference_pairs(reference_pairs: Iterable[Mapping]):
    """브라우저의 기준점 목록을 ``[(u,v,X,Y), ...]`` 형태로 검증한다."""
    validated = []
    for index, pair in enumerate(reference_pairs):
        try:
            pixel = pair['pixel']
            world = pair['world']
            values = (
                float(pixel[0]), float(pixel[1]),
                float(world[0]), float(world[1]),
            )
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            raise ValueError(
                f'reference[{index}] must contain pixel[u,v] and world[X,Y]'
            ) from exc
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f'reference[{index}] contains NaN/Inf')
        validated.append(values)

    if len(validated) < 4:
        raise ValueError('Homography requires at least four reference points')
    if len({(item[0], item[1]) for item in validated}) != len(validated):
        raise ValueError('reference pixel points must be unique')
    if len({(item[2], item[3]) for item in validated}) != len(validated):
        raise ValueError('reference world points must be unique')
    return validated


def transform_points(homography, points: Sequence[Sequence[float]]):
    """3x3 Homography로 픽셀 점들을 metre 좌표로 변환한다."""
    import numpy as np

    matrix = np.asarray(homography, dtype=float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError('homography must be a finite 3x3 matrix')
    result = []
    for index, point in enumerate(points):
        if len(point) != 2:
            raise ValueError(f'point[{index}] must contain u,v')
        vector = matrix @ np.array(
            [float(point[0]), float(point[1]), 1.0], dtype=float)
        if abs(float(vector[2])) < 1e-12:
            raise ValueError(f'point[{index}] maps to infinity')
        x_m = float(vector[0] / vector[2])
        y_m = float(vector[1] / vector[2])
        if not math.isfinite(x_m) or not math.isfinite(y_m):
            raise ValueError(f'point[{index}] maps to NaN/Inf')
        result.append((x_m, y_m))
    return result


def homography_reprojection_errors(homography, reference_pairs):
    """기준점별 재투영 오차(m), RMS(m), 최대 오차(m)를 계산한다."""
    pairs = validate_reference_pairs(reference_pairs)
    projected = transform_points(
        homography, [(item[0], item[1]) for item in pairs])
    errors = [
        math.hypot(px - item[2], py - item[3])
        for (px, py), item in zip(projected, pairs)
    ]
    rms = math.sqrt(sum(value * value for value in errors) / len(errors))
    return errors, rms, max(errors)


def _flat(values, digits=6):
    """ROS 2 double-array에 넣기 좋은 한 줄 YAML 배열을 만든다."""
    return '[' + ', '.join(
        f'{float(value):.{digits}f}' for value in values) + ']'


def _string_array(values):
    return '[' + ', '.join(json.dumps(str(value), ensure_ascii=False)
                           for value in values) + ']'


def render_parking_layout_yaml(
        slots: Sequence[ParkingSlot],
        waiting_polygon: Sequence[Sequence[float]],
        *,
        slot_polygons: Sequence[Sequence[Sequence[float]]] | None = None,
        map_origin_x_m: float = 0.0,
        map_origin_y_m: float = 0.0,
        map_width_m: float,
        map_height_m: float,
        map_resolution_m: float,
        slot_occupancy_overlap_ratio: float = 0.10,
        slot_fit_longitudinal_margin_m: float = 0.06,
        slot_fit_lateral_margin_m: float = 0.06,
        slot_staging_gap_m: float = 0.10,
        waiting_yaw_deg: float = 0.0,
        car_size_m: float = 0.90):
    """등록 결과를 두 ROS 노드가 함께 읽는 parameter YAML로 직렬화한다.

    ROS 2 parameter는 ``[{id: P1, ...}, ...]`` 같은 중첩 객체 배열을 직접
    다루기 불편하므로 ID/중심/크기/Yaw를 같은 순서의 평탄 배열로 저장한다.
    """
    if not slots:
        raise ValueError('at least one parking slot is required')
    if len(waiting_polygon) != 4:
        raise ValueError('waiting_polygon must contain four corners')
    waiting = []
    for index, point in enumerate(waiting_polygon):
        if len(point) != 2:
            raise ValueError(f'waiting_polygon[{index}] must contain x,y')
        x_m, y_m = float(point[0]), float(point[1])
        if not math.isfinite(x_m) or not math.isfinite(y_m):
            raise ValueError('waiting_polygon contains NaN/Inf')
        waiting.extend((x_m, y_m))

    map_origin_x = float(map_origin_x_m)
    map_origin_y = float(map_origin_y_m)
    map_width = float(map_width_m)
    map_height = float(map_height_m)
    resolution = float(map_resolution_m)
    overlap = float(slot_occupancy_overlap_ratio)
    long_margin = float(slot_fit_longitudinal_margin_m)
    lat_margin = float(slot_fit_lateral_margin_m)
    staging_gap = float(slot_staging_gap_m)
    waiting_yaw = float(waiting_yaw_deg)
    car_size = float(car_size_m)
    if not math.isfinite(map_origin_x) or not math.isfinite(map_origin_y):
        raise ValueError('map origin must be finite')
    if map_width <= 0.0 or map_height <= 0.0 or resolution <= 0.0:
        raise ValueError('map dimensions/resolution must be positive')
    if not 0.0 <= overlap <= 1.0:
        raise ValueError('slot_occupancy_overlap_ratio must be in [0,1]')
    if min(long_margin, lat_margin, staging_gap) < 0.0:
        raise ValueError('slot margins and staging gap must be non-negative')
    if not math.isfinite(waiting_yaw):
        raise ValueError('waiting_yaw_deg must be finite')
    if not math.isfinite(car_size) or car_size <= 0.0:
        raise ValueError('car_size_m must be finite and positive')

    # OccupancyGrid origin은 반드시 raster metadata와 같은 map frame이다.
    # 등록 범위 밖 슬롯을 저장하면 후단에서 갑자기 경로 실패가 나므로
    # 등록 단계에서 즉시 알린다.
    def inside_map(point):
        return (map_origin_x <= float(point[0]) <= map_origin_x + map_width and
                map_origin_y <= float(point[1]) <= map_origin_y + map_height)

    waiting_points = list(zip(waiting[0::2], waiting[1::2]))
    if not all(inside_map(point) for point in waiting_points):
        raise ValueError('waiting_polygon must lie inside map bounds')
    if slot_polygons is None:
        runtime_slot_polygons = [slot_polygon(slot) for slot in slots]
    else:
        if len(slot_polygons) != len(slots):
            raise ValueError('slot_polygons must contain one polygon per slot')
        runtime_slot_polygons = []
        for index, polygon in enumerate(slot_polygons):
            if len(polygon) != 4:
                raise ValueError(
                    f'slot_polygons[{index}] must contain four corners')
            cx = sum(float(point[0]) for point in polygon) / 4.0
            cy = sum(float(point[1]) for point in polygon) / 4.0
            # 브라우저는 모서리를 순서 없이 받으므로 중심각으로 CCW 정렬.
            ordered = sorted(
                [(float(point[0]), float(point[1])) for point in polygon],
                key=lambda point: math.atan2(point[1] - cy, point[0] - cx))
            runtime_slot_polygons.append(ordered)

    for slot, polygon in zip(slots, runtime_slot_polygons):
        if not all(inside_map(point) for point in polygon):
            raise ValueError(
                f'parking slot {slot.slot_id} must lie inside map bounds')

    slot_ids = [slot.slot_id for slot in slots]
    slot_coords = [value for slot in slots
                   for value in (slot.center_x_m, slot.center_y_m)]
    slot_sizes = [value for slot in slots
                  for value in (slot.length_m, slot.width_m)]
    slot_yaws_deg = [math.degrees(slot.entry_yaw_rad) for slot in slots]
    flat_slot_polygons = [
        value for polygon in runtime_slot_polygons
        for point in polygon for value in point]
    min_x = min(waiting[0::2])
    max_x = max(waiting[0::2])
    min_y = min(waiting[1::2])
    max_y = max(waiting[1::2])
    waiting_x = sum(waiting[0::2]) / 4.0
    waiting_y = sum(waiting[1::2]) / 4.0

    # ROS 2 parameter YAML은 node name이 일치해야 적용된다. 듀얼 CCTV의
    # sensor node는 yolo_bev_map_node_cam0/_cam2로 이름이 달라지므로
    # perception 공용 layout은 wildcard block으로 전달한다. 선언하지 않은
    # parameter는 각 node가 사용하지 않으며 Fleet 전용 값은 아래 전용
    # block에서 유지한다.
    return f'''# BEV 브라우저 등록 도구가 생성한 주차장 배치 파일
# 모든 좌표/크기는 map frame의 metre, Yaw는 degree이다.
# 같은 index의 slot_ids/slot_coords/slot_sizes/slot_yaws_deg가 한 슬롯이다.
# `/**`는 single/dual CCTV node name과 무관하게 공용 layout을 적용한다.
/**:
  ros__parameters:
    layout_registered: true
    map_resolution: {resolution:.6f}
    map_origin_x_m: {map_origin_x:.6f}
    map_origin_y_m: {map_origin_y:.6f}
    map_width_m: {map_width:.6f}
    map_height_m: {map_height:.6f}
    # Vehicle PCA is an undirected axis; every perception/fleet node receives
    # this single directed waiting orientation from the wildcard block.
    waiting_yaw_deg: {waiting_yaw:.6f}
    car_size_m: {car_size:.6f}
    # 대기영역의 실제 4개 모서리. 차량 중심이 이 다각형 안에 들어오면 타겟이다.
    waiting_polygon: {_flat(waiting)}
    # 기존 코드/수동 설정과의 호환용 축 정렬 bounding box.
    waiting_zone: {_flat([min_x, min_y, max_x, max_y])}
    # 고정 주차면 DB: 중심 2개, 길이/폭 2개, 통로->안쪽 방향 Yaw 1개/슬롯.
    slot_ids: {_string_array(slot_ids)}
    slot_coords: {_flat(slot_coords)}
    slot_sizes: {_flat(slot_sizes)}
    slot_yaws_deg: {_flat(slot_yaws_deg)}
    # 점유 겹침률은 fitted rectangle가 아니라 실제 클릭한 4점 polygon을 사용.
    slot_polygons: {_flat(flat_slot_polygons)}
    use_fixed_slots: true
    slot_occupancy_overlap_ratio: {overlap:.6f}
    # 일시 검출 누락으로 점유칸을 빈칸으로 바꾸지 않는 debounce.
    slot_empty_confirm_frames: 5
    slot_occupied_hold_s: 0.750000
    vehicle_feedback_association_gate_m: 0.450000
    # 차량 전용 YOLO11-Seg 마스크로 차량 크기와 점유율을 계산한다.
    use_mask_vehicle_dimensions: true

fleet_manager_node:
  ros__parameters:
    layout_registered: true
    map_resolution: {resolution:.6f}
    # waiting_polygon은 차량 중심 detection ROI이며 물리 footprint 경계가 아니다.
    waiting_polygon: {_flat(waiting)}
    # retrieve의 실제 map-frame 최종 차량 pose.
    waiting_x: {waiting_x:.6f}
    waiting_y: {waiting_y:.6f}
    # Perception의 polygon 없는 차량 fallback raster와 같은 source mask 크기.
    source_vehicle_fallback_mask_m: {car_size:.6f}
    slot_ids: {_string_array(slot_ids)}
    slot_coords: {_flat(slot_coords)}
    slot_sizes: {_flat(slot_sizes)}
    slot_yaws_deg: {_flat(slot_yaws_deg)}
    # 최종 차량만이 아니라 Front+차량+Rear 결합 footprint가 들어가는지 검사한다.
    slot_fit_longitudinal_margin_m: {long_margin:.6f}
    slot_fit_lateral_margin_m: {lat_margin:.6f}
    # 슬롯 입구 밖 정렬점에서 추가로 띄울 거리.
    slot_staging_gap_m: {staging_gap:.6f}
    # CCTV 차량 중심과 Front/Rear odom 중점 차이가 이 범위 안일 때 A* 시작점에 반영.
    initial_target_offset_gate_m: 0.500000
    use_staged_slot_entry: true
    parking_direction: "forward"
    # 현 실증 배치의 기본 운용은 기존 sequential Front-first 접근이다.
    simultaneous_entry: false
    # MVP 계획 기하 검사는 UI/로그에 남기고 실행 가능한 기존 경로는 계속 사용한다.
    planning_validation_mode: "warn_only"
    # IndividualMoveNode 접근 yaw controller와 동일한 preflight 모델.
    approach_yaw_gain: 1.500000
    approach_max_yaw_rate_rps: 0.150000

# 천장 카메라 2대 구성에서 최종 /parking/* 판단을 담당하는 병합 노드.
# 단일 카메라 구성에서는 이 노드를 띄우지 않으므로 이 블록은 무시된다.
cctv_merge_node:
  ros__parameters:
    layout_registered: true
    map_resolution: {resolution:.6f}
    map_origin_x_m: {map_origin_x:.6f}
    map_origin_y_m: {map_origin_y:.6f}
    map_width_m: {map_width:.6f}
    map_height_m: {map_height:.6f}
    car_size_m: {car_size:.6f}
    waiting_polygon: {_flat(waiting)}
    slot_ids: {_string_array(slot_ids)}
    slot_coords: {_flat(slot_coords)}
    slot_sizes: {_flat(slot_sizes)}
    slot_yaws_deg: {_flat(slot_yaws_deg)}
    slot_polygons: {_flat(flat_slot_polygons)}
    slot_occupancy_overlap_ratio: {overlap:.6f}
    slot_empty_confirm_frames: 5
    slot_occupied_hold_s: 0.750000
    vehicle_feedback_association_gate_m: 0.450000
'''


def load_layout_yaml(path: str):
    """이전에 저장된 ``parking_layout.yaml``에서 슬롯/대기영역을 복원한다.

    두 번째 천장 카메라에서 등록할 때, 첫 번째 카메라가 이미 등록해 둔
    주차면을 지우지 않고 이어붙이기 위해 필요하다. 파일이 없으면 ``None``.

    반환: ``{'slots': [ParkingSlot...], 'slot_polygons': [[(x,y)x4]...],
    'waiting_polygon': [(x,y)x4], 'map_origin_x_m':, 'map_origin_y_m':,
    'map_width_m':, 'map_height_m':,
    'map_resolution_m':}``
    """
    import yaml

    target = Path(path).expanduser()
    if not target.exists():
        return None
    with target.open('r', encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}
    params = {}
    for key in ('/**', 'yolo_bev_map_node'):
        node = document.get(key)
        if isinstance(node, dict) and node.get('ros__parameters'):
            params = node['ros__parameters']
            break
    if not params:
        raise ValueError(
            f'{target}에 `/**` 또는 yolo_bev_map_node의 '
            'ros__parameters 블록이 없습니다')

    slot_ids = [str(value) for value in params.get('slot_ids', [])]
    if not slot_ids:
        return None
    coords = [float(value) for value in params.get('slot_coords', [])]
    sizes = [float(value) for value in params.get('slot_sizes', [])]
    yaws = [float(value) for value in params.get('slot_yaws_deg', [])]
    slots = parse_registered_slots(slot_ids, coords, sizes, yaws)

    polygon_flat = [float(value) for value in params.get('slot_polygons', [])]
    if len(polygon_flat) == 8 * len(slots):
        polygons = [
            [(polygon_flat[8 * i + 2 * c], polygon_flat[8 * i + 2 * c + 1])
             for c in range(4)]
            for i in range(len(slots))
        ]
    else:
        polygons = [list(slot_polygon(slot)) for slot in slots]

    waiting_flat = [float(value) for value in params.get('waiting_polygon', [])]
    if len(waiting_flat) != 8:
        raise ValueError(f'{target}의 waiting_polygon이 8개 값이 아닙니다')
    waiting = [(waiting_flat[i], waiting_flat[i + 1]) for i in range(0, 8, 2)]

    return {
        'slots': slots,
        'slot_polygons': polygons,
        'waiting_polygon': waiting,
        'map_origin_x_m': float(params.get('map_origin_x_m', 0.0)),
        'map_origin_y_m': float(params.get('map_origin_y_m', 0.0)),
        'map_width_m': float(params.get('map_width_m', 0.0)),
        'map_height_m': float(params.get('map_height_m', 0.0)),
        'map_resolution_m': float(params.get('map_resolution', 0.05)),
    }


def merge_layout_registrations(existing, new_slots, new_polygons,
                               new_waiting=None):
    """기존 layout에 이번 카메라에서 새로 등록한 슬롯을 합친다.

    같은 slot_id가 양쪽에 있으면 **새로 등록한 쪽이 이긴다**. 두 카메라가
    같은 슬롯을 볼 때 나중에 더 정확한 위치에서 다시 찍었을 가능성이 높고,
    무엇보다 "다시 찍었는데 반영이 안 된다"가 현장에서 가장 헷갈리기 때문이다.

    대기영역은 새로 등록했으면 교체하고, 아니면 기존 것을 유지한다.
    """
    if existing is None:
        if new_waiting is None:
            raise ValueError('기존 layout이 없으면 대기영역을 반드시 등록해야 합니다')
        return list(new_slots), [list(p) for p in new_polygons], list(new_waiting)

    merged_slots = []
    merged_polygons = []
    new_ids = {slot.slot_id for slot in new_slots}
    for slot, polygon in zip(existing['slots'], existing['slot_polygons']):
        if slot.slot_id in new_ids:
            continue  # 새 등록으로 대체
        merged_slots.append(slot)
        merged_polygons.append(list(polygon))
    for slot, polygon in zip(new_slots, new_polygons):
        merged_slots.append(slot)
        merged_polygons.append(list(polygon))

    waiting = (list(new_waiting) if new_waiting is not None
               else list(existing['waiting_polygon']))
    if not merged_slots:
        raise ValueError('병합 결과에 주차면이 하나도 없습니다')
    return merged_slots, merged_polygons, waiting


def write_text_atomic(path: str, text: str):
    """중간에 전원이 꺼져도 반쪽짜리 설정 파일이 남지 않게 원자 저장한다."""
    target = Path(path).expanduser()
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(target.name + '.tmp')
    temporary.write_text(text, encoding='utf-8')
    temporary.replace(target)
    return target
