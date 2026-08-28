#!/usr/bin/env python3
"""천장 CCTV 여러 대의 검출 결과를 공통 world 좌표계에서 병합하는 순수 로직.

이 모듈은 ROS/OpenCV/YOLO에 의존하지 않는다. ``yolo_bev_map_node``(카메라별
sensor 인스턴스)가 만든 검출 envelope을 ``cctv_merge_node``가 받아서 하나의
주차장 상태로 합칠 때 쓰는 계산을 모아둔 곳이며, 단위 테스트가 쉬워지도록
클래스/함수 단위로 분리했다.

설계 전제 (docs/DUAL_CCTV_MERGE_20260812.md와 짝을 이룬다)
--------------------------------------------------------
1. **카메라마다 자기 homography를 갖는다.** cam0은 ``H0``, cam2는 ``H2``.
   두 H 모두 "같은 바닥 기준점을 같은 실측 metre 값으로" 등록했기 때문에
   출력 좌표계가 자동으로 같은 map frame이 된다. 따라서 이 모듈은 카메라 간
   추가 변환 행렬을 쓰지 않는다 — 이미 world 좌표로 들어온 값을 다룬다.
2. **겹치는 영역은 두 카메라가 같은 차량을 두 번 보고한다.** 그대로 두면
   장애물이 두 개로 보이고 슬롯 점유율도 중복 계산되므로 반드시 dedup한다.
   같은 물체인지 판단하는 기준은 (a) 중심 거리 gate, (b) polygon 겹침률이다.
3. **카메라는 자기 시야(coverage polygon) 안의 슬롯만 판정할 자격이 있다.**
   cam0이 못 보는 슬롯을 cam0의 "차량 없음"으로 빈자리 처리하면 실제로는
   차가 있는 칸에 로봇을 보내게 된다. 그래서 슬롯마다 "이 슬롯을 볼 수 있는
   카메라가 현재 살아있는가"를 먼저 확인하고, 아무도 못 보면 상태를
   갱신하지 않고 직전 상태를 유지한다(unknown-safe).
4. **광축에서 가까운 관측이 더 정확하다.** 바닥 homography는 차량 상면에
   대해 parallax 오차를 갖고, 그 크기는 광축 지상점에서 멀수록 커진다.
   중복 검출 중 하나를 골라야 할 때는 ``axis_dist_m``이 작은 쪽을 채택한다.
"""

from __future__ import annotations

import json
import math
from collections import deque
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from cooperative_parking_robot.parking_geometry import polygon_overlap_ratio

# 검출 envelope 스키마 버전. 필드가 바뀌면 올리고, 수신측은 모르는 버전을
# 조용히 받아들이지 않고 경고한다(잘못된 좌표로 임무가 도는 것보다 낫다).
DETECTION_ENVELOPE_VERSION = 1


def target_presence_state(ready, observed_recently, perception_available=True):
    """Keep perception health distinct from an observed vehicle absence."""
    if not perception_available:
        return 'PERCEPTION_UNAVAILABLE'
    if ready:
        return 'READY'
    return 'DETECTING' if observed_recently else 'ABSENT'


def perception_is_available(camera_states, require_all_cameras=True):
    alive = [bool(state.get('alive', False))
             for state in camera_states.values()]
    return bool(alive) and (all(alive) if require_all_cameras else any(alive))


# ======================================================================
# 1. 검출 envelope 직렬화/역직렬화
# ======================================================================

def _finite_xy(name: str, point: Sequence[float]) -> Tuple[float, float]:
    if len(point) != 2:
        raise ValueError(f'{name} must contain exactly x,y')
    x = float(point[0])
    y = float(point[1])
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError(f'{name} contains NaN/Inf')
    return x, y


def _optional_float(value) -> Optional[float]:
    if value is None:
        return None
    number = float(value)
    if not math.isfinite(number):
        return None
    return number


def _polygon_or_none(name: str, polygon) -> Optional[List[Tuple[float, float]]]:
    if polygon is None:
        return None
    points = [_finite_xy(f'{name}[{i}]', p) for i, p in enumerate(polygon)]
    if len(points) < 3:
        return None
    return points


class CameraDetection:
    """한 카메라가 한 프레임에서 본 차량 하나.

    좌표는 모두 map frame metre다. ``polygon``은 segmentation mask의 world
    convex hull이며 COCO detection 모델에서는 ``None``이 될 수 있다.
    """

    __slots__ = (
        'camera_id', 'center', 'polygon', 'yaw', 'length_m', 'width_m',
        'in_waiting', 'confidence', 'axis_dist_m', 'vehicle_class',
        'classified_wheelbase_m',
    )

    def __init__(
            self,
            camera_id: str,
            center: Sequence[float],
            polygon=None,
            yaw: Optional[float] = None,
            length_m: Optional[float] = None,
            width_m: Optional[float] = None,
            in_waiting: bool = False,
            confidence: float = 0.0,
            axis_dist_m: float = 0.0,
            vehicle_class: Optional[str] = None,
            classified_wheelbase_m: Optional[float] = None):
        self.camera_id = str(camera_id)
        self.center = _finite_xy('center', center)
        self.polygon = _polygon_or_none('polygon', polygon)
        self.yaw = _optional_float(yaw)
        self.length_m = _optional_float(length_m)
        self.width_m = _optional_float(width_m)
        self.in_waiting = bool(in_waiting)
        self.confidence = float(confidence)
        self.axis_dist_m = float(axis_dist_m)
        self.vehicle_class = (
            None if vehicle_class is None else str(vehicle_class))
        self.classified_wheelbase_m = _optional_float(classified_wheelbase_m)

    def to_dict(self) -> Dict:
        return {
            'center': list(self.center),
            'polygon': (None if self.polygon is None
                        else [list(point) for point in self.polygon]),
            'yaw': self.yaw,
            'length_m': self.length_m,
            'width_m': self.width_m,
            'in_waiting': self.in_waiting,
            'confidence': self.confidence,
            'axis_dist_m': self.axis_dist_m,
            'vehicle_class': self.vehicle_class,
            'classified_wheelbase_m': self.classified_wheelbase_m,
        }

    @classmethod
    def from_dict(cls, camera_id: str, payload: Mapping) -> 'CameraDetection':
        return cls(
            camera_id=camera_id,
            center=payload['center'],
            polygon=payload.get('polygon'),
            yaw=payload.get('yaw'),
            length_m=payload.get('length_m'),
            width_m=payload.get('width_m'),
            in_waiting=payload.get('in_waiting', False),
            confidence=payload.get('confidence', 0.0),
            axis_dist_m=payload.get('axis_dist_m', 0.0),
            vehicle_class=payload.get('vehicle_class'),
            classified_wheelbase_m=payload.get('classified_wheelbase_m'),
        )

    def __repr__(self):  # pragma: no cover - 디버깅 편의용
        return (f'<CameraDetection {self.camera_id} '
                f'({self.center[0]:.3f},{self.center[1]:.3f}) '
                f'axis={self.axis_dist_m:.2f}>')


def encode_detection_envelope(
        camera_id: str,
        stamp_ns: int,
        sequence: int,
        coverage_polygon: Optional[Sequence[Sequence[float]]],
        detections: Iterable[CameraDetection],
        homography_ok: bool = True) -> str:
    """카메라 sensor 노드가 ``std_msgs/String``으로 실어 보낼 JSON을 만든다.

    커스텀 msg 패키지를 새로 만들면 세 장비 모두 재빌드해야 하고 rosdep
    의존성이 늘어난다. 검출 개수가 한 자릿수인 규모에서는 JSON String이
    충분히 싸고, ``ros2 topic echo``로 바로 눈으로 볼 수 있는 이점이 크다.
    """
    payload = {
        'version': DETECTION_ENVELOPE_VERSION,
        'camera_id': str(camera_id),
        'stamp_ns': int(stamp_ns),
        'sequence': int(sequence),
        'homography_ok': bool(homography_ok),
        'coverage_polygon': (
            None if coverage_polygon is None
            else [[float(p[0]), float(p[1])] for p in coverage_polygon]),
        'detections': [detection.to_dict() for detection in detections],
    }
    return json.dumps(payload, ensure_ascii=False)


def decode_detection_envelope(text: str) -> Dict:
    """``encode_detection_envelope``의 역변환. 실패하면 ValueError."""
    try:
        payload = json.loads(text)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'detection envelope is not valid JSON: {exc}') from exc
    if not isinstance(payload, dict):
        raise ValueError('detection envelope must be a JSON object')
    version = int(payload.get('version', 0))
    if version != DETECTION_ENVELOPE_VERSION:
        raise ValueError(
            f'unsupported detection envelope version {version} '
            f'(expected {DETECTION_ENVELOPE_VERSION})')
    camera_id = str(payload.get('camera_id', '')).strip()
    if not camera_id:
        raise ValueError('detection envelope requires a camera_id')
    stamp_ns = int(payload.get('stamp_ns', 0))
    coverage = payload.get('coverage_polygon')
    coverage_polygon = _polygon_or_none('coverage_polygon', coverage)
    raw_detections = payload.get('detections', [])
    if not isinstance(raw_detections, list):
        raise ValueError('detections must be a list')
    detections = [
        CameraDetection.from_dict(camera_id, item) for item in raw_detections]
    return {
        'version': version,
        'camera_id': camera_id,
        'stamp_ns': stamp_ns,
        'sequence': int(payload.get('sequence', 0)),
        'homography_ok': bool(payload.get('homography_ok', True)),
        'coverage_polygon': coverage_polygon,
        'detections': detections,
    }


# ======================================================================
# 2. 기하 도우미
# ======================================================================

def point_in_polygon(x: float, y: float,
                     polygon: Sequence[Sequence[float]]) -> bool:
    """경계를 포함하는 ray-casting 판정.

    ``yolo_bev_map_node.point_in_polygon``과 동일한 규칙을 쓴다. 대기영역
    판정과 coverage 판정이 노드마다 다르게 동작하면 디버깅이 불가능해진다.
    """
    count = len(polygon)
    if count < 3:
        return False
    inside = False
    for index in range(count):
        x1, y1 = float(polygon[index][0]), float(polygon[index][1])
        x2, y2 = (float(polygon[(index + 1) % count][0]),
                  float(polygon[(index + 1) % count][1]))
        cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
        if (abs(cross) <= 1e-9 and
                min(x1, x2) - 1e-9 <= x <= max(x1, x2) + 1e-9 and
                min(y1, y2) - 1e-9 <= y <= max(y1, y2) + 1e-9):
            return True
        if (y1 > y) != (y2 > y):
            hit_x = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < hit_x:
                inside = not inside
    return inside


def polygon_centroid(polygon: Sequence[Sequence[float]]) -> Tuple[float, float]:
    """단순 꼭짓점 평균. coverage polygon 대표점으로만 쓴다."""
    count = len(polygon)
    if count == 0:
        raise ValueError('polygon must not be empty')
    return (sum(float(p[0]) for p in polygon) / count,
            sum(float(p[1]) for p in polygon) / count)


def image_corner_coverage(
        homography,
        width_px: int,
        height_px: int,
        margin_px: float = 0.0,
        scale_to_m: float = 1.0) -> List[Tuple[float, float]]:
    """영상 네 귀퉁이를 H로 투영해 그 카메라의 바닥 시야 사각형을 만든다.

    coverage polygon을 사람이 손으로 재서 파라미터에 넣게 하면 반드시
    homography와 어긋난다. 어차피 H가 정의하는 게 "이 카메라가 바닥의 어디를
    보는가"이므로 H에서 직접 유도하는 편이 항상 일관된다.

    ``margin_px``는 렌즈 가장자리 왜곡 잔차와 검출 신뢰도가 떨어지는 테두리를
    잘라내기 위한 안쪽 여유다.
    """
    if width_px <= 0 or height_px <= 0:
        raise ValueError('image size must be positive')
    margin = float(margin_px)
    if margin < 0.0:
        raise ValueError('margin_px must be non-negative')
    if 2.0 * margin >= min(width_px, height_px):
        raise ValueError('margin_px is larger than half the image')
    scale = float(scale_to_m)
    if scale <= 0.0:
        raise ValueError('scale_to_m must be positive')

    left = margin
    top = margin
    right = float(width_px) - 1.0 - margin
    bottom = float(height_px) - 1.0 - margin
    corners_px = ((left, top), (right, top), (right, bottom), (left, bottom))

    matrix = [[float(homography[r][c]) for c in range(3)] for r in range(3)]
    world = []
    for px, py in corners_px:
        wx = matrix[0][0] * px + matrix[0][1] * py + matrix[0][2]
        wy = matrix[1][0] * px + matrix[1][1] * py + matrix[1][2]
        ww = matrix[2][0] * px + matrix[2][1] * py + matrix[2][2]
        if abs(ww) < 1e-12:
            raise ValueError('image corner maps to infinity under homography')
        world.append((wx / ww * scale, wy / ww * scale))
    if not all(math.isfinite(v) for point in world for v in point):
        raise ValueError('coverage polygon contains NaN/Inf')
    return world


# ======================================================================
# 3. 카메라 간 중복 검출 병합
# ======================================================================

class MergedDetection:
    """여러 카메라의 관측이 하나로 합쳐진 차량."""

    __slots__ = ('center', 'polygon', 'yaw', 'length_m', 'width_m',
                 'in_waiting', 'sources', 'primary')

    def __init__(self, primary: CameraDetection):
        self.primary = primary
        self.center = primary.center
        self.polygon = primary.polygon
        self.yaw = primary.yaw
        self.length_m = primary.length_m
        self.width_m = primary.width_m
        self.in_waiting = primary.in_waiting
        self.sources: List[str] = [primary.camera_id]

    def absorb(self, other: CameraDetection, blend: float) -> None:
        """중복으로 판정된 관측을 흡수한다.

        위치는 ``blend`` 가중치로 살짝 섞어 두 카메라 사이 경계에서 튀지 않게
        한다(blend=0이면 광축에 가까운 primary 값을 그대로 쓴다). 형상 정보는
        primary가 비어 있을 때만 보충한다 — 서로 다른 시점의 mask를 평균하면
        차량이 실제보다 커져 슬롯 적합성 판정이 보수적으로 망가진다.
        """
        if other.camera_id not in self.sources:
            self.sources.append(other.camera_id)
        if blend > 0.0:
            weight = min(max(float(blend), 0.0), 0.5)
            self.center = (
                (1.0 - weight) * self.center[0] + weight * other.center[0],
                (1.0 - weight) * self.center[1] + weight * other.center[1],
            )
        if self.polygon is None and other.polygon is not None:
            self.polygon = other.polygon
        if self.yaw is None and other.yaw is not None:
            self.yaw = other.yaw
        if self.length_m is None and other.length_m is not None:
            self.length_m = other.length_m
            self.width_m = other.width_m
        # 대기영역 판정은 OR이다. 경계에 걸친 차량을 한 카메라가 밖으로 봤다는
        # 이유로 타겟에서 제외하면 입차가 시작되지 않는다.
        self.in_waiting = self.in_waiting or other.in_waiting

    def as_dict(self) -> Dict:
        return {
            'center': list(self.center),
            'polygon': (None if self.polygon is None
                        else [list(p) for p in self.polygon]),
            'yaw': self.yaw,
            'length_m': self.length_m,
            'width_m': self.width_m,
            'in_waiting': self.in_waiting,
            'sources': list(self.sources),
        }


def _mutual_overlap(a: Sequence[Sequence[float]],
                    b: Sequence[Sequence[float]]) -> float:
    """두 polygon의 상호 겹침률 중 큰 값.

    ``polygon_overlap_ratio(subject, clip)``은 clip 면적 기준 비율을 준다.
    한쪽 mask가 잘려서 작게 나온 경우에도 중복임을 잡아내려면 양방향을 모두
    보고 큰 쪽을 써야 한다.
    """
    try:
        forward = polygon_overlap_ratio(a, b)
        backward = polygon_overlap_ratio(b, a)
    except (TypeError, ValueError):
        return 0.0
    return max(forward, backward)


def merge_detections(
        detections: Iterable[CameraDetection],
        duplicate_center_gate_m: float = 0.35,
        duplicate_overlap_ratio: float = 0.30,
        center_blend: float = 0.0) -> List[MergedDetection]:
    """여러 카메라의 검출 목록을 하나로 합친다.

    알고리즘
    --------
    1. ``axis_dist_m`` 오름차순(= 광축에 가까워 parallax 오차가 작은 순)으로
       정렬한다. 동률이면 confidence가 높은 쪽을 먼저 본다.
    2. 앞에서부터 확정 목록에 넣되, 이미 확정된 것과 (a) 중심거리가 gate
       이내이거나 (b) polygon 상호 겹침률이 임계 이상이면 중복으로 보고
       흡수시킨다.
    3. 결과 순서는 확정된 순서(정확도 높은 순)를 유지한다.

    같은 카메라 안의 서로 다른 차량 두 대가 gate 안에 들어오는 상황은
    실물 크기상 발생하지 않는다고 보고 카메라 구분 없이 dedup한다.
    """
    gate = float(duplicate_center_gate_m)
    overlap_threshold = float(duplicate_overlap_ratio)
    if gate < 0.0:
        raise ValueError('duplicate_center_gate_m must be non-negative')
    if not 0.0 <= overlap_threshold <= 1.0:
        raise ValueError('duplicate_overlap_ratio must be in [0,1]')

    ordered = sorted(
        detections,
        key=lambda d: (d.axis_dist_m, -d.confidence))

    merged: List[MergedDetection] = []
    for detection in ordered:
        duplicate_of = None
        for candidate in merged:
            distance = math.hypot(
                candidate.center[0] - detection.center[0],
                candidate.center[1] - detection.center[1])
            if distance <= gate:
                duplicate_of = candidate
                break
            if (candidate.polygon is not None and
                    detection.polygon is not None and
                    _mutual_overlap(candidate.polygon, detection.polygon)
                    >= overlap_threshold):
                duplicate_of = candidate
                break
        if duplicate_of is None:
            merged.append(MergedDetection(detection))
        else:
            duplicate_of.absorb(detection, center_blend)
    return merged


# ======================================================================
# 4. 슬롯 점유 판정 (coverage 인식형)
# ======================================================================

class SlotOccupancyTracker:
    """카메라 시야를 고려한 슬롯 점유 debounce.

    단일 카메라 시절 ``yolo_bev_map_node.publish_empty_slots``와 동일한
    debounce 규칙(점유 유지 ``hold_s``, 빈칸 확정 ``confirm_frames``)을
    쓰되, "이 슬롯을 지금 볼 수 있는 카메라가 하나도 없다"는 상태를 추가로
    구분한다. 관측자가 없으면 카운터를 건드리지 않고 직전 상태를 유지한다.

    시작 상태는 ``occupied=True``다. 아직 아무것도 못 본 시점에 빈자리로
    발행하면 로봇이 차 있는 칸으로 출발한다.
    """

    def __init__(self,
                 slot_ids: Sequence[str],
                 overlap_threshold: float = 0.10,
                 empty_confirm_frames: int = 5,
                 occupied_hold_s: float = 0.75,
                 now: float = 0.0):
        if not slot_ids:
            raise ValueError('at least one slot id is required')
        if not 0.0 <= float(overlap_threshold) <= 1.0:
            raise ValueError('overlap_threshold must be in [0,1]')
        if int(empty_confirm_frames) <= 0:
            raise ValueError('empty_confirm_frames must be positive')
        if float(occupied_hold_s) < 0.0:
            raise ValueError('occupied_hold_s must be non-negative')
        self.overlap_threshold = float(overlap_threshold)
        self.empty_confirm_frames = int(empty_confirm_frames)
        self.occupied_hold_s = float(occupied_hold_s)
        self.state: Dict[str, Dict] = {
            str(slot_id): {
                'occupied': True,
                'observed': False,
                'empty_count': 0,
                'last_occupied': float(now),
            }
            for slot_id in slot_ids
        }

    def update(self,
               slot_polygons: Mapping[str, Sequence[Sequence[float]]],
               detections: Sequence[MergedDetection],
               observable: Mapping[str, bool],
               now: float) -> Dict[str, Dict]:
        """한 사이클의 점유 상태를 갱신하고 슬롯별 상태 dict를 돌려준다."""
        for slot_id, state in self.state.items():
            if not observable.get(slot_id, False):
                # 관측자 없음 — empty_count를 0으로 되돌려 카메라가 복귀한 뒤
                # 처음부터 다시 confirm_frames를 채우게 한다.
                state['observed'] = False
                state['empty_count'] = 0
                continue
            state['observed'] = True
            polygon = slot_polygons[slot_id]
            occupied_now = False
            for detection in detections:
                if detection.polygon is not None:
                    if (polygon_overlap_ratio(detection.polygon, polygon)
                            >= self.overlap_threshold):
                        occupied_now = True
                        break
                elif point_in_polygon(
                        detection.center[0], detection.center[1], polygon):
                    # mask 없는 COCO 폴백. 중심 포함 여부만 본다.
                    occupied_now = True
                    break
            if occupied_now:
                state['occupied'] = True
                state['empty_count'] = 0
                state['last_occupied'] = float(now)
            elif float(now) - state['last_occupied'] < self.occupied_hold_s:
                state['empty_count'] = 0
            else:
                state['empty_count'] += 1
                if state['empty_count'] >= self.empty_confirm_frames:
                    state['occupied'] = False
        return self.state

    def empty_slot_ids(self) -> List[str]:
        """빈자리로 확정된 슬롯만 반환한다(관측 불가 슬롯은 절대 포함 안 함)."""
        return [
            slot_id for slot_id, state in self.state.items()
            if state['observed'] and not state['occupied']
        ]


def slot_observability(
        slot_polygons: Mapping[str, Sequence[Sequence[float]]],
        coverage_polygons: Mapping[str, Sequence[Sequence[float]]],
        require_full_slot: bool = False) -> Dict[str, bool]:
    """슬롯별로 "지금 살아있는 카메라 중 이 칸을 보는 카메라가 있는가".

    ``require_full_slot=True``면 슬롯의 네 모서리가 모두 한 카메라 안에 들어와야
    관측 가능으로 본다. 기본값(False)은 슬롯 중심만 확인한다 — 두 카메라 경계에
    걸친 슬롯이 영원히 unknown으로 남는 것을 막기 위한 실용적 기본값이다.
    """
    result: Dict[str, bool] = {}
    for slot_id, polygon in slot_polygons.items():
        centroid = polygon_centroid(polygon)
        visible = False
        for coverage in coverage_polygons.values():
            if coverage is None or len(coverage) < 3:
                continue
            if require_full_slot:
                if all(point_in_polygon(p[0], p[1], coverage)
                       for p in polygon):
                    visible = True
                    break
            elif point_in_polygon(centroid[0], centroid[1], coverage):
                visible = True
                break
        result[slot_id] = visible
    return result


def coverage_grid_values(
        width: int, height: int, resolution: float,
        coverage_polygons: Mapping[
            str, Optional[Sequence[Sequence[float]]]],
        origin_x_m: float = 0.0, origin_y_m: float = 0.0) -> List[int]:
    """Return an OccupancyGrid base layer: observed=free, unseen=unknown.

    Each cell is classified at its centre in the configured map origin.
    Obstacles are intentionally not painted here; the merge node adds them
    after constructing this conservative observation mask.
    """
    width = int(width)
    height = int(height)
    resolution = float(resolution)
    if width <= 0 or height <= 0 or not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('grid width/height/resolution must be positive')
    polygons = [
        polygon for polygon in coverage_polygons.values()
        if polygon is not None and len(polygon) >= 3
    ]
    values = [-1] * (width * height)
    for gy in range(height):
        y_m = float(origin_y_m) + (gy + 0.5) * resolution
        row = gy * width
        for gx in range(width):
            x_m = float(origin_x_m) + (gx + 0.5) * resolution
            if any(point_in_polygon(x_m, y_m, polygon)
                   for polygon in polygons):
                values[row + gx] = 0
    return values


# ======================================================================
# 5. 타겟 latch / 차량 치수 (단일 카메라 노드와 동일 규칙)
# ======================================================================

class TargetLatchTracker:
    """대기영역 차량이 ``hold_s`` 동안 ``tolerance_m`` 안에 멈춰 있으면 latch."""

    def __init__(self,
                 stationary_tolerance_m: float = 0.02,
                 stationary_hold_s: float = 2.0,
                 detection_timeout_s: float = 0.5,
                 position_filter_window: int = 1):
        if float(stationary_tolerance_m) <= 0.0:
            raise ValueError('stationary_tolerance_m must be positive')
        if float(stationary_hold_s) < 0.0:
            raise ValueError('stationary_hold_s must be non-negative')
        if float(detection_timeout_s) < 0.0:
            raise ValueError('detection_timeout_s must be non-negative')
        if int(position_filter_window) <= 0:
            raise ValueError('position_filter_window must be positive')
        self.tolerance = float(stationary_tolerance_m)
        self.hold_s = float(stationary_hold_s)
        self.timeout_s = float(detection_timeout_s)
        self.position_filter_window = int(position_filter_window)
        self._position_history = deque(maxlen=self.position_filter_window)
        self.latched: Optional[Tuple[float, float]] = None
        self.candidate: Optional[Tuple[float, float]] = None
        self.anchor: Optional[Tuple[float, float]] = None
        self.stable_since: Optional[float] = None
        self.last_seen: float = 0.0
        self.just_latched: bool = False

    def reset(self) -> None:
        self.latched = None
        self.candidate = None
        self.anchor = None
        self.stable_since = None
        self.last_seen = 0.0
        self.just_latched = False
        self._position_history.clear()

    def _filtered_point(
            self, target: Sequence[float]) -> Tuple[float, float]:
        """Median-filter recent world centres without hiding long motion."""
        point = (float(target[0]), float(target[1]))
        self._position_history.append(point)

        def median(values):
            ordered = sorted(values)
            middle = len(ordered) // 2
            if len(ordered) % 2:
                return ordered[middle]
            return 0.5 * (ordered[middle - 1] + ordered[middle])

        return (
            median([value[0] for value in self._position_history]),
            median([value[1] for value in self._position_history]),
        )

    def update(self, target: Optional[Sequence[float]],
               now: float, preserve_latched: bool = False
               ) -> Optional[Tuple[float, float]]:
        self.just_latched = False
        if self.latched is not None:
            if preserve_latched:
                return self.latched
            if target is not None:
                # A visible target that actually moved must revoke READY
                # immediately. detection_timeout_s is only a grace period for
                # missed frames, not permission for a moving car to stay READY.
                point = (float(target[0]), float(target[1]))
                if math.hypot(
                        point[0] - self.latched[0],
                        point[1] - self.latched[1]) <= self.tolerance:
                    self.last_seen = float(now)
                    return self.latched
                self.reset()
            elif float(now) - self.last_seen <= self.timeout_s:
                return self.latched
            else:
                self.reset()
        if target is None:
            if float(now) - self.last_seen > self.timeout_s:
                self.candidate = None
                self.anchor = None
                self.stable_since = None
            return None
        point = self._filtered_point(target)
        self.last_seen = float(now)
        if self.anchor is None or math.hypot(
                point[0] - self.anchor[0],
                point[1] - self.anchor[1]) > self.tolerance:
            self.anchor = point
            self.candidate = point
            self.stable_since = float(now)
            return None
        self.candidate = (
            0.8 * self.candidate[0] + 0.2 * point[0],
            0.8 * self.candidate[1] + 0.2 * point[1],
        )
        if float(now) - self.stable_since >= self.hold_s:
            self.latched = self.candidate
            self.just_latched = True
        return self.latched


class VehicleDimensionTracker:
    """차량 mask의 길이/폭/yaw를 검증하고 EMA로 안정화한다."""

    def __init__(self,
                 default_length_m: float,
                 default_width_m: float,
                 padding_m: float = 0.03,
                 length_range_m: Sequence[float] = (0.30, 6.50),
                 width_range_m: Sequence[float] = (0.20, 2.80),
                 dimension_alpha: float = 0.20,
                 yaw_alpha: float = 0.15):
        if float(default_length_m) <= 0.0 or float(default_width_m) <= 0.0:
            raise ValueError('default vehicle dimensions must be positive')
        if float(padding_m) < 0.0:
            raise ValueError('padding_m must be non-negative')
        if len(length_range_m) != 2 or len(width_range_m) != 2:
            raise ValueError('dimension ranges must contain two values')
        if not 0.0 < float(dimension_alpha) <= 1.0:
            raise ValueError('dimension_alpha must be in (0,1]')
        if not 0.0 < float(yaw_alpha) <= 1.0:
            raise ValueError('yaw_alpha must be in (0,1]')
        self.default_length = float(default_length_m)
        self.default_width = float(default_width_m)
        self.padding = float(padding_m)
        self.length_range = (float(length_range_m[0]), float(length_range_m[1]))
        self.width_range = (float(width_range_m[0]), float(width_range_m[1]))
        self.dimension_alpha = float(dimension_alpha)
        self.yaw_alpha = float(yaw_alpha)
        self.length_m = self.default_length
        self.width_m = self.default_width
        self.dimension_valid = False
        self.yaw = 0.0
        self.yaw_valid = False

    def reset(self) -> None:
        self.length_m = self.default_length
        self.width_m = self.default_width
        self.dimension_valid = False
        self.yaw = 0.0
        self.yaw_valid = False

    def update_dimensions(self, measured_length, measured_width) -> bool:
        if measured_length is None or measured_width is None:
            return False
        length = float(measured_length) + 2.0 * self.padding
        width = float(measured_width) + 2.0 * self.padding
        if not self.length_range[0] <= length <= self.length_range[1]:
            return False
        if not self.width_range[0] <= width <= self.width_range[1]:
            return False
        if not self.dimension_valid:
            self.length_m = length
            self.width_m = width
            self.dimension_valid = True
            return True
        alpha = self.dimension_alpha
        self.length_m = (1.0 - alpha) * self.length_m + alpha * length
        self.width_m = (1.0 - alpha) * self.width_m + alpha * width
        return True

    def update_yaw(self, measured_yaw) -> None:
        """장축 yaw는 yaw와 yaw+pi가 같은 축이므로 2*yaw 벡터를 평균한다."""
        if measured_yaw is None:
            return
        yaw = float(measured_yaw)
        if not math.isfinite(yaw):
            return
        if not self.yaw_valid:
            self.yaw = yaw
            self.yaw_valid = True
            return
        alpha = self.yaw_alpha
        x = ((1.0 - alpha) * math.cos(2.0 * self.yaw) +
             alpha * math.cos(2.0 * yaw))
        y = ((1.0 - alpha) * math.sin(2.0 * self.yaw) +
             alpha * math.sin(2.0 * yaw))
        self.yaw = 0.5 * math.atan2(y, x)


# ======================================================================
# 6. 진단 요약
# ======================================================================

def summarize_merge(
        camera_states: Mapping[str, Mapping],
        merged: Sequence[MergedDetection],
        slot_state: Mapping[str, Mapping],
        stamp_ns: int) -> str:
    """``/cctv/merge_status``로 발행할 진단 JSON.

    현장에서 "빈자리가 왜 안 뜨지"를 5초 안에 판단할 수 있어야 한다. 카메라별
    생존 여부, 중복 제거 통계, 슬롯별 관측 여부를 한 번에 담는다.
    """
    duplicate_count = sum(
        len(detection.sources) - 1 for detection in merged)
    return json.dumps({
        'stamp_ns': int(stamp_ns),
        'cameras': {
            camera_id: {
                'alive': bool(state.get('alive', False)),
                'age_s': round(float(state.get('age_s', 0.0)), 3),
                'detections': int(state.get('detections', 0)),
                'coverage_ready': bool(state.get('coverage_ready', False)),
            }
            for camera_id, state in camera_states.items()
        },
        'merged_detections': len(merged),
        'duplicates_removed': duplicate_count,
        'multi_camera_detections': sum(
            1 for detection in merged if len(detection.sources) > 1),
        'slots': {
            slot_id: {
                'observed': bool(state.get('observed', False)),
                'occupied': bool(state.get('occupied', True)),
            }
            for slot_id, state in slot_state.items()
        },
    }, ensure_ascii=False)
