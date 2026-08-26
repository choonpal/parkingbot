#!/usr/bin/env python3
"""주차면 적합성 및 메카넘 협동 운반체의 최종 주차 기하 계산.

이 파일은 ROS 메시지, OpenCV, YOLO에 의존하지 않는 순수 계산 모듈이다.
따라서 화면에서 클릭한 주차면 네 모서리를 등록하는 도구, BEV 인지 노드,
``fleet_manager_node``가 같은 계산을 공유할 수 있다.

좌표/방향 정의
----------------

* 모든 위치는 ``map`` 좌표계의 metre 단위다.
* ``ParkingSlot.entry_yaw_rad``는 **통로에서 주차면 안쪽으로 들어가는 방향**이다.
* 슬롯 ``length_m``은 entry yaw 축, ``width_m``은 그 수직축의 길이다.
* 슬롯의 긴 축은 180도 대칭이지만 차량 앞뒤는 대칭이 아니므로, 실제 목표 yaw는
  정방향/후진/최소 회전 정책 중 하나로 명시해서 고른다.

메카넘 운반체는 경로 방향을 보기 위해 계속 회전할 필요가 없다. 가장 단순하고
검증 가능한 주차 절차는 ``통로 주행 -> 슬롯 앞 정렬점 -> 제자리 회전 -> 직선
삽입``이다. 이 모듈은 그 세 자세를 계산할 뿐, 장애물 충돌이나 회전 공간의
OccupancyGrid 검사는 플래너가 별도로 수행해야 한다.
"""

from dataclasses import dataclass
import math
from typing import Iterable, Sequence, Tuple


Point2D = Tuple[float, float]


def _finite(name, value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _positive(name, value):
    value = _finite(name, value)
    if value <= 0.0:
        raise ValueError(f"{name} must be positive")
    return value


def grid_cell_count(extent_m, resolution_m):
    """요청한 metric 범위를 빠짐없이 덮는 OccupancyGrid 셀 수."""
    extent = _positive("extent_m", extent_m)
    resolution = _positive("resolution_m", resolution_m)
    return max(1, int(math.ceil(extent / resolution)))


def _nonnegative(name, value):
    value = _finite(name, value)
    if value < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return value


def normalize_angle(angle_rad):
    """각도를 ``[-pi, pi]`` 범위로 정규화한다."""
    angle = _finite("angle_rad", angle_rad)
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_error(target_yaw_rad, current_yaw_rad):
    """현재 yaw에서 목표 yaw까지의 최단 회전량을 반환한다."""
    return normalize_angle(
        _finite("target_yaw_rad", target_yaw_rad)
        - _finite("current_yaw_rad", current_yaw_rad))


@dataclass(frozen=True)
class Pose2D:
    """ROS와 무관한 map-frame 2차원 자세."""

    x_m: float
    y_m: float
    yaw_rad: float

    def __post_init__(self):
        object.__setattr__(self, "x_m", _finite("x_m", self.x_m))
        object.__setattr__(self, "y_m", _finite("y_m", self.y_m))
        object.__setattr__(
            self, "yaw_rad", normalize_angle(self.yaw_rad))

    @property
    def position(self):
        return (self.x_m, self.y_m)


@dataclass(frozen=True)
class RegisteredSlot:
    """BEV에 등록된 하나의 직사각형 주차면."""

    slot_id: str
    center_x_m: float
    center_y_m: float
    length_m: float
    width_m: float
    entry_yaw_rad: float

    def __post_init__(self):
        object.__setattr__(self, "slot_id", str(self.slot_id))
        object.__setattr__(
            self, "center_x_m", _finite("center_x_m", self.center_x_m))
        object.__setattr__(
            self, "center_y_m", _finite("center_y_m", self.center_y_m))
        object.__setattr__(
            self, "length_m", _positive("length_m", self.length_m))
        object.__setattr__(
            self, "width_m", _positive("width_m", self.width_m))
        object.__setattr__(
            self, "entry_yaw_rad", normalize_angle(self.entry_yaw_rad))
        if self.length_m < self.width_m:
            raise ValueError(
                "slot length_m must be the long axis (length >= width)")

    @property
    def center(self):
        return (self.center_x_m, self.center_y_m)

    @property
    def entry_unit(self):
        """통로에서 슬롯 중심 쪽으로 향하는 단위벡터."""
        return (math.cos(self.entry_yaw_rad),
                math.sin(self.entry_yaw_rad))


# 기존 코드에서 읽기 쉬운 이름과 등록 도구에서 명시적인 이름을 모두 제공한다.
# 두 이름은 완전히 같은 dataclass를 가리키므로 isinstance 검사도 일관된다.
ParkingSlot = RegisteredSlot


@dataclass(frozen=True)
class FitResult:
    """직사각형 footprint와 주차면의 축별 여유량."""

    fits: bool
    reason: str
    required_length_m: float
    required_width_m: float
    length_clearance_m: float
    width_clearance_m: float


@dataclass(frozen=True)
class ParkingManeuver:
    """통로에서 목표 주차 자세까지의 최소 단계형 기하 계획.

    ``staging_before_rotation``까지는 현재 yaw를 유지하며 이동한다.
    같은 위치에서 ``staging_aligned`` yaw로 회전한 뒤, 슬롯 중심의
    ``target_pose``까지 entry 축을 따라 평행이동한다.
    """

    slot_id: str
    fit: FitResult
    staging_before_rotation: Pose2D
    staging_aligned: Pose2D
    target_pose: Pose2D
    yaw_change_rad: float
    needs_rotation: bool
    rotation_sweep_radius_m: float
    rotation_fits_inside_slot: bool

    @property
    def insertion_segment(self):
        """정렬 후 직선 삽입 구간의 시작/끝 자세."""
        return (self.staging_aligned, self.target_pose)


@dataclass(frozen=True)
class ApproachCandidate:
    """등록된 한 입구에서 가능한 차량 앞방향 후보 하나."""

    parking_direction: str
    staging_pose: Pose2D
    target_pose: Pose2D
    yaw_change_rad: float


def _validated_points(corners: Iterable[Sequence[float]]):
    points = []
    for index, point in enumerate(corners):
        if len(point) != 2:
            raise ValueError(f"corner[{index}] must contain x,y")
        points.append((
            _finite(f"corner[{index}].x", point[0]),
            _finite(f"corner[{index}].y", point[1]),
        ))
    if len(points) != 4:
        raise ValueError("exactly four slot corners are required")
    if len(set(points)) != 4:
        raise ValueError("slot corners must be unique")
    return points


def slot_from_corners(slot_id, corners, aisle_point):
    """클릭한 네 모서리와 통로 기준점으로 ``ParkingSlot``을 만든다.

    모서리는 어떤 순서로 전달해도 중심각으로 다시 정렬한다. 다만 네 점은
    직사각형 주차선의 모서리를 따라 찍어야 한다. ``aisle_point``는 슬롯의 열린
    입구 바깥 통로에 한 점을 찍은 값이다. 긴 축의 부호를 이 점에서 슬롯 중심
    방향으로 맞추므로, 이후 정방향/후진 진입점을 일관되게 계산할 수 있다.
    """
    points = _validated_points(corners)
    if len(aisle_point) != 2:
        raise ValueError("aisle_point must contain x,y")
    aisle = (
        _finite("aisle_point.x", aisle_point[0]),
        _finite("aisle_point.y", aisle_point[1]),
    )

    cx = sum(p[0] for p in points) / 4.0
    cy = sum(p[1] for p in points) / 4.0
    ordered = sorted(points, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))
    edges = [
        (ordered[(i + 1) % 4][0] - ordered[i][0],
         ordered[(i + 1) % 4][1] - ordered[i][1])
        for i in range(4)
    ]
    edge_lengths = [math.hypot(dx, dy) for dx, dy in edges]
    if min(edge_lengths) <= 1e-9:
        raise ValueError("slot corner edges must have non-zero length")

    # 마주 보는 두 변을 한 쌍으로 묶어, 긴 변 쌍이 슬롯 종축이 되게 한다.
    pair_0 = 0.5 * (edge_lengths[0] + edge_lengths[2])
    pair_1 = 0.5 * (edge_lengths[1] + edge_lengths[3])
    if math.isclose(pair_0, pair_1, rel_tol=1e-6, abs_tol=1e-9):
        raise ValueError("square-like slot has no unambiguous long axis")
    long_index = 0 if pair_0 > pair_1 else 1

    def unit(edge, length):
        return (edge[0] / length, edge[1] / length)

    u1 = unit(edges[long_index], edge_lengths[long_index])
    opposite = (long_index + 2) % 4
    u2 = unit(edges[opposite], edge_lengths[opposite])
    # 둘레를 같은 방향으로 돌면 마주 보는 변 벡터는 반대이므로 부호를 맞춘다.
    if u1[0] * u2[0] + u1[1] * u2[1] < 0.0:
        u2 = (-u2[0], -u2[1])
    ux, uy = u1[0] + u2[0], u1[1] + u2[1]
    norm = math.hypot(ux, uy)
    if norm <= 1e-9:
        raise ValueError("opposite slot edges do not define a stable axis")
    ux, uy = ux / norm, uy / norm

    # 약간 비뚤게 클릭해도 네 점 전체를 포함하도록 투영 범위로 치수를 구한다.
    vx, vy = -uy, ux
    long_proj = [p[0] * ux + p[1] * uy for p in points]
    lat_proj = [p[0] * vx + p[1] * vy for p in points]
    length = max(long_proj) - min(long_proj)
    width = max(lat_proj) - min(lat_proj)
    if length < width:
        # 수치적으로 축 선택이 뒤집힌 경우에도 데이터 의미를 보존한다.
        ux, uy, vx, vy = vx, vy, -ux, -uy
        length, width = width, length

    toward_center = (cx - aisle[0], cy - aisle[1])
    aisle_distance = math.hypot(*toward_center)
    if aisle_distance <= 1e-9:
        raise ValueError("aisle_point must differ from slot center")
    axial = ux * toward_center[0] + uy * toward_center[1]
    if abs(axial) <= 1e-6 * aisle_distance:
        raise ValueError(
            "aisle_point must lie near a longitudinal entry side")
    if axial < 0.0:
        ux, uy = -ux, -uy

    return ParkingSlot(
        slot_id=str(slot_id),
        center_x_m=cx,
        center_y_m=cy,
        length_m=length,
        width_m=width,
        entry_yaw_rad=math.atan2(uy, ux),
    )


def build_slot(
        slot_id, *, corners=None, aisle_point=None,
        center=None, size=None, yaw_deg=None, yaw_rad=None):
    """클릭 모서리 또는 중심/치수/yaw로 등록 슬롯 하나를 만든다.

    사용 예::

        build_slot("P1", corners=[...], aisle_point=(0.5, 2.0))
        build_slot("P1", center=(3.0, 2.0), size=(5.0, 2.4), yaw_deg=90)

    ``size`` 순서는 ``(length_m, width_m)``다. 모서리 모드에는 긴 축의
    어느 쪽이 열린 입구인지 정할 ``aisle_point``가 반드시 필요하다.
    """
    corner_mode = corners is not None
    metric_mode = center is not None or size is not None
    if corner_mode == metric_mode:
        raise ValueError(
            "provide either corners or center/size/yaw, not both")
    if corner_mode:
        if aisle_point is None:
            raise ValueError("aisle_point is required with corners")
        if any(value is not None for value in (
                center, size, yaw_deg, yaw_rad)):
            raise ValueError(
                "center/size/yaw are not used with corner registration")
        return slot_from_corners(slot_id, corners, aisle_point)

    if center is None or size is None:
        raise ValueError("center and size are both required")
    if len(center) != 2 or len(size) != 2:
        raise ValueError("center and size must each contain two values")
    if (yaw_deg is None) == (yaw_rad is None):
        raise ValueError("provide exactly one of yaw_deg or yaw_rad")
    yaw = (math.radians(_finite("yaw_deg", yaw_deg))
           if yaw_deg is not None else _finite("yaw_rad", yaw_rad))
    return ParkingSlot(
        slot_id=str(slot_id),
        center_x_m=center[0],
        center_y_m=center[1],
        length_m=size[0],
        width_m=size[1],
        entry_yaw_rad=yaw,
    )


def parse_registered_slots(slot_ids, flat_coords, flat_sizes, yaws_deg):
    """ROS2의 평탄한 YAML 배열을 ``ParkingSlot`` 목록으로 변환한다.

    * ``slot_ids``: ``["P1", "P2", ...]``
    * ``flat_coords``: ``[cx1, cy1, cx2, cy2, ...]``
    * ``flat_sizes``: ``[length1, width1, length2, width2, ...]``
    * ``yaws_deg``: 통로에서 슬롯 안쪽을 향하는 각 슬롯의 yaw(deg)
    """
    ids = list(slot_ids)
    coords = list(flat_coords)
    sizes = list(flat_sizes)
    yaws = list(yaws_deg)
    count = len(ids)
    if count == 0:
        return []
    if len(coords) != 2 * count:
        raise ValueError("flat_coords must contain two values per slot")
    if len(sizes) != 2 * count:
        raise ValueError("flat_sizes must contain length,width per slot")
    if len(yaws) != count:
        raise ValueError("yaws_deg must contain one value per slot")
    if len(set(str(slot_id) for slot_id in ids)) != count:
        raise ValueError("slot_ids must be unique")

    return [
        build_slot(
            ids[i],
            center=(coords[2 * i], coords[2 * i + 1]),
            size=(sizes[2 * i], sizes[2 * i + 1]),
            yaw_deg=yaws[i],
        )
        for i in range(count)
    ]


def slot_polygon(slot):
    """등록 슬롯을 반시계 방향 네 모서리 polygon으로 변환한다."""
    if not isinstance(slot, ParkingSlot):
        raise TypeError("slot must be ParkingSlot")
    ux, uy = slot.entry_unit
    vx, vy = -uy, ux
    hl, hw = slot.length_m / 2.0, slot.width_m / 2.0
    # local (-length,-width) -> (+length,-width) -> ... 순서(CCW)
    return tuple(
        (slot.center_x_m + sl * hl * ux + sw * hw * vx,
         slot.center_y_m + sl * hl * uy + sw * hw * vy)
        for sl, sw in ((-1.0, -1.0), (1.0, -1.0),
                       (1.0, 1.0), (-1.0, 1.0))
    )


def _polygon_points(name, polygon):
    points = []
    for index, point in enumerate(polygon):
        if len(point) != 2:
            raise ValueError(f"{name}[{index}] must contain x,y")
        points.append((
            _finite(f"{name}[{index}].x", point[0]),
            _finite(f"{name}[{index}].y", point[1]),
        ))
    # OpenCV contour처럼 마지막에 첫 점을 반복한 입력도 허용한다.
    if len(points) >= 2 and points[0] == points[-1]:
        points.pop()
    if len(points) < 3:
        raise ValueError(f"{name} must contain at least three points")
    return points


def _signed_polygon_area(points):
    return 0.5 * sum(
        points[i][0] * points[(i + 1) % len(points)][1]
        - points[(i + 1) % len(points)][0] * points[i][1]
        for i in range(len(points)))


def _cross(ax, ay, bx, by):
    return ax * by - ay * bx


def _line_intersection(segment_start, segment_end, clip_start, clip_end):
    sx, sy = segment_start
    ex, ey = segment_end
    ax, ay = clip_start
    bx, by = clip_end
    dx, dy = ex - sx, ey - sy
    qx, qy = bx - ax, by - ay
    denominator = _cross(dx, dy, qx, qy)
    if abs(denominator) <= 1e-12:
        # inside/outside 전환인데 평행한 경우는 수치오차 상황이다. 끝점을
        # 사용하면 clipping 루프를 안정적으로 계속할 수 있다.
        return segment_end
    t = _cross(ax - sx, ay - sy, qx, qy) / denominator
    return (sx + t * dx, sy + t * dy)


def polygon_overlap_ratio(subject, slot_polygon_points):
    """convex clipping으로 ``교집합 면적 / 주차면 면적``을 계산한다.

    ``subject``는 YOLO 차량 mask의 convex hull 또는 차량 회전사각형이며,
    ``slot_polygon_points``는 클릭 등록된 볼록 사각형이어야 한다. 결과는
    0~1이고, 빈자리 판정에서는 이 비율이 임계값보다 크면 점유로 본다.
    """
    output = _polygon_points("subject", subject)
    clip = _polygon_points("slot_polygon", slot_polygon_points)
    clip_area_signed = _signed_polygon_area(clip)
    if abs(clip_area_signed) <= 1e-12:
        raise ValueError("slot_polygon area must be non-zero")
    # Sutherland-Hodgman inside 판정을 위해 clip polygon을 CCW로 맞춘다.
    if clip_area_signed < 0.0:
        clip.reverse()
    clip_area = abs(_signed_polygon_area(clip))

    for i, clip_start in enumerate(clip):
        if not output:
            return 0.0
        clip_end = clip[(i + 1) % len(clip)]
        input_points = output
        output = []

        def inside(point):
            return _cross(
                clip_end[0] - clip_start[0],
                clip_end[1] - clip_start[1],
                point[0] - clip_start[0],
                point[1] - clip_start[1]) >= -1e-12

        previous = input_points[-1]
        previous_inside = inside(previous)
        for current in input_points:
            current_inside = inside(current)
            if current_inside:
                if not previous_inside:
                    output.append(_line_intersection(
                        previous, current, clip_start, clip_end))
                output.append(current)
            elif previous_inside:
                output.append(_line_intersection(
                    previous, current, clip_start, clip_end))
            previous = current
            previous_inside = current_inside

    if len(output) < 3:
        return 0.0
    intersection_area = abs(_signed_polygon_area(output))
    return max(0.0, min(1.0, intersection_area / clip_area))


def check_slot_fit(
        slot,
        footprint_length_m,
        footprint_width_m,
        longitudinal_margin_m=0.0,
        lateral_margin_m=0.0):
    """정렬 완료된 footprint가 주차면 안에 들어가는지 검사한다.

    margin은 양쪽 각각에 필요한 거리다. 예를 들어 좌우 0.10 m씩 띄우려면
    ``lateral_margin_m=0.10``으로 호출하며, 요구 폭에는 총 0.20 m가 더해진다.
    운반 중에는 차량 몸체보다 큰 ``LoadedFootprint`` 치수를 넣어야 로봇 두 대가
    붙은 상태까지 검사할 수 있다.
    """
    if not isinstance(slot, ParkingSlot):
        raise TypeError("slot must be ParkingSlot")
    length = _positive("footprint_length_m", footprint_length_m)
    width = _positive("footprint_width_m", footprint_width_m)
    long_margin = _nonnegative(
        "longitudinal_margin_m", longitudinal_margin_m)
    lat_margin = _nonnegative("lateral_margin_m", lateral_margin_m)

    required_length = length + 2.0 * long_margin
    required_width = width + 2.0 * lat_margin
    length_clearance = slot.length_m - required_length
    width_clearance = slot.width_m - required_width
    length_ok = length_clearance >= -1e-9
    width_ok = width_clearance >= -1e-9
    if length_ok and width_ok:
        reason = "OK"
    elif not length_ok and not width_ok:
        reason = "SLOT_TOO_SHORT_AND_NARROW"
    elif not length_ok:
        reason = "SLOT_TOO_SHORT"
    else:
        reason = "SLOT_TOO_NARROW"
    return FitResult(
        fits=length_ok and width_ok,
        reason=reason,
        required_length_m=required_length,
        required_width_m=required_width,
        length_clearance_m=length_clearance,
        width_clearance_m=width_clearance,
    )


def slot_fits(
        footprint_length_m, footprint_width_m, slot, clearance_m=0.0):
    """균일한 사방 여유거리를 적용한 간단한 bool 적합성 API."""
    clearance = _nonnegative("clearance_m", clearance_m)
    return check_slot_fit(
        slot,
        footprint_length_m,
        footprint_width_m,
        longitudinal_margin_m=clearance,
        lateral_margin_m=clearance,
    ).fits


def choose_target_yaw(slot, current_yaw_rad, parking_direction="minimum_rotation"):
    """슬롯 축의 180도 대칭 중 실제 차량 목표 yaw를 선택한다.

    ``forward``는 차량 앞이 통로에서 슬롯 안쪽을 향하고, ``reverse``는 차량
    앞이 통로를 향한다. ``minimum_rotation``은 현재 자세에서 회전량이 작은 쪽을
    택한다. 운반체가 앞뒤 어느 방향으로도 움직일 수 있으므로 기본값은 최소 회전이다.
    """
    if not isinstance(slot, ParkingSlot):
        raise TypeError("slot must be ParkingSlot")
    current = normalize_angle(current_yaw_rad)
    forward = slot.entry_yaw_rad
    reverse = normalize_angle(slot.entry_yaw_rad + math.pi)
    policy = str(parking_direction).strip().lower()
    if policy == "forward":
        return forward
    if policy == "reverse":
        return reverse
    if policy != "minimum_rotation":
        raise ValueError(
            "parking_direction must be forward, reverse, or minimum_rotation")
    forward_error = abs(angle_error(forward, current))
    reverse_error = abs(angle_error(reverse, current))
    return forward if forward_error <= reverse_error else reverse


def nearest_axis_yaw(slot_yaw_rad, current_yaw_rad):
    """주차축 yaw와 yaw+pi 중 현재 자세에서 가까운 값을 반환한다."""
    current = normalize_angle(current_yaw_rad)
    forward = normalize_angle(slot_yaw_rad)
    reverse = normalize_angle(forward + math.pi)
    return (forward
            if abs(angle_error(forward, current))
            <= abs(angle_error(reverse, current))
            else reverse)


def make_approach_candidates(
        slot, loaded_length_m, gap_m, current_yaw_rad):
    """등록된 열린 입구에서 정방향/후진 정렬 후보를 회전량 순으로 만든다.

    두 후보의 staging 위치는 같다. 차이는 차량 앞이 슬롯 안쪽을 향하는지
    (``forward``), 통로를 향하는지(``reverse``)뿐이다. 메카넘은 둘 다 삽입할
    수 있으므로 fleet manager가 경로/정책에 맞는 후보를 선택할 수 있다.
    """
    if not isinstance(slot, ParkingSlot):
        raise TypeError("slot must be ParkingSlot")
    length = _positive("loaded_length_m", loaded_length_m)
    gap = _nonnegative("gap_m", gap_m)
    current = normalize_angle(current_yaw_rad)
    ux, uy = slot.entry_unit
    offset = slot.length_m / 2.0 + length / 2.0 + gap
    stage_x = slot.center_x_m - ux * offset
    stage_y = slot.center_y_m - uy * offset

    candidates = []
    for direction, target_yaw in (
            ("forward", slot.entry_yaw_rad),
            ("reverse", normalize_angle(slot.entry_yaw_rad + math.pi))):
        yaw_change = angle_error(target_yaw, current)
        candidates.append(ApproachCandidate(
            parking_direction=direction,
            staging_pose=Pose2D(stage_x, stage_y, target_yaw),
            target_pose=Pose2D(
                slot.center_x_m, slot.center_y_m, target_yaw),
            yaw_change_rad=yaw_change,
        ))
    candidates.sort(key=lambda candidate: (
        abs(candidate.yaw_change_rad),
        0 if candidate.parking_direction == "forward" else 1))
    return candidates


def footprint_extents_in_slot_axes(
        footprint_length_m, footprint_width_m, relative_yaw_rad):
    """회전된 직사각형을 슬롯 종/횡축에 투영한 전체 길이와 폭."""
    length = _positive("footprint_length_m", footprint_length_m)
    width = _positive("footprint_width_m", footprint_width_m)
    angle = _finite("relative_yaw_rad", relative_yaw_rad)
    c, s = abs(math.cos(angle)), abs(math.sin(angle))
    return (length * c + width * s,
            length * s + width * c)


def _projection_max_on_interval(a, b, low, high):
    """``a|cos(t)| + b|sin(t)|``의 닫힌 구간 정확한 최댓값."""
    if high < low:
        low, high = high, low
    candidates = [low, high]
    alpha = math.atan2(b, a)
    # 함수 주기는 pi이며 각 주기마다 alpha, pi-alpha에서 극댓값이 난다.
    start_k = int(math.floor(low / math.pi)) - 2
    end_k = int(math.ceil(high / math.pi)) + 2
    for k in range(start_k, end_k + 1):
        for candidate in (
                k * math.pi + alpha,
                k * math.pi + math.pi - alpha):
            if low <= candidate <= high:
                candidates.append(candidate)
    return max(a * abs(math.cos(t)) + b * abs(math.sin(t))
               for t in candidates)


def check_rotation_sweep_fit(
        slot,
        footprint_length_m,
        footprint_width_m,
        start_yaw_rad,
        target_yaw_rad,
        longitudinal_margin_m=0.0,
        lateral_margin_m=0.0):
    """슬롯 중심에서 start->target 최단 회전을 모두 수행할 공간이 있는지 검사.

    최종 자세 한 장면만 검사하는 ``check_slot_fit``과 달리, 회전 도중 대각선으로
    가장 커지는 투영 크기까지 확인한다. 일반적인 길고 좁은 주차면에서는 False가
    정상이며, 그 경우 슬롯 안이 아니라 통로의 정렬점에서 먼저 회전해야 한다.
    """
    if not isinstance(slot, ParkingSlot):
        raise TypeError("slot must be ParkingSlot")
    length = _positive("footprint_length_m", footprint_length_m)
    width = _positive("footprint_width_m", footprint_width_m)
    long_margin = _nonnegative(
        "longitudinal_margin_m", longitudinal_margin_m)
    lat_margin = _nonnegative("lateral_margin_m", lateral_margin_m)
    start = _finite("start_yaw_rad", start_yaw_rad)
    target = _finite("target_yaw_rad", target_yaw_rad)

    rotation = angle_error(target, start)
    relative_start = start - slot.entry_yaw_rad
    relative_end = relative_start + rotation
    low, high = sorted((relative_start, relative_end))
    swept_length = _projection_max_on_interval(length, width, low, high)
    swept_width = _projection_max_on_interval(width, length, low, high)
    required_length = swept_length + 2.0 * long_margin
    required_width = swept_width + 2.0 * lat_margin
    length_clearance = slot.length_m - required_length
    width_clearance = slot.width_m - required_width
    length_ok = length_clearance >= -1e-9
    width_ok = width_clearance >= -1e-9
    if length_ok and width_ok:
        reason = "OK"
    elif not length_ok and not width_ok:
        reason = "ROTATION_SWEEP_TOO_LONG_AND_WIDE"
    elif not length_ok:
        reason = "ROTATION_SWEEP_TOO_LONG"
    else:
        reason = "ROTATION_SWEEP_TOO_WIDE"
    return FitResult(
        fits=length_ok and width_ok,
        reason=reason,
        required_length_m=required_length,
        required_width_m=required_width,
        length_clearance_m=length_clearance,
        width_clearance_m=width_clearance,
    )


def plan_mecanum_parking(
        slot,
        current_pose,
        footprint_length_m,
        footprint_width_m,
        longitudinal_margin_m=0.0,
        lateral_margin_m=0.0,
        staging_gap_m=0.10,
        yaw_tolerance_rad=math.radians(3.0),
        parking_direction="minimum_rotation"):
    """적합한 슬롯에 대해 정렬점, 회전자세, 최종자세를 계산한다.

    이 함수가 반환하는 순서 자체가 제어 FSM의 의도다.

    1. 현재 yaw를 유지한 채 ``staging_before_rotation``으로 이동
    2. 같은 위치에서 ``staging_aligned`` yaw로 제자리 회전
    3. entry 축을 따라 ``target_pose``로 저속 직선 삽입

    회전 정렬점 주변에는 ``rotation_sweep_radius_m`` 원 전체가 장애물/맵 경계와
    겹치지 않는지 별도 검사해야 한다. 이 원 검사를 통과시키지 않고 회전 명령을
    내리면 2D A* 경로만 안전해도 운반체 모서리가 충돌할 수 있다.
    """
    if not isinstance(slot, ParkingSlot):
        raise TypeError("slot must be ParkingSlot")
    if not isinstance(current_pose, Pose2D):
        raise TypeError("current_pose must be Pose2D")
    length = _positive("footprint_length_m", footprint_length_m)
    width = _positive("footprint_width_m", footprint_width_m)
    staging_gap = _nonnegative("staging_gap_m", staging_gap_m)
    yaw_tolerance = _nonnegative("yaw_tolerance_rad", yaw_tolerance_rad)
    long_margin = _nonnegative(
        "longitudinal_margin_m", longitudinal_margin_m)
    lat_margin = _nonnegative("lateral_margin_m", lateral_margin_m)

    fit = check_slot_fit(
        slot, length, width,
        long_margin, lat_margin)
    if not fit.fits:
        raise ValueError(
            f"footprint does not fit slot {slot.slot_id}: {fit.reason}")

    target_yaw = choose_target_yaw(
        slot, current_pose.yaw_rad, parking_direction)
    yaw_change = angle_error(target_yaw, current_pose.yaw_rad)
    ux, uy = slot.entry_unit
    # 슬롯 열린 경계에서 운반체 앞/뒤 끝까지 staging_gap을 확보한다.
    staging_offset = (
        slot.length_m / 2.0 + length / 2.0 + staging_gap)
    stage_x = slot.center_x_m - ux * staging_offset
    stage_y = slot.center_y_m - uy * staging_offset

    rotation_fit = check_rotation_sweep_fit(
        slot, length, width,
        current_pose.yaw_rad, target_yaw,
        long_margin, lat_margin)
    rotation_radius = 0.5 * math.hypot(
        length + 2.0 * long_margin,
        width + 2.0 * lat_margin)
    return ParkingManeuver(
        slot_id=slot.slot_id,
        fit=fit,
        staging_before_rotation=Pose2D(
            stage_x, stage_y, current_pose.yaw_rad),
        staging_aligned=Pose2D(stage_x, stage_y, target_yaw),
        target_pose=Pose2D(
            slot.center_x_m, slot.center_y_m, target_yaw),
        yaw_change_rad=yaw_change,
        needs_rotation=abs(yaw_change) > yaw_tolerance,
        rotation_sweep_radius_m=rotation_radius,
        rotation_fits_inside_slot=rotation_fit.fits,
    )
