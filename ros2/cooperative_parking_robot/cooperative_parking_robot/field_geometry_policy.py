#!/usr/bin/env python3
"""Field geometry helpers for the 4.40 m x 3.83 m demo site.

The physical tape rectangles describe the final vehicle body only.  The
Front + vehicle + Rear assembly remains the collision footprint used by A*,
rotation and insertion checks.
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import Iterable, Sequence, Tuple


Point2D = Tuple[float, float]


def _finite(name: str, value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _positive(name: str, value: float) -> float:
    value = _finite(name, value)
    if value <= 0.0:
        raise ValueError(f"{name} must be positive")
    return value


def _nonnegative(name: str, value: float) -> float:
    value = _finite(name, value)
    if value < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return value


@dataclass(frozen=True)
class VehicleSlotFit:
    fits: bool
    reason: str
    required_length_m: float
    required_width_m: float
    length_clearance_m: float
    width_clearance_m: float


@dataclass(frozen=True)
class LoadedOverhangCheck:
    fits: bool
    reason: str
    effective_loaded_length_m: float
    overhang_each_end_m: float
    required_back_clearance_m: float
    available_back_clearance_m: float
    clearance_m: float


@dataclass(frozen=True)
class AxisAlignedRect:
    """Axis-aligned protected rectangle in the vehicle (s,d) frame."""

    center_s_m: float
    center_d_m: float
    half_s_m: float
    half_d_m: float

    def __post_init__(self):
        object.__setattr__(self, "center_s_m",
                           _finite("center_s_m", self.center_s_m))
        object.__setattr__(self, "center_d_m",
                           _finite("center_d_m", self.center_d_m))
        object.__setattr__(self, "half_s_m",
                           _positive("half_s_m", self.half_s_m))
        object.__setattr__(self, "half_d_m",
                           _positive("half_d_m", self.half_d_m))


def check_vehicle_only_slot_fit(
        slot_length_m: float,
        slot_width_m: float,
        vehicle_length_m: float,
        vehicle_width_m: float,
        longitudinal_margin_m: float = 0.05,
        lateral_margin_m: float = 0.05) -> VehicleSlotFit:
    """Check the vehicle body, not the robot pair, against a taped slot."""

    slot_length = _positive("slot_length_m", slot_length_m)
    slot_width = _positive("slot_width_m", slot_width_m)
    vehicle_length = _positive("vehicle_length_m", vehicle_length_m)
    vehicle_width = _positive("vehicle_width_m", vehicle_width_m)
    long_margin = _nonnegative(
        "longitudinal_margin_m", longitudinal_margin_m)
    lat_margin = _nonnegative("lateral_margin_m", lateral_margin_m)

    required_length = vehicle_length + 2.0 * long_margin
    required_width = vehicle_width + 2.0 * lat_margin
    length_clearance = slot_length - required_length
    width_clearance = slot_width - required_width
    length_ok = length_clearance >= -1e-9
    width_ok = width_clearance >= -1e-9

    if length_ok and width_ok:
        reason = "OK"
    elif not length_ok and not width_ok:
        reason = "VEHICLE_SLOT_TOO_SHORT_AND_NARROW"
    elif not length_ok:
        reason = "VEHICLE_SLOT_TOO_SHORT"
    else:
        reason = "VEHICLE_SLOT_TOO_NARROW"

    return VehicleSlotFit(
        fits=length_ok and width_ok,
        reason=reason,
        required_length_m=required_length,
        required_width_m=required_width,
        length_clearance_m=length_clearance,
        width_clearance_m=width_clearance,
    )


def check_loaded_overhang_clearance(
        slot_length_m: float,
        loaded_length_m: float,
        loaded_collision_margin_m: float,
        back_clearance_m: float,
        reserve_m: float = 0.03) -> LoadedOverhangCheck:
    """Check the measured free space behind a vehicle-only slot.

    The final vehicle control point is the slot centre.  The loaded rectangle
    is conservatively symmetric about that point.  The aisle-side overhang is
    handled by the existing OccupancyGrid insertion check; this function checks
    the measured free space at the closed/back end of the slot.
    """

    slot_length = _positive("slot_length_m", slot_length_m)
    loaded_length = _positive("loaded_length_m", loaded_length_m)
    collision_margin = _nonnegative(
        "loaded_collision_margin_m", loaded_collision_margin_m)
    back_clearance = _nonnegative("back_clearance_m", back_clearance_m)
    reserve = _nonnegative("reserve_m", reserve_m)

    effective_loaded_length = loaded_length + 2.0 * collision_margin
    overhang_each_end = max(
        0.0, 0.5 * (effective_loaded_length - slot_length))
    required_back = overhang_each_end + reserve
    clearance = back_clearance - required_back
    fits = clearance >= -1e-9

    return LoadedOverhangCheck(
        fits=fits,
        reason="OK" if fits else "INSUFFICIENT_SLOT_BACK_CLEARANCE",
        effective_loaded_length_m=effective_loaded_length,
        overhang_each_end_m=overhang_each_end,
        required_back_clearance_m=required_back,
        available_back_clearance_m=back_clearance,
        clearance_m=clearance,
    )


def projected_half_extents(
        length_m: float,
        width_m: float,
        relative_yaw_rad: float,
        margin_m: float = 0.0) -> Tuple[float, float]:
    """Conservative AABB half extents in the vehicle frame."""

    length = _positive("length_m", length_m)
    width = _positive("width_m", width_m)
    yaw = _finite("relative_yaw_rad", relative_yaw_rad)
    margin = _nonnegative("margin_m", margin_m)
    c = abs(math.cos(yaw))
    s = abs(math.sin(yaw))
    return (
        0.5 * (length * c + width * s) + margin,
        0.5 * (length * s + width * c) + margin,
    )


def _point_inside_open_rect(point: Point2D, rect: AxisAlignedRect) -> bool:
    return (
        abs(point[0] - rect.center_s_m) < rect.half_s_m and
        abs(point[1] - rect.center_d_m) < rect.half_d_m
    )


def _segment_intersects_open_rect(
        start: Point2D, end: Point2D, rect: AxisAlignedRect) -> bool:
    """Slab test for entry into an open axis-aligned rectangle."""

    local_start = (
        start[0] - rect.center_s_m,
        start[1] - rect.center_d_m,
    )
    local_end = (
        end[0] - rect.center_s_m,
        end[1] - rect.center_d_m,
    )
    if (abs(local_start[0]) < rect.half_s_m and
            abs(local_start[1]) < rect.half_d_m):
        return True
    if (abs(local_end[0]) < rect.half_s_m and
            abs(local_end[1]) < rect.half_d_m):
        return True

    lo, hi = 0.0, 1.0
    for origin, delta, lower, upper in (
            (local_start[0], local_end[0] - local_start[0],
             -rect.half_s_m, rect.half_s_m),
            (local_start[1], local_end[1] - local_start[1],
             -rect.half_d_m, rect.half_d_m)):
        if abs(delta) < 1e-12:
            if origin <= lower or origin >= upper:
                return False
            continue
        t0 = (lower - origin) / delta
        t1 = (upper - origin) / delta
        if t0 > t1:
            t0, t1 = t1, t0
        lo = max(lo, t0)
        hi = min(hi, t1)
        if lo > hi:
            return False

    mid = (lo + hi) / 2.0
    probe = (
        local_start[0] + (local_end[0] - local_start[0]) * mid,
        local_start[1] + (local_end[1] - local_start[1]) * mid,
    )
    return (
        abs(probe[0]) < rect.half_s_m and
        abs(probe[1]) < rect.half_d_m
    )


def route_is_clear(
        start: Sequence[float],
        route: Iterable[Sequence[float]],
        rectangles: Iterable[AxisAlignedRect]) -> bool:
    current = (_finite("start.s", start[0]),
               _finite("start.d", start[1]))
    rects = tuple(rectangles)
    for raw in route:
        target = (_finite("route.s", raw[0]),
                  _finite("route.d", raw[1]))
        if any(_segment_intersects_open_rect(current, target, rect)
               for rect in rects):
            return False
        current = target
    return True


def plan_route_around_rectangles(
        start: Sequence[float],
        goal: Sequence[float],
        rectangles: Iterable[AxisAlignedRect],
        corner_margin_m: float = 0.03) -> list[Point2D]:
    """Shortest visibility-graph route around multiple protected rectangles.

    Returned points exclude ``start`` and include ``goal``.  The rectangles are
    expressed in the common vehicle frame, which is exactly the frame used by
    the underbody entry controller.
    """

    start_pt = (_finite("start.s", start[0]),
                _finite("start.d", start[1]))
    goal_pt = (_finite("goal.s", goal[0]),
               _finite("goal.d", goal[1]))
    margin = _positive("corner_margin_m", corner_margin_m)
    rects = tuple(rectangles)

    if not rects:
        return [goal_pt]
    for rect in rects:
        if _point_inside_open_rect(start_pt, rect):
            raise ValueError("start lies inside a protected rectangle")
        if _point_inside_open_rect(goal_pt, rect):
            raise ValueError("goal lies inside a protected rectangle")

    nodes: list[Point2D] = [start_pt, goal_pt]
    for rect in rects:
        hs = rect.half_s_m + margin
        hd = rect.half_d_m + margin
        nodes.extend([
            (rect.center_s_m - hs, rect.center_d_m - hd),
            (rect.center_s_m - hs, rect.center_d_m + hd),
            (rect.center_s_m + hs, rect.center_d_m - hd),
            (rect.center_s_m + hs, rect.center_d_m + hd),
        ])

    graph = {index: [] for index in range(len(nodes))}
    for i, first in enumerate(nodes):
        for j in range(i + 1, len(nodes)):
            second = nodes[j]
            if any(_segment_intersects_open_rect(first, second, rect)
                   for rect in rects):
                continue
            cost = math.hypot(
                second[0] - first[0], second[1] - first[1])
            graph[i].append((cost, j))
            graph[j].append((cost, i))

    queue = [(0.0, 0)]
    distance = {0: 0.0}
    previous: dict[int, int] = {}
    while queue:
        cost, current = heapq.heappop(queue)
        if cost != distance.get(current):
            continue
        if current == 1:
            break
        for edge_cost, neighbor in graph[current]:
            candidate = cost + edge_cost
            if candidate < distance.get(neighbor, math.inf):
                distance[neighbor] = candidate
                previous[neighbor] = current
                heapq.heappush(queue, (candidate, neighbor))

    if 1 not in distance:
        raise ValueError("no collision-free route around protected rectangles")

    indices = [1]
    while indices[-1] != 0:
        indices.append(previous[indices[-1]])
    indices.reverse()
    route = [nodes[index] for index in indices[1:]]
    if not route_is_clear(start_pt, route, rects):
        raise RuntimeError("visibility graph returned an invalid route")
    return route
