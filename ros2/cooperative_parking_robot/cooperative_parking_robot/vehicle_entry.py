"""Vehicle-frame geometry for safe underbody entry and exit.

The target vehicle pose defines a 2-D frame:

* ``s``: vehicle longitudinal axis (+X / front)
* ``d``: vehicle lateral axis (+Y / left)

Keeping this math ROS-independent makes the safety-critical route and frame
transformations directly testable.
"""

from __future__ import annotations

import heapq
import math


ROLES = ("front", "rear")

# Measured setup: 0.785 m axle-to-axle wheelbase and 0.565 m robot length
# leave a 0.220 m longitudinal body-to-body gap at axle alignment.
DEFAULT_WHEELBASE_M = 0.785
ROBOT_LENGTH_M = 0.565
ROBOT_WIDTH_M = 0.420
MIN_INTER_ROBOT_GAP_M = 0.22


def initial_approach_phase(role, simultaneous_entry=False):
    """Choose whether Rear queues or both robots stage concurrently."""
    if role not in ROLES:
        raise ValueError("role must be 'front' or 'rear'")
    if bool(simultaneous_entry) or role == "front":
        return "WAIT_TARGET"
    return "WAIT_FRONT_STAGED"


def initial_align_phase(role, simultaneous_entry=False):
    """Select the peer gate used before the underbody axle scan."""
    if role not in ROLES:
        raise ValueError("role must be 'front' or 'rear'")
    if bool(simultaneous_entry):
        return "WAIT_PEER_STAGED"
    return "WAIT_REAR_OBSERVATION" if role == "front" else "WAIT_FRONT_ALIGNED"


def angle_norm(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def role_sign(role):
    if role not in ROLES:
        raise ValueError("role must be 'front' or 'rear'")
    return 1.0 if role == "front" else -1.0


def exit_longitudinal_translation(
        role, exit_distance, wheelbase,
        same_direction=False, same_direction_sign=1):
    """Return the signed distance used to clear the released vehicle.

    Split exit moves each axle robot toward its nearest end. Same-direction
    exit keeps Front leading and translates both robots by one wheelbase plus
    the normal clearance distance, so the trailing Rear also clears the
    vehicle front while the center separation stays unchanged.
    """
    sign = role_sign(role)
    exit_distance = float(exit_distance)
    wheelbase = float(wheelbase)
    same_direction_sign = int(same_direction_sign)
    if exit_distance <= 0.0 or wheelbase <= 0.0:
        raise ValueError("exit_distance and wheelbase must be positive")
    if same_direction_sign not in (-1, 1):
        raise ValueError("same_direction_sign must be -1 or 1")
    if bool(same_direction):
        return same_direction_sign * (wheelbase + exit_distance)
    return sign * exit_distance


def world_to_vehicle(x, y, vehicle_x, vehicle_y, vehicle_yaw):
    """Project a world point into vehicle longitudinal/lateral coordinates."""
    dx = float(x) - float(vehicle_x)
    dy = float(y) - float(vehicle_y)
    c = math.cos(vehicle_yaw)
    s = math.sin(vehicle_yaw)
    return c * dx + s * dy, -s * dx + c * dy


def vehicle_to_world(longitudinal, lateral, vehicle_x, vehicle_y,
                     vehicle_yaw):
    """Transform vehicle longitudinal/lateral coordinates into the world."""
    c = math.cos(vehicle_yaw)
    s = math.sin(vehicle_yaw)
    return (
        float(vehicle_x) + c * longitudinal - s * lateral,
        float(vehicle_y) + s * longitudinal + c * lateral,
    )


def approach_motion_metrics(start, current, goal):
    """Measure initial travel against the intended map-frame approach.

    Returns ``(travelled, progress, cross_track, alignment_cosine)``.  The
    controller uses this after a few centimetres of CCTV/odom motion so a
    wrong marker-to-base yaw cannot turn a valid map goal into a sideways or
    opposite body command.
    """
    sx, sy = (float(start[0]), float(start[1]))
    cx, cy = (float(current[0]), float(current[1]))
    gx, gy = (float(goal[0]), float(goal[1]))
    values = (sx, sy, cx, cy, gx, gy)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("approach points must be finite")
    goal_dx, goal_dy = gx - sx, gy - sy
    goal_distance = math.hypot(goal_dx, goal_dy)
    if goal_distance <= 1e-9:
        raise ValueError("approach start and goal must differ")
    ux, uy = goal_dx / goal_distance, goal_dy / goal_distance
    move_dx, move_dy = cx - sx, cy - sy
    travelled = math.hypot(move_dx, move_dy)
    progress = move_dx * ux + move_dy * uy
    cross_track = abs(move_dx * uy - move_dy * ux)
    alignment = 1.0 if travelled <= 1e-9 else progress / travelled
    return travelled, progress, cross_track, alignment


def plan_peer_safe_approach(
        start, goal, peer,
        robot_length=ROBOT_LENGTH_M,
        robot_width=ROBOT_WIDTH_M,
        clearance=0.06):
    """Plan an approach that does not cut diagonally through the peer robot.

    Points are in the vehicle ``(s, d)`` frame.  Because both robots hold the
    vehicle yaw during staging, the pair collision envelope is the sum of the
    two half-extents: one full robot length/width plus ``clearance``.  If the
    direct segment enters that envelope, the moving robot first translates on
    the vehicle axis while keeping its lateral lane, then turns toward its
    staging point.  This is the Front departure used while Rear waits.
    """
    start = (float(start[0]), float(start[1]))
    goal = (float(goal[0]), float(goal[1]))
    peer = (float(peer[0]), float(peer[1]))
    robot_length = float(robot_length)
    robot_width = float(robot_width)
    clearance = float(clearance)
    values = (*start, *goal, *peer, robot_length, robot_width, clearance)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("peer approach geometry must be finite")
    if robot_length <= 0.0 or robot_width <= 0.0 or clearance <= 0.0:
        raise ValueError("robot dimensions and peer clearance must be positive")

    relative_start = (start[0] - peer[0], start[1] - peer[1])
    relative_goal = (goal[0] - peer[0], goal[1] - peer[1])
    protected_s = robot_length + clearance
    protected_d = robot_width + clearance
    if not segment_intersects_open_rect(
            relative_start, relative_goal, protected_s, protected_d):
        return [goal]

    # A robot that already overlaps the peer's physical footprint cannot be
    # made safe by planning; do not issue an escape command on an assumption.
    if _point_inside_open_rect(relative_start, robot_length, robot_width):
        raise ValueError("approach starts inside peer robot footprint")
    direction = 1.0 if goal[0] >= start[0] else -1.0
    departure = (
        peer[0] + direction * protected_s,
        start[1],
    )
    # The first leg is allowed inside the *margin* envelope when the robots
    # begin side-by-side, but it must never enter the physical footprint.
    if segment_intersects_open_rect(
            relative_start,
            (departure[0] - peer[0], departure[1] - peer[1]),
            robot_length, robot_width):
        raise ValueError("no collision-free axial departure from peer")
    if segment_intersects_open_rect(
            (departure[0] - peer[0], departure[1] - peer[1]),
            relative_goal, protected_s, protected_d):
        raise ValueError("peer-safe departure does not clear approach route")
    return [departure, goal]


def axle_longitudinal(role, wheelbase):
    return role_sign(role) * float(wheelbase) / 2.0


def standoff_longitudinal(role, entry_standoff):
    """Return the common rear-side staging coordinate.

    Both robots enter from the vehicle rear (-s). Front moves first and
    crosses the rear axle before stopping at the front axle; Rear moves only
    after Front has announced alignment.
    """
    if role not in ROLES:
        raise ValueError("role must be 'front' or 'rear'")
    return -abs(float(entry_standoff))


def approach_longitudinal(role, entry_standoff, wheelbase):
    """Front staging point or Rear's outside ID0 observation queue.

    Front stops at the rear-side standoff. Rear remains one wheelbase farther
    back, so its front camera faces ID0 with the nominal aligned centre
    separation while neither robot has entered the vehicle yet.
    """
    base = standoff_longitudinal(role, entry_standoff)
    if role == "front":
        return base
    return base - abs(float(wheelbase))


def scan_direction(role):
    """Both robots scan from the vehicle rear toward the front (+s)."""
    if role not in ROLES:
        raise ValueError("role must be 'front' or 'rear'")
    return 1.0


def target_axle_index(role):
    """Wheel-pair occurrence that belongs to each same-side entrant."""
    if role not in ROLES:
        raise ValueError("role must be 'front' or 'rear'")
    return 2 if role == "front" else 1


def inter_robot_gap(wheelbase, robot_length=ROBOT_LENGTH_M):
    """Longitudinal body-to-body gap when both robot centers sit on axles."""
    return float(wheelbase) - float(robot_length)


def validate_wheelbase_clearance(
        wheelbase,
        robot_length=ROBOT_LENGTH_M,
        minimum_gap=MIN_INTER_ROBOT_GAP_M):
    wheelbase = float(wheelbase)
    robot_length = float(robot_length)
    minimum_gap = float(minimum_gap)
    if robot_length <= 0.0 or minimum_gap < 0.0:
        raise ValueError("robot_length must be positive and gap non-negative")
    gap = inter_robot_gap(wheelbase, robot_length)
    if gap < minimum_gap:
        raise ValueError(
            f"wheelbase {wheelbase:.3f}m leaves only {gap:.3f}m between "
            f"{robot_length:.3f}m robots; need at least {minimum_gap:.3f}m")
    return gap


def marker_loss_speed_scale(loss_age_s, slowdown_s, stop_s):
    """Encoder-only fallback scale after all usable relative vision is lost."""
    loss_age_s = float(loss_age_s)
    slowdown_s = float(slowdown_s)
    stop_s = float(stop_s)
    if not 0.0 < slowdown_s < stop_s:
        raise ValueError("need 0 < slowdown_s < stop_s")
    if loss_age_s <= slowdown_s:
        return 1.0
    if loss_age_s < stop_s:
        return 0.35
    return 0.0


def rear_scan_speed_from_relative(
        center_distance,
        wheelbase,
        nominal_speed,
        minimum_speed,
        slowdown_window,
        scan_overshoot):
    """Slow Rear near its expected axle without stealing ultrasonic authority.

    The ultrasonic detector needs to pass the wheel trailing edge before it
    can calculate the center, so ArUco never commands a final stop at exactly
    ``wheelbase``. It only slows the coarse scan and enforces a hard lower
    guard at ``wheelbase - scan_overshoot``.
    """
    center_distance = float(center_distance)
    wheelbase = float(wheelbase)
    nominal_speed = float(nominal_speed)
    minimum_speed = float(minimum_speed)
    slowdown_window = float(slowdown_window)
    scan_overshoot = float(scan_overshoot)
    if not 0.0 < minimum_speed <= nominal_speed:
        raise ValueError("need 0 < minimum_speed <= nominal_speed")
    if slowdown_window <= 0.0 or scan_overshoot <= 0.0:
        raise ValueError("slowdown_window/scan_overshoot must be positive")
    remaining = center_distance - wheelbase
    if remaining < -scan_overshoot:
        return None
    if remaining >= slowdown_window:
        return nominal_speed
    progress = max(0.0, (remaining + scan_overshoot) /
                   (slowdown_window + scan_overshoot))
    return minimum_speed + (nominal_speed - minimum_speed) * progress


def relative_alignment_is_consistent(
        center_distance,
        relative_yaw,
        wheelbase,
        distance_tolerance,
        yaw_tolerance,
        *,
        relative_lateral=0.0,
        lateral_tolerance=math.inf):
    return (
        abs(float(center_distance) - float(wheelbase)) <=
        float(distance_tolerance) and
        abs(float(relative_lateral)) <= float(lateral_tolerance) and
        abs(angle_norm(float(relative_yaw))) <= float(yaw_tolerance)
    )


def projected_robot_x_offset(offset_m, robot_yaw, vehicle_yaw):
    """Project a robot +X mounting offset onto the vehicle longitudinal axis."""
    return float(offset_m) * math.cos(
        angle_norm(float(robot_yaw) - float(vehicle_yaw)))


def _point_inside_open_rect(point, half_s, half_d):
    return abs(point[0]) < half_s and abs(point[1]) < half_d


def segment_intersects_open_rect(a, b, half_s, half_d):
    """Return True when a segment enters the open protected rectangle.

    Touching the boundary is allowed because route nodes are placed on a
    separately inflated boundary. This uses a slab intersection test and then
    checks that the intersection contains an interior point.
    """
    if _point_inside_open_rect(a, half_s, half_d):
        return True
    if _point_inside_open_rect(b, half_s, half_d):
        return True

    lo = 0.0
    hi = 1.0
    for origin, delta, lower, upper in (
            (a[0], b[0] - a[0], -half_s, half_s),
            (a[1], b[1] - a[1], -half_d, half_d)):
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
        a[0] + (b[0] - a[0]) * mid,
        a[1] + (b[1] - a[1]) * mid,
    )
    return _point_inside_open_rect(probe, half_s, half_d)


def plan_around_vehicle(start, goal, protected_half_length,
                        protected_half_width, corner_margin=0.01):
    """Shortest visibility-graph route that avoids the target envelope.

    ``start`` and ``goal`` are vehicle-frame points. The returned list excludes
    ``start`` and includes ``goal``. It is intended for reaching a side staging
    point; deliberate underbody entry happens only after this route completes.
    """
    half_s = float(protected_half_length)
    half_d = float(protected_half_width)
    margin = float(corner_margin)
    if half_s <= 0.0 or half_d <= 0.0 or margin <= 0.0:
        raise ValueError("protected extents and corner_margin must be positive")
    if _point_inside_open_rect(start, half_s, half_d):
        raise ValueError("start lies inside protected vehicle envelope")
    if _point_inside_open_rect(goal, half_s, half_d):
        raise ValueError("goal lies inside protected vehicle envelope")

    corners = [
        (-half_s - margin, -half_d - margin),
        (-half_s - margin, half_d + margin),
        (half_s + margin, -half_d - margin),
        (half_s + margin, half_d + margin),
    ]
    nodes = [tuple(start), tuple(goal)] + corners
    graph = {index: [] for index in range(len(nodes))}
    for i, a in enumerate(nodes):
        for j in range(i + 1, len(nodes)):
            b = nodes[j]
            if segment_intersects_open_rect(a, b, half_s, half_d):
                continue
            cost = math.hypot(b[0] - a[0], b[1] - a[1])
            graph[i].append((cost, j))
            graph[j].append((cost, i))

    queue = [(0.0, 0)]
    distance = {0: 0.0}
    previous = {}
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
        raise ValueError("no collision-free route around vehicle envelope")
    indices = [1]
    while indices[-1] != 0:
        indices.append(previous[indices[-1]])
    indices.reverse()
    return [nodes[index] for index in indices[1:]]


def yaw_is_aligned(robot_yaw, vehicle_yaw, tolerance):
    return abs(angle_norm(robot_yaw - vehicle_yaw)) <= float(tolerance)
