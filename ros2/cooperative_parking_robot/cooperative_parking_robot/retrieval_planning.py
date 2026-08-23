'''Pure geometry used to connect retrieval to the existing motion stack.'''

from dataclasses import dataclass
import math

from cooperative_parking_robot.parking_geometry import ParkingSlot, Pose2D


@dataclass(frozen=True)
class ExtractionGeometry:
    source_staging: Pose2D
    clear_pose: Pose2D


def _positive(name, value):
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f'{name} must be finite and positive')
    return value


def _nonnegative(name, value):
    value = float(value)
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f'{name} must be finite and non-negative')
    return value


def _finite(name, value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    return value


def make_extraction_geometry(slot, final_pose, loaded_length_m,
                             staging_gap_m, lookahead_m, safety_margin_m):
    if not isinstance(slot, ParkingSlot) or not isinstance(final_pose, Pose2D):
        raise TypeError('slot and final_pose have invalid types')
    length = _positive('loaded_length_m', loaded_length_m)
    gap = _nonnegative('staging_gap_m', staging_gap_m)
    lookahead = _positive('lookahead_m', lookahead_m)
    safety = _nonnegative('safety_margin_m', safety_margin_m)
    ux, uy = slot.entry_unit
    relative_x = final_pose.x_m - slot.center_x_m
    relative_y = final_pose.y_m - slot.center_y_m
    final_axial = relative_x * ux + relative_y * uy
    staging_axial = -slot.length_m / 2.0 - length / 2.0 - gap
    outward_distance = final_axial - staging_axial
    if outward_distance <= 0.0:
        raise ValueError('final vehicle pose is already outside source slot')
    staging = Pose2D(
        final_pose.x_m - ux * outward_distance,
        final_pose.y_m - uy * outward_distance,
        final_pose.yaw_rad)
    clear_distance = lookahead + safety
    clear = Pose2D(
        staging.x_m - ux * clear_distance,
        staging.y_m - uy * clear_distance,
        final_pose.yaw_rad)
    return ExtractionGeometry(staging, clear)


def make_waiting_staging(waiting_pose, loaded_length_m, staging_gap_m):
    if not isinstance(waiting_pose, Pose2D):
        raise TypeError('waiting_pose must be Pose2D')
    distance = (_positive('loaded_length_m', loaded_length_m) +
                _nonnegative('staging_gap_m', staging_gap_m))
    c, s = math.cos(waiting_pose.yaw_rad), math.sin(waiting_pose.yaw_rad)
    return Pose2D(
        waiting_pose.x_m - c * distance,
        waiting_pose.y_m - s * distance,
        waiting_pose.yaw_rad)


def _cell_occupied(grid, width, height, gx, gy, unknown_is_occupied=True):
    if gx < 0 or gy < 0 or gx >= width or gy >= height:
        return True
    value = int(grid[gy * width + gx])
    return value >= 50 or (value < 0 and unknown_is_occupied)


def oriented_footprint_is_free(grid, width, height, resolution,
                               x_m, y_m, yaw_rad, length_m, width_m,
                               margin_m=0.0, unknown_is_occupied=True,
                               origin_x_m=0.0, origin_y_m=0.0):
    resolution = _positive('resolution', resolution)
    origin_x = _finite('origin_x_m', origin_x_m)
    origin_y = _finite('origin_y_m', origin_y_m)
    half_length = _positive('length_m', length_m) / 2.0
    half_width = _positive('width_m', width_m) / 2.0
    margin = _nonnegative('margin_m', margin_m)
    half_length += margin
    half_width += margin
    radius = math.hypot(half_length, half_width)
    min_gx = int(math.floor((x_m - radius - origin_x) / resolution))
    max_gx = int(math.floor((x_m + radius - origin_x) / resolution))
    min_gy = int(math.floor((y_m - radius - origin_y) / resolution))
    max_gy = int(math.floor((y_m + radius - origin_y) / resolution))
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    padding = resolution / math.sqrt(2.0)
    for gy in range(min_gy, max_gy + 1):
        for gx in range(min_gx, max_gx + 1):
            if not _cell_occupied(
                    grid, width, height, gx, gy, unknown_is_occupied):
                continue
            cell_x = origin_x + (gx + 0.5) * resolution
            cell_y = origin_y + (gy + 0.5) * resolution
            dx, dy = cell_x - x_m, cell_y - y_m
            local_x = dx * c + dy * s
            local_y = -dx * s + dy * c
            if (abs(local_x) <= half_length + padding and
                    abs(local_y) <= half_width + padding):
                return False
    return True


def corridor_is_free(grid, width, height, resolution, start, goal, yaw_rad,
                     length_m, width_m, margin_m=0.0,
                     unknown_is_occupied=True, goal_yaw_rad=None,
                     speed_mps=None, yaw_gain=1.5, max_yaw_rate=0.15,
                     origin_x_m=0.0, origin_y_m=0.0):
    dx, dy = float(goal[0]) - float(start[0]), float(goal[1]) - float(start[1])
    distance = math.hypot(dx, dy)
    spatial_count = max(
        1, int(math.ceil(distance / max(resolution * 0.5, 1e-3))))
    if goal_yaw_rad is None:
        duration = 0.0
        count = spatial_count
    else:
        speed = _positive('speed_mps', speed_mps)
        duration = max(
            distance / speed,
            _yaw_settle_duration(
                yaw_rad, goal_yaw_rad, yaw_gain, max_yaw_rate))
        count = max(spatial_count, int(math.ceil(duration / 0.10)))
    for index in range(count + 1):
        ratio = index / count
        if goal_yaw_rad is None:
            x_m = float(start[0]) + dx * ratio
            y_m = float(start[1]) + dy * ratio
            sample_yaw = yaw_rad
        else:
            elapsed = duration * ratio
            x_m, y_m = _route_position((start, goal), elapsed, speed)
            sample_yaw = _yaw_toward(
                yaw_rad, goal_yaw_rad, elapsed,
                yaw_gain, max_yaw_rate)
        if not oriented_footprint_is_free(
                grid, width, height, resolution,
                x_m, y_m, sample_yaw, length_m, width_m,
                margin_m, unknown_is_occupied,
                origin_x_m, origin_y_m):
            return False
    return True


def clear_source_vehicle(grid, width, height, resolution, pose,
                         length_m, width_m, minimum_mask_size_m=0.0,
                         origin_x_m=0.0, origin_y_m=0.0):
    if len(grid) != int(width) * int(height):
        raise ValueError('grid dimensions do not match data')
    if not isinstance(pose, Pose2D):
        raise TypeError('pose must be Pose2D')
    output = list(grid)
    resolution = _positive('resolution', resolution)
    origin_x = _finite('origin_x_m', origin_x_m)
    origin_y = _finite('origin_y_m', origin_y_m)
    minimum_size = _nonnegative(
        'minimum_mask_size_m', minimum_mask_size_m)
    # COCO/dual fallback map은 polygon이 없을 때 car_size 정사각형을 채운다.
    # Registry 실차 폭만 지우면 그 정사각형의 잔여 셀이 extraction을 막으므로
    # 두 표현 중 큰 쪽과 raster cell 반대각까지 source mask로 간주한다.
    cell_padding = resolution / math.sqrt(2.0)
    half_length = max(
        _positive('length_m', length_m), minimum_size) / 2.0 + cell_padding
    half_width = max(
        _positive('width_m', width_m), minimum_size) / 2.0 + cell_padding
    radius = math.hypot(half_length, half_width)
    c, s = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
    min_gx = max(0, int(math.floor(
        (pose.x_m - radius - origin_x) / resolution)))
    max_gx = min(width - 1, int(math.floor(
        (pose.x_m + radius - origin_x) / resolution)))
    min_gy = max(0, int(math.floor(
        (pose.y_m - radius - origin_y) / resolution)))
    max_gy = min(height - 1, int(math.floor(
        (pose.y_m + radius - origin_y) / resolution)))
    for gy in range(min_gy, max_gy + 1):
        for gx in range(min_gx, max_gx + 1):
            dx = origin_x + (gx + 0.5) * resolution - pose.x_m
            dy = origin_y + (gy + 0.5) * resolution - pose.y_m
            local_x = dx * c + dy * s
            local_y = -dx * s + dy * c
            if abs(local_x) <= half_length and abs(local_y) <= half_width:
                output[gy * width + gx] = 0
    return output


def _rectangle_corners(center, yaw, half_length, half_width):
    c, s = math.cos(yaw), math.sin(yaw)
    return tuple((
        center[0] + c * sx - s * sy,
        center[1] + s * sx + c * sy,
    ) for sx, sy in ((-half_length, -half_width),
                     (half_length, -half_width),
                     (half_length, half_width),
                     (-half_length, half_width)))


def _rectangles_overlap(first, second, first_yaw, second_yaw,
                        half_length, half_width):
    first_corners = _rectangle_corners(
        first, first_yaw, half_length, half_width)
    second_corners = _rectangle_corners(
        second, second_yaw, half_length, half_width)
    axes = tuple((math.cos(yaw), math.sin(yaw)) for yaw in (
        first_yaw, first_yaw + math.pi / 2.0,
        second_yaw, second_yaw + math.pi / 2.0))
    for axis_x, axis_y in axes:
        first_projection = [x * axis_x + y * axis_y for x, y in first_corners]
        second_projection = [x * axis_x + y * axis_y for x, y in second_corners]
        if (max(first_projection) < min(second_projection) or
                max(second_projection) < min(first_projection)):
            return False
    return True


def _point_segment_distance(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-18:
        return math.dist(point, start)
    ratio = (
        (point[0] - start[0]) * dx +
        (point[1] - start[1]) * dy) / length_sq
    ratio = max(0.0, min(1.0, ratio))
    projection = (start[0] + ratio * dx, start[1] + ratio * dy)
    return math.dist(point, projection)


def _rectangles_violate_clearance(
        first, second, first_yaw, second_yaw,
        half_length, half_width, minimum_gap):
    if _rectangles_overlap(
            first, second, first_yaw, second_yaw,
            half_length, half_width):
        return True
    if minimum_gap <= 0.0:
        return False
    first_corners = _rectangle_corners(
        first, first_yaw, half_length, half_width)
    second_corners = _rectangle_corners(
        second, second_yaw, half_length, half_width)
    first_edges = tuple(zip(
        first_corners, first_corners[1:] + first_corners[:1]))
    second_edges = tuple(zip(
        second_corners, second_corners[1:] + second_corners[:1]))
    clearance = min(
        [_point_segment_distance(point, *edge)
         for point in first_corners for edge in second_edges] +
        [_point_segment_distance(point, *edge)
         for point in second_corners for edge in first_edges])
    return clearance < minimum_gap - 1e-9


def _route_position(route, elapsed, speed):
    start, goal = route
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    distance = math.hypot(dx, dy)
    if distance <= 1e-12:
        return float(goal[0]), float(goal[1])
    ratio = min(1.0, elapsed * speed / distance)
    return start[0] + ratio * dx, start[1] + ratio * dy


def _angle_norm(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def _yaw_settle_duration(start_yaw, goal_yaw, yaw_gain, max_yaw_rate,
                         tolerance=0.005):
    gain = _positive('yaw_gain', yaw_gain)
    rate = _positive('max_yaw_rate', max_yaw_rate)
    tolerance = _positive('yaw_tolerance', tolerance)
    magnitude = abs(_angle_norm(goal_yaw - start_yaw))
    if magnitude <= tolerance:
        return 0.0
    threshold = rate / gain
    if magnitude > threshold:
        saturated = (magnitude - threshold) / rate
        return saturated + math.log(threshold / tolerance) / gain
    return math.log(magnitude / tolerance) / gain


def _yaw_toward(start_yaw, goal_yaw, elapsed, yaw_gain, max_yaw_rate):
    gain = _positive('yaw_gain', yaw_gain)
    rate = _positive('max_yaw_rate', max_yaw_rate)
    elapsed = _nonnegative('elapsed', elapsed)
    error = _angle_norm(goal_yaw - start_yaw)
    magnitude = abs(error)
    if magnitude <= 1e-12:
        return _angle_norm(goal_yaw)
    sign = 1.0 if error > 0.0 else -1.0
    threshold = rate / gain
    if magnitude > threshold:
        saturated = (magnitude - threshold) / rate
        if elapsed <= saturated:
            remaining = magnitude - rate * elapsed
        else:
            remaining = threshold * math.exp(
                -gain * (elapsed - saturated))
    else:
        remaining = magnitude * math.exp(-gain * elapsed)
    return _angle_norm(goal_yaw - sign * remaining)


def simultaneous_routes_clear(front_route, rear_route, speed_mps,
                              robot_length_m, robot_width_m,
                              minimum_gap_m, front_yaw_rad=0.0,
                              rear_yaw_rad=0.0,
                              front_goal_yaw_rad=None,
                              rear_goal_yaw_rad=None,
                              yaw_gain=1.5, max_yaw_rate=0.15):
    speed = _positive('speed_mps', speed_mps)
    length = _positive('robot_length_m', robot_length_m)
    width = _positive('robot_width_m', robot_width_m)
    gap = _nonnegative('minimum_gap_m', minimum_gap_m)
    front_goal_yaw = (front_yaw_rad if front_goal_yaw_rad is None
                      else front_goal_yaw_rad)
    rear_goal_yaw = (rear_yaw_rad if rear_goal_yaw_rad is None
                     else rear_goal_yaw_rad)
    durations = [
        max(math.dist(*route) / speed,
            _yaw_settle_duration(start_yaw, goal_yaw,
                                 yaw_gain, max_yaw_rate))
        for route, start_yaw, goal_yaw in (
            (front_route, front_yaw_rad, front_goal_yaw),
            (rear_route, rear_yaw_rad, rear_goal_yaw))]
    duration = max(durations)
    spatial_step = max(0.002, min(0.01, max(gap, 0.01) / 3.0))
    count = max(1, int(math.ceil(duration * speed / spatial_step)))
    half_length = length / 2.0
    half_width = width / 2.0
    for index in range(count + 1):
        elapsed = duration * index / count
        front = _route_position(front_route, elapsed, speed)
        rear = _route_position(rear_route, elapsed, speed)
        front_yaw = _yaw_toward(
            front_yaw_rad, front_goal_yaw, elapsed,
            yaw_gain, max_yaw_rate)
        rear_yaw = _yaw_toward(
            rear_yaw_rad, rear_goal_yaw, elapsed,
            yaw_gain, max_yaw_rate)
        if _rectangles_violate_clearance(
                front, rear, front_yaw, rear_yaw,
                half_length, half_width, gap):
            return False
    return True


def sequential_routes_clear(front_route, rear_route, speed_mps,
                            robot_length_m, robot_width_m,
                            minimum_gap_m, front_yaw_rad=0.0,
                            rear_yaw_rad=0.0,
                            front_goal_yaw_rad=None,
                            rear_goal_yaw_rad=None,
                            yaw_gain=1.5, max_yaw_rate=0.15):
    '''Check the existing Front-first entry timing without route replanning.

    Front first translates to staging on its current HOME axis while Rear
    stays at HOME. Front then turns at staging while Rear translates on its
    HOME axis. Finally Rear turns at staging while Front stays aligned. This
    mirrors simultaneous_entry=false and keeps close HOME rotation out of the
    first phase without weakening the footprint-gap check.
    '''
    speed = _positive('speed_mps', speed_mps)
    length = _positive('robot_length_m', robot_length_m)
    width = _positive('robot_width_m', robot_width_m)
    gap = _nonnegative('minimum_gap_m', minimum_gap_m)
    half_length = length / 2.0
    half_width = width / 2.0
    spatial_step = max(0.002, min(0.01, max(gap, 0.01) / 3.0))
    front_goal_yaw = (front_yaw_rad if front_goal_yaw_rad is None
                      else front_goal_yaw_rad)
    rear_goal_yaw = (rear_yaw_rad if rear_goal_yaw_rad is None
                     else rear_goal_yaw_rad)
    # Phase 1: Front translates first, Rear remains at HOME.
    duration = math.dist(*front_route) / speed
    count = max(1, int(math.ceil(duration * speed / spatial_step)))
    for index in range(count + 1):
        elapsed = duration * index / count
        front = _route_position(front_route, elapsed, speed)
        if _rectangles_violate_clearance(
                front, rear_route[0], front_yaw_rad, rear_yaw_rad,
                half_length, half_width, gap):
            return False

    # Phase 2: Front has staging clearance and turns while Rear translates.
    duration = max(
        math.dist(*rear_route) / speed,
        _yaw_settle_duration(
            front_yaw_rad, front_goal_yaw, yaw_gain, max_yaw_rate))
    count = max(1, int(math.ceil(duration * speed / spatial_step)))
    for index in range(count + 1):
        elapsed = duration * index / count
        front_yaw = _yaw_toward(
            front_yaw_rad, front_goal_yaw, elapsed,
            yaw_gain, max_yaw_rate)
        rear = _route_position(rear_route, elapsed, speed)
        if _rectangles_violate_clearance(
                front_route[1], rear, front_yaw, rear_yaw_rad,
                half_length, half_width, gap):
            return False

    # Phase 3: Rear aligns at staging after Front is already aligned.
    duration = _yaw_settle_duration(
        rear_yaw_rad, rear_goal_yaw, yaw_gain, max_yaw_rate)
    count = max(1, int(math.ceil(duration * speed / spatial_step)))
    for index in range(count + 1):
        elapsed = duration * index / count
        rear_yaw = _yaw_toward(
            rear_yaw_rad, rear_goal_yaw, elapsed,
            yaw_gain, max_yaw_rate)
        if _rectangles_violate_clearance(
                front_route[1], rear_route[1],
                front_goal_yaw, rear_yaw,
                half_length, half_width, gap):
            return False
    return True
