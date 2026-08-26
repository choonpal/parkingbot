#!/usr/bin/env python3
"""
==================================================
astar_planner.py
==================================================
OccupancyGrid 기반 경량 A* 경로계획.

천장 카메라가 전역 위치를 직접 제공하므로
자기위치추정/로컬플래너 없이 경량 A*로 충분.

입력: OccupancyGrid (2D 점유 격자)
출력: waypoint 목록 (실좌표 m)
"""

import heapq
import math


class AStarPlanner:
    def __init__(self, resolution=0.05, robot_radius_cells=None,
                 footprint_half_length_m=None, footprint_half_width_m=None,
                 unknown_is_occupied=True, origin_x_m=0.0,
                 origin_y_m=0.0):
        """Create a fixed-yaw rectangular-footprint A* planner.

        ``footprint_half_length_m`` is the front/rear (+x) half extent and
        ``footprint_half_width_m`` is the left/right (+y) half extent of the
        complete Front+vehicle+Rear assembly.  ``robot_radius_cells`` remains
        only as a backwards-compatible way to request a square footprint.
        """
        self.resolution = self._positive("resolution", resolution)
        self.origin_x_m = self._finite("origin_x_m", origin_x_m)
        self.origin_y_m = self._finite("origin_y_m", origin_y_m)
        self.unknown_is_occupied = bool(unknown_is_occupied)

        if footprint_half_length_m is None and footprint_half_width_m is None:
            radius_cells = 3 if robot_radius_cells is None else int(
                robot_radius_cells)
            if radius_cells < 0:
                raise ValueError('robot_radius_cells must be non-negative')
            half_extent = radius_cells * self.resolution
            footprint_half_length_m = half_extent
            footprint_half_width_m = half_extent
        elif (footprint_half_length_m is None or
              footprint_half_width_m is None):
            raise ValueError('both rectangular footprint half extents required')

        self.set_footprint(
            footprint_half_length_m, footprint_half_width_m)

    @staticmethod
    def _positive(name, value):
        value = float(value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    @staticmethod
    def _nonnegative(name, value):
        value = float(value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f'{name} must be finite and non-negative')
        return value

    @staticmethod
    def _finite(name, value):
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f'{name} must be finite')
        return value

    def set_map_geometry(self, resolution, origin_x_m=0.0, origin_y_m=0.0):
        """Synchronize world-to-grid geometry from an OccupancyGrid."""
        self.resolution = self._positive("resolution", resolution)
        self.origin_x_m = self._finite("origin_x_m", origin_x_m)
        self.origin_y_m = self._finite("origin_y_m", origin_y_m)

    def set_footprint(self, half_length_m, half_width_m):
        """Update the mission footprint without rebuilding the planner."""
        self.footprint_half_length_m = self._nonnegative(
            'footprint_half_length_m', half_length_m)
        self.footprint_half_width_m = self._nonnegative(
            'footprint_half_width_m', half_width_m)

    def footprint_half_extent_cells(self):
        return (
            int(math.ceil(
                self.footprint_half_length_m / self.resolution)),
            int(math.ceil(
                self.footprint_half_width_m / self.resolution)),
        )

    def plan(self, grid, width, height, start_m, goal_m):
        """
        A* 경로계획

        Args:
            grid: 1D int8 배열 (0=빈공간, 100=장애물, -1=미확인)
            width, height: 격자 크기
            start_m, goal_m: (x, y) 실좌표 (m)

        Returns:
            waypoints: [(x_m, y_m), ...] 또는 None
        """
        if width <= 0 or height <= 0 or len(grid) != width * height:
            return None

        # 실좌표 → 격자
        start = self._to_cell(start_m)
        goal = self._to_cell(goal_m)

        if not self._valid(start, width, height) or \
           not self._valid(goal, width, height):
            return None

        # 장애물 팽창된 맵 생성
        inflated = self._inflate(grid, width, height)
        if not self._free(inflated, width, start) or not self._free(
                inflated, width, goal):
            return None

        # A* 탐색
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}

        # 8방향 이동
        neighbors = [(-1,0),(1,0),(0,-1),(0,1),
                     (-1,-1),(-1,1),(1,-1),(1,1)]

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self._reconstruct(came_from, current)

            for dx, dy in neighbors:
                nx, ny = current[0]+dx, current[1]+dy
                neighbor = (nx, ny)

                if not self._valid(neighbor, width, height):
                    continue
                # 장애물 체크
                idx = ny * width + nx
                if inflated[idx] >= 50:  # 장애물 or 팽창영역
                    continue

                # 대각선으로 맞닿은 두 장애물의 꼭짓점 사이를 점처럼
                # 통과하는 corner cutting을 막는다. 대각선 이동은 양옆의
                # 직교 셀이 모두 자유공간일 때만 허용한다.
                if dx != 0 and dy != 0:
                    side_x = (current[0] + dx, current[1])
                    side_y = (current[0], current[1] + dy)
                    if (not self._free(inflated, width, side_x) or
                            not self._free(inflated, width, side_y)):
                        continue

                # 이동 비용 (대각선은 √2)
                move_cost = math.hypot(dx, dy)
                tentative = g_score[current] + move_cost

                if neighbor not in g_score or tentative < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative
                    f = tentative + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))

        return None  # 경로 없음

    def _to_cell(self, pos_m):
        return (
            math.floor((pos_m[0] - self.origin_x_m) / self.resolution),
            math.floor((pos_m[1] - self.origin_y_m) / self.resolution),
        )

    def _to_world(self, cell):
        return (
            self.origin_x_m + cell[0] * self.resolution + self.resolution / 2,
            self.origin_y_m + cell[1] * self.resolution + self.resolution / 2,
        )

    def _valid(self, cell, width, height):
        return 0 <= cell[0] < width and 0 <= cell[1] < height

    def _heuristic(self, a, b):
        return math.hypot(a[0]-b[0], a[1]-b[1])

    @staticmethod
    def _free(grid, width, cell):
        return grid[cell[1] * width + cell[0]] < 50

    def _inflate(self, grid, width, height):
        """Inflate obstacles by the fixed-yaw rectangular footprint."""
        inflated = list(grid)
        obstacles = []
        for i, value in enumerate(grid):
            blocked = value >= 50 or (
                self.unknown_is_occupied and value < 0)
            if blocked:
                inflated[i] = 100
                obstacles.append((i % width, i // width))
            elif value < 0:
                inflated[i] = 0

        half_x, half_y = self.footprint_half_extent_cells()
        for ox, oy in obstacles:
            for dy in range(-half_y, half_y + 1):
                for dx in range(-half_x, half_x + 1):
                    nx, ny = ox + dx, oy + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        inflated[ny * width + nx] = 100

        # Grid 밖에도 벽이 있다고 간주한다. 중심이 이 띠 안으로 들어가면
        # 실제 footprint 일부가 맵 밖으로 나가므로 계획 대상에서 제외한다.
        for y in range(height):
            for x in range(width):
                if (x < half_x or x >= width - half_x or
                        y < half_y or y >= height - half_y):
                    inflated[y * width + x] = 100
        return inflated

    def _reconstruct(self, came_from, current):
        """경로 역추적 + 실좌표 변환 + 단순화"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()

        # 직선 구간 단순화 (waypoint 수 줄이기)
        simplified = self._simplify(path)
        return [self._to_world(c) for c in simplified]

    def _simplify(self, path):
        """같은 방향 연속 점 제거 (꺾이는 지점만 남김)"""
        if len(path) < 3:
            return path
        result = [path[0]]
        for i in range(1, len(path)-1):
            dx1 = path[i][0] - path[i-1][0]
            dy1 = path[i][1] - path[i-1][1]
            dx2 = path[i+1][0] - path[i][0]
            dy2 = path[i+1][1] - path[i][1]
            # 방향이 바뀌면 waypoint로 유지
            if (dx1, dy1) != (dx2, dy2):
                result.append(path[i])
        result.append(path[-1])
        return result
