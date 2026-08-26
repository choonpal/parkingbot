#!/usr/bin/env python3
"""
==================================================
pure_pursuit.py
==================================================
홀로노믹(메카넘) waypoint 추종기.

waypoint 경로에서 현재 진행 위치를 투영하고 lookahead 지점을 계산한 뒤,
그 지점으로 향하는 로봇 body-frame 평행이동 속도(vx, vy)를 만든다.

기본값은 ``rotate_to_path=False``다. 메카넘 로봇은 목표 방향을 보기 위해
차체를 돌릴 필요가 없으므로 omega=0을 반환하고, 차량 전체 yaw 유지 제어는
rigid_body_sync_node가 별도로 담당한다. 필요할 때만
``rotate_to_path=True``를 사용한다.
"""

import math


class PurePursuit:
    def __init__(self, lookahead=0.15, max_speed=0.08, max_omega=0.3,
                 goal_tolerance=0.03, rotate_to_path=False):
        """
        Args:
            lookahead: 경로 위 전방 주시 거리 (m)
            max_speed: 최대 평면 선속도 (m/s)
            max_omega: rotate_to_path=True일 때 최대 각속도 (rad/s)
            goal_tolerance: 도착 판정 거리 (m)
            rotate_to_path: 진행 방향을 바라보도록 회전할지 여부
        """
        self.lookahead = float(lookahead)
        self.max_speed = float(max_speed)
        self.max_omega = float(max_omega)
        self.goal_tolerance = float(goal_tolerance)
        self.rotate_to_path = bool(rotate_to_path)
        self.waypoints = []
        # 현재 위치가 투영된 경로 segment의 시작 waypoint index
        self.current_idx = 0

    def set_path(self, waypoints):
        """새 경로 설정."""
        self.waypoints = [(float(x), float(y)) for x, y in waypoints]
        self.current_idx = 0

    def is_finished(self, cx, cy):
        """최종 목표 도착 여부."""
        if not self.waypoints:
            return True
        gx, gy = self.waypoints[-1]
        return math.hypot(gx - cx, gy - cy) < self.goal_tolerance

    def compute(self, cx, cy, ctheta):
        """현재 가상 중심점에서 body-frame (vx, vy, omega)를 계산한다."""
        if not self.waypoints:
            return None
        if self.is_finished(cx, cy):
            return (0.0, 0.0, 0.0)

        target = self._find_lookahead(cx, cy)
        if target is None:
            target = self.waypoints[-1]

        dx = target[0] - cx
        dy = target[1] - cy
        dist = math.hypot(dx, dy)
        if dist < 1e-9:
            return (0.0, 0.0, 0.0)

        # world -> robot body. 목표가 뒤면 local_x<0로 후진하고,
        # 옆이면 local_y로 횡이동한다.
        cos_t = math.cos(ctheta)
        sin_t = math.sin(ctheta)
        local_x = dx * cos_t + dy * sin_t
        local_y = -dx * sin_t + dy * cos_t

        vx = self.max_speed * (local_x / dist)
        vy = self.max_speed * (local_y / dist)

        # 최종점 근처에서만 감속해 코너마다 불필요하게 느려지지 않게 한다.
        gx, gy = self.waypoints[-1]
        goal_dist = math.hypot(gx - cx, gy - cy)
        if goal_dist < 0.08:
            scale = max(0.0, goal_dist / 0.08)
            vx *= scale
            vy *= scale

        omega = 0.0
        if self.rotate_to_path:
            target_heading = math.atan2(dy, dx)
            heading_err = self._normalize(target_heading - ctheta)
            omega = self._clamp(heading_err, self.max_omega)

        return (vx, vy, omega)

    def _find_lookahead(self, cx, cy):
        """현재 위치를 경로 polyline에 투영하고 전방 lookahead 점을 반환한다.

        기존 구현은 이미 지나친 코너가 다시 멀어지면 그 코너를 목표로 잡아
        갑자기 후진할 수 있었다. segment 투영 기반 진행도로 바꿔 경로 index가
        뒤로 돌아가지 않도록 한다.
        """
        count = len(self.waypoints)
        if count == 0:
            return None
        if count == 1:
            return self.waypoints[0]

        start_seg = min(max(self.current_idx, 0), count - 2)
        best = None  # (distance_sq, segment_index, t, proj_x, proj_y)
        for i in range(start_seg, count - 1):
            ax, ay = self.waypoints[i]
            bx, by = self.waypoints[i + 1]
            sx, sy = bx - ax, by - ay
            seg_len_sq = sx * sx + sy * sy
            if seg_len_sq < 1e-12:
                t = 0.0
            else:
                t = ((cx - ax) * sx + (cy - ay) * sy) / seg_len_sq
                t = max(0.0, min(1.0, t))
            px = ax + t * sx
            py = ay + t * sy
            d2 = (cx - px) ** 2 + (cy - py) ** 2
            if best is None or d2 < best[0]:
                best = (d2, i, t, px, py)

        if best is None:
            return self.waypoints[-1]

        _, seg_idx, _, px, py = best
        self.current_idx = max(self.current_idx, seg_idx)

        remaining = max(0.0, self.lookahead)
        bx, by = self.waypoints[seg_idx + 1]
        seg_remaining = math.hypot(bx - px, by - py)
        if remaining <= seg_remaining and seg_remaining > 1e-12:
            ratio = remaining / seg_remaining
            return (px + ratio * (bx - px), py + ratio * (by - py))

        remaining -= seg_remaining
        for i in range(seg_idx + 1, count - 1):
            ax, ay = self.waypoints[i]
            bx, by = self.waypoints[i + 1]
            seg_len = math.hypot(bx - ax, by - ay)
            if remaining <= seg_len and seg_len > 1e-12:
                ratio = remaining / seg_len
                self.current_idx = max(self.current_idx, i)
                return (ax + ratio * (bx - ax), ay + ratio * (by - ay))
            remaining -= seg_len

        self.current_idx = count - 1
        return self.waypoints[-1]

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    @staticmethod
    def _normalize(angle):
        return math.atan2(math.sin(angle), math.cos(angle))
