#!/usr/bin/env python3
"""RViz 없이 터미널에서 /parking/map을 보는 헤드리스 뷰어.

젯슨을 SSH나 VSCode 원격으로만 쓰면 `rviz2`가
``qt.qpa.xcb: could not connect to display``로 죽는다. X 포워딩을 붙이는
방법도 있지만, 맵이 제대로 나오는지 확인하는 데는 이 정도면 충분하고
훨씬 빠르다.

보여주는 것
-----------
  * /parking/map            OccupancyGrid를 ASCII로 (점유 '#', 빈 곳 '.')
  * /parking/empty_slots    빈자리로 확정된 슬롯 위치에 'O'
  * /cctv/merge_status      카메라 생존/중복제거/슬롯 관측 요약
  * /parking/target_pose    대기영역 타겟 차량 'T'
  * /front,/rear cctv_pose  로봇 절대 pose 'F' / 'R'

맵은 ROS 관례대로 원점(0,0)이 **좌하단**이므로, 화면에서도 아래쪽이 y=0이다.

사용법
------
    ros2 run cooperative_parking_robot show_map_ascii
    # 또는 소스에서 직접
    python3 scripts/show_map_ascii.py --width 100 --rate 2
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

# 점유도 -> 문자. 낮을수록 비어 있음.
_RAMP = [(1, '.'), (25, ':'), (50, '+'), (75, '*'), (101, '#')]


def _cell_char(value: int) -> str:
    if value < 0:
        return '?'          # unknown
    for threshold, symbol in _RAMP:
        if value < threshold:
            return symbol
    return '#'


class AsciiMapViewer(Node):
    def __init__(self, columns: int, rate_hz: float, use_color: bool):
        super().__init__('show_map_ascii')
        self.columns = columns
        self.use_color = use_color
        self.grid = None
        self.empty_slots = []
        self.target = None
        self.robots = {}
        self.status = None
        self.last_map_wall = 0.0

        self.create_subscription(
            OccupancyGrid, '/parking/map', self._map_cb, 10)
        self.create_subscription(
            PoseArray, '/parking/empty_slots', self._empty_cb, 10)
        self.create_subscription(
            PoseStamped, '/parking/target_pose', self._target_cb, 10)
        self.create_subscription(
            String, '/cctv/merge_status', self._status_cb, 10)
        for role in ('front', 'rear'):
            self.create_subscription(
                PoseStamped, f'/{role}/cctv_pose',
                lambda msg, r=role: self._robot_cb(r, msg),
                qos_profile_sensor_data)

        self.create_timer(1.0 / max(0.2, rate_hz), self._render)

    # ---------------- 콜백 ----------------
    def _map_cb(self, msg):
        self.grid = msg
        self.last_map_wall = time.monotonic()

    def _empty_cb(self, msg):
        self.empty_slots = [(p.position.x, p.position.y) for p in msg.poses]

    def _target_cb(self, msg):
        self.target = (msg.pose.position.x, msg.pose.position.y)

    def _robot_cb(self, role, msg):
        self.robots[role] = (msg.pose.position.x, msg.pose.position.y,
                             time.monotonic())

    def _status_cb(self, msg):
        try:
            self.status = json.loads(msg.data)
        except (TypeError, ValueError):
            self.status = None

    # ---------------- 렌더 ----------------
    def _render(self):
        lines = []
        if self.grid is None:
            lines.append('/parking/map 수신 대기 중...')
            lines.append('  merge 노드가 "살아있는 CCTV sensor 노드가 없습니다"를')
            lines.append('  반복하면 카메라/센서 노드부터 확인하세요.')
            self._paint(lines)
            return

        grid = self.grid
        width, height = grid.info.width, grid.info.height
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y

        # 터미널 폭에 맞춰 열을 줄인다. 문자 종횡비(약 2:1)를 반영해
        # 행은 열의 절반 비율로 뽑아야 정사각형처럼 보인다.
        columns = max(20, min(self.columns, width))
        col_step = width / columns
        rows = max(8, int(round(height / col_step / 2.0)))
        row_step = height / rows

        def sample(cx, cy):
            """해당 화면 셀이 덮는 영역에서 가장 큰 점유도를 취한다.
            축소 표시에서 장애물이 사라지지 않게 하려는 것."""
            x0 = int(cx * col_step)
            x1 = max(x0 + 1, int((cx + 1) * col_step))
            y0 = int(cy * row_step)
            y1 = max(y0 + 1, int((cy + 1) * row_step))
            best = -1
            for gy in range(y0, min(y1, height)):
                base = gy * width
                row = grid.data[base + x0:base + min(x1, width)]
                if row:
                    best = max(best, max(row))
            return best

        canvas = [[_cell_char(sample(cx, cy)) for cx in range(columns)]
                  for cy in range(rows)]

        def plot(world_x, world_y, symbol):
            gx = (world_x - origin_x) / resolution
            gy = (world_y - origin_y) / resolution
            cx = int(gx / col_step)
            cy = int(gy / row_step)
            if 0 <= cx < columns and 0 <= cy < rows:
                canvas[cy][cx] = symbol

        for x, y in self.empty_slots:
            plot(x, y, 'O')
        if self.target is not None:
            plot(self.target[0], self.target[1], 'T')
        now = time.monotonic()
        for role, (x, y, stamp) in self.robots.items():
            if now - stamp < 1.0:
                plot(x, y, 'F' if role == 'front' else 'R')

        span_x = width * resolution
        span_y = height * resolution
        age = now - self.last_map_wall
        lines.append(
            f'/parking/map  {width}x{height} cells @ {resolution:.3f} m  '
            f'= {span_x:.2f} x {span_y:.2f} m   (수신 {age:.1f}s 전)')
        lines.append('')
        border = '+' + '-' * columns + '+'
        lines.append(f'  y={span_y:>4.1f}m {border}')
        for cy in range(rows - 1, -1, -1):   # 위쪽이 +y
            lines.append('         |' + ''.join(canvas[cy]) + '|')
        lines.append(f'  y={0.0:>4.1f}m {border}')
        lines.append(f'          x=0.0m{" " * max(0, columns - 14)}x={span_x:.1f}m')
        lines.append('')
        lines.append('  범례: . 빈 곳   # 장애물   O 빈 주차면   '
                     'T 타겟차량   F/R 로봇   ? 미탐색')
        lines.append('')

        if self.status:
            cams = self.status.get('cameras', {})
            parts = []
            for name, state in sorted(cams.items()):
                mark = 'OK ' if state.get('alive') else '죽음'
                parts.append(
                    f"{name}={mark}({state.get('detections', 0)}건, "
                    f"{state.get('age_s', 0):.2f}s)")
            lines.append('  카메라  : ' + '  '.join(parts))
            lines.append(
                f"  병합    : 검출 {self.status.get('merged_detections', 0)}개 | "
                f"중복제거 {self.status.get('duplicates_removed', 0)} | "
                f"2대관측 {self.status.get('multi_camera_detections', 0)}")
            slots = self.status.get('slots', {})
            if slots:
                shown = []
                for slot_id, state in sorted(slots.items()):
                    if not state.get('observed'):
                        shown.append(f'{slot_id}:시야밖')
                    elif state.get('occupied'):
                        shown.append(f'{slot_id}:점유')
                    else:
                        shown.append(f'{slot_id}:빈칸')
                lines.append('  슬롯    : ' + '  '.join(shown))
        else:
            lines.append('  (/cctv/merge_status 없음 — 단일 카메라 구성이면 정상)')

        self._paint(lines)

    def _paint(self, lines):
        sys.stdout.write('\033[H\033[J')  # 커서 홈 + 화면 지우기
        sys.stdout.write('\n'.join(lines) + '\n')
        sys.stdout.flush()


def main(args=None):
    parser = argparse.ArgumentParser(
        description='RViz 없이 터미널에서 /parking/map 보기')
    default_columns = max(40, min(110, shutil.get_terminal_size((100, 30)).columns - 12))
    parser.add_argument('--width', type=int, default=default_columns,
                        help='ASCII 맵 가로 문자 수')
    parser.add_argument('--rate', type=float, default=2.0,
                        help='화면 갱신 Hz')
    parser.add_argument('--no-color', action='store_true')
    known, ros_args = parser.parse_known_args(
        args if args is not None else sys.argv[1:])

    rclpy.init(args=ros_args)
    node = AsciiMapViewer(known.width, known.rate, not known.no_color)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        sys.stdout.write('\n')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
