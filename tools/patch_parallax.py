#!/usr/bin/env python3
"""camera_preview_node.py 에 높이(시차) 보정을 넣는다.

문제
----
바닥 기준 homography 는 **바닥 평면 위의 점**만 정확하다. 차량이나 로봇
위 ArUco 처럼 높이가 있는 점은, 카메라마다 자기 광축 지상점에서 바깥으로
밀려난 자리에 사상된다. 밀리는 양은

    오차 = (물체높이 / 카메라높이) x 광축지상점으로부터의 거리

라서 두 카메라가 서로 다른 방향으로 밀어낸다. BEV 로 합치면 같은 차가
두 곳에 보이고, 색분리 화면에서 청록/빨강으로 갈라진다.

카메라 높이 2.61 m, 모형차 높이 0.53 m 면 h/H = 0.203 이다. 광축에서
1.5 m 떨어진 차량은 약 30 cm 밀린다.

해결
----
``vision_utils.correct_floor_projection`` 이 이미 그 역변환을 한다.
런타임 ``yolo_bev_map_node`` 는 쓰고 있으나 프리뷰는 안 쓰고 있었다.
이 패치는 프리뷰의 단일 관문 ``_pixel_to_world`` 에 높이 인자를 넣고,
검출/마커 호출부에서 각각의 실제 높이를 넘긴다.

판정 로직(슬롯 점유, 중복 제거)은 그대로다. 좌표가 정확해지면 그 위에서
도는 판정도 같이 정확해진다.

새 파라미터
-----------
camera_optics_csv        'cctv0:2.463,1.982,2.610; cctv2:1.831,0.507,2.610'
                         label: 광축지상점 X(m), Y(m), 카메라 높이(m)
vehicle_detection_height_m   YOLO 중심이 놓인 높이(m). 0 이면 보정 안 함
marker_height_m              로봇 위 ArUco 높이(m). 0 이면 보정 안 함

사용법:
    python3 patch_parallax.py <camera_preview_node.py 경로>
"""

from __future__ import annotations

import io
from pathlib import Path
import sys


# ------------------------------------------------------------ 1. import

IMPORT_ANCHOR = """from cooperative_parking_robot.bev_fusion_core import (
    SlotOccupancyTracker,"""

IMPORT_NEW = """# 바닥 homography 는 바닥 위의 점만 맞는다. 높이가 있는 점을 되돌리는
# 역변환은 런타임 yolo_bev_map 이 쓰는 것과 **같은 함수**를 쓴다.
from cooperative_parking_robot.vision_utils import correct_floor_projection
from cooperative_parking_robot.bev_fusion_core import (
    SlotOccupancyTracker,"""


# ------------------------------------------------------------ 2. 파서

PARSER_ANCHOR = "def parse_robot_markers(text):"

PARSER_NEW = '''def parse_camera_optics(text):
    """``'cctv0:2.463,1.982,2.610; cctv2:...'`` 를 광학 정보 표로 바꾼다.

    값은 순서대로 광축 지상점 X(m), Y(m), 카메라 높이(m) 다. 광축 지상점은
    렌즈에서 추를 내렸을 때 바닥에 닿는 점이며, homography 와 **같은 map
    좌표계**여야 한다.
    """
    optics = {}
    for chunk in str(text or '').replace('\\n', ';').split(';'):
        chunk = chunk.strip()
        if not chunk:
            continue
        if ':' not in chunk:
            raise ValueError(
                f"camera_optics_csv 항목은 'label:x,y,height' 형식이어야 "
                f"합니다: {chunk!r}")
        label, values = chunk.split(':', 1)
        label = label.strip()
        if not label:
            raise ValueError(f'camera_optics_csv 라벨이 비어 있습니다: {chunk!r}')
        parts = [p.strip() for p in values.split(',') if p.strip()]
        if len(parts) != 3:
            raise ValueError(
                f'{label}: x,y,height 세 값이 필요합니다 (받은 값 {len(parts)}개)')
        try:
            x, y, height = (float(p) for p in parts)
        except ValueError as exc:
            raise ValueError(f'{label}: 숫자로 읽을 수 없습니다: {values!r}') from exc
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(height)):
            raise ValueError(f'{label}: NaN/Inf 는 쓸 수 없습니다')
        if height <= 0.0:
            raise ValueError(f'{label}: 카메라 높이는 0보다 커야 합니다')
        optics[label] = (x, y, height)
    return optics


def parse_robot_markers(text):'''


# ------------------------------------------------------------ 3. 선언

DECL_ANCHOR = "        self.declare_parameter('draw_slots_on_camera', True)"

DECL_NEW = """        self.declare_parameter('draw_slots_on_camera', True)
        # --- 높이(시차) 보정 ---
        # 바닥 homography 는 바닥 점만 맞다. 높이가 있는 점은 카메라마다
        # 자기 광축 지상점 바깥으로 밀린다. 두 카메라가 서로 다른 방향으로
        # 밀어내므로 BEV 에서 같은 차가 두 곳에 보인다.
        #   오차 = (물체높이 / 카메라높이) x 광축지상점에서의 거리
        # 값을 안 주면(높이 0) 보정하지 않으므로 기존 동작 그대로다.
        self.declare_parameter('camera_optics_csv', '')
        self.declare_parameter('vehicle_detection_height_m', 0.0)
        self.declare_parameter('marker_height_m', 0.0)"""


# ------------------------------------------------------------ 4. 읽기

READ_ANCHOR = """        self.robot_marker_ids = parse_robot_markers(
            self.get_parameter('robot_marker_ids_csv').value)"""

READ_NEW = """        self.camera_optics = parse_camera_optics(
            self.get_parameter('camera_optics_csv').value)
        self.vehicle_detection_height = float(
            self.get_parameter('vehicle_detection_height_m').value)
        self.marker_height = float(self.get_parameter('marker_height_m').value)
        for name, value in (('vehicle_detection_height_m',
                             self.vehicle_detection_height),
                            ('marker_height_m', self.marker_height)):
            if value < 0.0:
                raise ValueError(f'{name} 는 0 이상이어야 합니다')
        # 카메라보다 높은 물체는 이 모형으로 되돌릴 수 없다. 조용히 이상한
        # 좌표를 내놓느니 기동에서 막는다.
        for label, (_x, _y, height) in self.camera_optics.items():
            worst = max(self.vehicle_detection_height, self.marker_height)
            if worst >= height:
                raise ValueError(
                    f'{label}: 카메라 높이 {height:.3f} m 가 물체 높이 '
                    f'{worst:.3f} m 이하입니다')
        self.robot_marker_ids = parse_robot_markers(
            self.get_parameter('robot_marker_ids_csv').value)"""


# ------------------------------------------------------------ 5. 변환

W2W_ANCHOR = '''    def _pixel_to_world(self, label, px, py):
        """영상 픽셀을 map 좌표(m)로. H가 없으면 None."""
        matrix = self.pixel_to_world_H.get(label)
        if matrix is None:
            return None
        vector = matrix @ np.array([float(px), float(py), 1.0])
        if abs(float(vector[2])) < 1e-12:
            return None
        x = float(vector[0] / vector[2])
        y = float(vector[1] / vector[2])
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        return [round(x, 3), round(y, 3)]'''

W2W_NEW = '''    def _pixel_to_world(self, label, px, py, height=0.0):
        """영상 픽셀을 map 좌표(m)로. H가 없으면 None.

        ``height`` 가 0 보다 크고 그 카메라의 광학 정보가 있으면, 바닥
        homography 가 낸 점을 실제 물체 높이의 평면으로 되돌린다. 광학
        정보가 없으면 보정 없이 그대로 둔다 — 틀린 값으로 보정하는 것보다
        보정을 안 하는 편이 낫고, 화면에도 그 사실이 표시된다.
        """
        matrix = self.pixel_to_world_H.get(label)
        if matrix is None:
            return None
        vector = matrix @ np.array([float(px), float(py), 1.0])
        if abs(float(vector[2])) < 1e-12:
            return None
        x = float(vector[0] / vector[2])
        y = float(vector[1] / vector[2])
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        optics = self.camera_optics.get(label)
        if optics is not None and float(height) > 0.0:
            ground_x, ground_y, cam_height = optics
            try:
                x, y = correct_floor_projection(
                    x, y, ground_x, ground_y, cam_height, float(height))
            except ValueError:
                # 기동에서 이미 막았지만, 파라미터가 런타임에 바뀌어도
                # 화면이 죽지는 않게 한다.
                pass
        return [round(x, 3), round(y, 3)]

    def parallax_shift_m(self, label, height):
        """이 카메라·높이에서 화면 가장자리가 얼마나 밀리는지(m). 화면 표시용."""
        optics = self.camera_optics.get(label)
        if optics is None or float(height) <= 0.0:
            return None
        _gx, _gy, cam_height = optics
        coverage = self.camera_coverage.get(label)
        if not coverage:
            return None
        ground = (optics[0], optics[1])
        far = max(math.dist(ground, point) for point in coverage)
        return round(far * float(height) / float(cam_height), 3)'''


# ------------------------------------------------------------ 6. 호출부

MARKER_ANCHOR = """            metrics['world'] = self._pixel_to_world(
                state['label'], metrics['center'][0], metrics['center'][1])"""

MARKER_NEW = """            # ArUco 는 로봇 상판 위에 있다. 바닥 평면이 아니다.
            metrics['world'] = self._pixel_to_world(
                state['label'], metrics['center'][0], metrics['center'][1],
                self.marker_height)"""

DET_ANCHOR = """                world_center = self._pixel_to_world(state['label'], *center)"""

DET_NEW = """                world_center = self._pixel_to_world(
                    state['label'], *center, self.vehicle_detection_height)"""

POLY_ANCHOR = """                    corners = [self._pixel_to_world(state['label'], x, y)
                               for x, y in geometry['corners']]"""

POLY_NEW = """                    corners = [
                        self._pixel_to_world(
                            state['label'], x, y,
                            self.vehicle_detection_height)
                        for x, y in geometry['corners']]"""

LEN_ANCHOR = """        start = self._pixel_to_world(label, segment[0][0], segment[0][1])
        end = self._pixel_to_world(label, segment[1][0], segment[1][1])"""

LEN_NEW = """        start = self._pixel_to_world(label, segment[0][0], segment[0][1],
                                     self.vehicle_detection_height)
        end = self._pixel_to_world(label, segment[1][0], segment[1][1],
                                   self.vehicle_detection_height)"""


# ------------------------------------------------------------ 7. API

API_ANCHOR = """                'guidance': self._guidance(time.monotonic()),"""

API_NEW = """                'parallax': {
                    'configured': bool(self.camera_optics),
                    'vehicle_height_m': self.vehicle_detection_height,
                    'marker_height_m': self.marker_height,
                    'cameras': {
                        label: {'ground': [optics[0], optics[1]],
                                'height_m': optics[2],
                                'vehicle_edge_shift_m': self.parallax_shift_m(
                                    label, self.vehicle_detection_height),
                                'marker_edge_shift_m': self.parallax_shift_m(
                                    label, self.marker_height)}
                        for label, optics in self.camera_optics.items()},
                    # 같은 마커를 두 카메라가 볼 때 두 추정 사이의 거리.
                    # 높이 값이 맞을수록 0 에 가까워지므로 조정 기준이 된다.
                    'marker_disagreement_m': self._marker_disagreement(),
                },
                'guidance': self._guidance(time.monotonic()),"""

DISAGREE_ANCHOR = """    def _guidance(self, now):"""

DISAGREE_NEW = '''    def _marker_disagreement(self):
        """같은 ArUco 를 두 카메라가 본 위치 차이(m). 높이 조정의 기준.

        두 카메라의 밀림 방향이 다르므로, 높이 값이 실제와 맞을수록 이
        값이 줄어든다. 화면에서 이 숫자를 보며 높이를 맞추면 된다.
        """
        now = time.monotonic()
        seen = {}
        with self._lock:
            for state in self.cameras:
                if now - state['marker_wall'] > self.stale_after:
                    continue
                for marker in (state['markers'] or []):
                    world = marker.get('world')
                    if world is None:
                        continue
                    seen.setdefault(marker['id'], []).append(tuple(world))
        gaps = {}
        for marker_id, points in seen.items():
            if len(points) < 2:
                continue
            gaps[str(marker_id)] = round(
                max(math.dist(a, b)
                    for i, a in enumerate(points)
                    for b in points[i + 1:]), 3)
        return gaps or None

    def _guidance(self, now):'''


EDITS = [
    (IMPORT_ANCHOR, IMPORT_NEW),
    (PARSER_ANCHOR, PARSER_NEW),
    (DECL_ANCHOR, DECL_NEW),
    (READ_ANCHOR, READ_NEW),
    (W2W_ANCHOR, W2W_NEW),
    (MARKER_ANCHOR, MARKER_NEW),
    (DET_ANCHOR, DET_NEW),
    (POLY_ANCHOR, POLY_NEW),
    (LEN_ANCHOR, LEN_NEW),
    (API_ANCHOR, API_NEW),
    (DISAGREE_ANCHOR, DISAGREE_NEW),
]


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    path = Path(sys.argv[1]).expanduser().resolve()
    if not path.is_file():
        raise SystemExit(f"file not found: {path}")
    text = io.open(path, encoding="utf-8").read()
    if "parse_camera_optics" in text:
        print("skip (already applied)")
        return 0
    for old, new in EDITS:
        count = text.count(old)
        if count != 1:
            raise SystemExit(
                f"anchor found {count} times, expected 1:\n{old[:100]!r}")
        text = text.replace(old, new, 1)
    io.open(path, "w", encoding="utf-8").write(text)
    print(f"patched: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
