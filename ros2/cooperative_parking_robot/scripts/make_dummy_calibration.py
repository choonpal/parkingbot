#!/usr/bin/env python3
"""현장 실측 없이 파이프라인만 확인하기 위한 **임시** 캘리브레이션 생성기.

만드는 것
---------
  ~/.ros/adaptive_valet_bot/homography_cam0_rectified.npy   (+ .json)
  ~/.ros/adaptive_valet_bot/homography_cam2_rectified.npy   (+ .json)
  ~/.ros/adaptive_valet_bot/parking_layout.yaml

이 H는 "픽셀을 그냥 비례해서 metre로 늘린" 순수 affine이다. 실제 카메라
렌즈/설치각과 아무 상관이 없으므로 **좌표값 자체는 의미가 없다.** 대신
다음은 전부 진짜로 검증할 수 있다.

  * 두 카메라 sensor 인스턴스가 각각 detection envelope을 내는가
  * cctv_merge_node가 두 envelope을 받아 하나로 합치는가
  * 겹침 영역의 같은 물체를 중복 제거하는가 (duplicates_removed)
  * 카메라별 coverage polygon이 계산되는가
  * 어떤 카메라도 못 보는 슬롯이 빈자리에서 빠지는가 (observed=false)
  * /parking/map OccupancyGrid가 생성되고 fleet_manager가 A*를 도는가

기본 배치 (map 6.0m x 4.0m)
--------------------------
    y=4 ┌───────────────────────────────────┐
        │  P1    P2      P3        P4       │   ← 주차면 4칸
        │ [==]  [==]    [==]      [==]      │
        │                                   │
        │            ┌──────┐               │
        │            │ 대기 │               │
    y=0 └────────────┴──────┴───────────────┘
        x=0        2.6    3.4              x=6
        │◀────── cam0 (0 ~ 3.4) ──────▶│
                      │◀──────── cam2 (2.6 ~ 6.0) ────────▶│
                      │◀ 겹침 ▶│
                        0.8m

  P1(x=0.8), P2(x=1.8) → cam0만 봄
  P3(x=3.0)            → 두 카메라가 다 봄 (겹침 검증용)
  P4(x=4.6)            → cam2만 봄
  대기영역(x 2.6~3.4)  → 두 카메라가 다 봄

사용법
------
    # 패키지 소스 루트에서
    python3 scripts/make_dummy_calibration.py

    # 이번 임시 intrinsic과 같은 기본 해상도는 640x480이다.
    python3 scripts/make_dummy_calibration.py

    # 다른 해상도의 파이프라인 연결만 확인하려면 명시한다.
    python3 scripts/make_dummy_calibration.py --width 1280 --height 720

나중에 실제 캘리브레이션을 하면 bev_layout_calibration.launch.py가 이 파일들을
그대로 덮어쓴다. 따로 지울 필요 없다.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys

# 소스 트리에서 바로 실행할 수 있게 패키지 경로를 잡아준다.
_HERE = Path(__file__).resolve().parent
for candidate in (_HERE.parent, _HERE.parent.parent):
    if (candidate / 'cooperative_parking_robot' / '__init__.py').is_file():
        sys.path.insert(0, str(candidate))
        break

import numpy as np  # noqa: E402

from cooperative_parking_robot.bev_layout_core import (  # noqa: E402
    render_parking_layout_yaml,
    write_text_atomic,
)
from cooperative_parking_robot.parking_geometry import (  # noqa: E402
    slot_from_corners,
)
from cooperative_parking_robot.bev_fusion_core import (  # noqa: E402
    image_corner_coverage,
    point_in_polygon,
    polygon_centroid,
)

DEFAULT_OUTPUT = '~/.ros/adaptive_valet_bot'


def affine_homography(x_min, x_max, y_min, y_max, width_px, height_px):
    """영상 전체를 [x_min,x_max] x [y_min,y_max] 바닥 사각형에 대응시킨다.

    영상 위쪽(row 0)이 map의 +y(먼 쪽)가 되도록 y를 뒤집는다. 천장 카메라를
    수직으로 내려다보게 달았을 때의 상식적인 방향이다.
    """
    if width_px < 2 or height_px < 2:
        raise ValueError('image size must be at least 2x2')
    a = (x_max - x_min) / (width_px - 1.0)
    b = -(y_max - y_min) / (height_px - 1.0)
    return np.array([
        [a, 0.0, x_min],
        [0.0, b, y_max],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)


def save_npy_atomic(path: Path, matrix):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('wb') as stream:
        np.save(stream, matrix, allow_pickle=False)
        stream.flush()
    temporary.replace(path)


def corner_references(homography, width_px, height_px):
    """등록 도구가 남기는 것과 같은 형식의 기준점 목록을 만든다.

    더미이므로 재투영 오차는 0이다. 실측 파일과 구분되도록 metadata에
    ``synthetic: true``를 남긴다.
    """
    corners_px = [
        (0.0, 0.0),
        (width_px - 1.0, 0.0),
        (width_px - 1.0, height_px - 1.0),
        (0.0, height_px - 1.0),
    ]
    references = []
    for px, py in corners_px:
        vector = homography @ np.array([px, py, 1.0])
        references.append({
            'pixel': [px, py],
            'world': [float(vector[0] / vector[2]),
                      float(vector[1] / vector[2])],
        })
    return references


def make_slot(slot_id, center_x, y_near, y_far, half_width):
    """통로(y가 작은 쪽)에서 안쪽(y가 큰 쪽)으로 진입하는 주차면."""
    corners = [
        (center_x - half_width, y_near),
        (center_x + half_width, y_near),
        (center_x + half_width, y_far),
        (center_x - half_width, y_far),
    ]
    aisle = (center_x, y_near - 0.5)
    return slot_from_corners(slot_id, corners, aisle), corners


def main():
    parser = argparse.ArgumentParser(
        description='현장 실측 없이 파이프라인만 확인하기 위한 임시 캘리브레이션 생성')
    parser.add_argument('--output-dir', default=DEFAULT_OUTPUT)
    parser.add_argument('--width', type=int, default=640,
                        help='카메라 영상 가로 픽셀 (실제 스트림과 같아야 함)')
    parser.add_argument('--height', type=int, default=480,
                        help='카메라 영상 세로 픽셀 (실제 스트림과 같아야 함)')
    parser.add_argument('--map-width', type=float, default=6.0)
    parser.add_argument('--map-height', type=float, default=4.0)
    parser.add_argument('--map-resolution', type=float, default=0.05)
    parser.add_argument('--overlap', type=float, default=0.8,
                        help='두 카메라 시야가 겹치는 폭 [m]')
    parser.add_argument('--coverage-margin-px', type=float, default=8.0,
                        help='launch의 coverage_margin_px와 같은 값을 주면 '
                             '실제 coverage를 미리 확인할 수 있다')
    parser.add_argument('--force', action='store_true',
                        help='기존 파일이 있어도 덮어쓴다')
    args = parser.parse_args()

    out_dir = Path(os.path.expanduser(args.output_dir))
    map_w, map_h = float(args.map_width), float(args.map_height)
    overlap = float(args.overlap)
    if overlap <= 0.0 or overlap >= map_w / 2.0:
        raise SystemExit('--overlap은 0보다 크고 map 폭의 절반보다 작아야 합니다')

    # 실측 결과를 실수로 덮어쓰지 않게 막는다.
    layout_path = out_dir / 'parking_layout.yaml'
    existing = [p for p in (
        out_dir / 'homography_cam0_rectified.npy',
        out_dir / 'homography_cam2_rectified.npy',
        layout_path) if p.exists()]
    if existing and not args.force:
        print('이미 파일이 있습니다. 실측 결과를 덮어쓰지 않도록 중단합니다.')
        for path in existing:
            print('   ', path)
        print('\n그래도 덮어쓰려면 --force 를 붙이세요.')
        return 1

    # ---- 1. 두 카메라의 H ------------------------------------------------
    split = map_w / 2.0
    cam0_x = (0.0, split + overlap / 2.0)
    cam2_x = (split - overlap / 2.0, map_w)

    cameras = {
        'cam0': affine_homography(cam0_x[0], cam0_x[1], 0.0, map_h,
                                  args.width, args.height),
        'cam2': affine_homography(cam2_x[0], cam2_x[1], 0.0, map_h,
                                  args.width, args.height),
    }

    for name, matrix in cameras.items():
        npy_path = out_dir / f'homography_{name}_rectified.npy'
        save_npy_atomic(npy_path, matrix)
        metadata = {
            'format': 'pixel_to_map_metre_homography_v1',
            'synthetic': True,
            'note': ('scripts/make_dummy_calibration.py가 만든 임시 파일. '
                     '실제 렌즈/설치각과 무관하므로 좌표값은 의미 없음.'),
            'camera_label': name,
            'image_width_px': args.width,
            'image_height_px': args.height,
            'homography_scale_to_m': 1.0,
            'references': corner_references(matrix, args.width, args.height),
            'reprojection_errors_m': [0.0, 0.0, 0.0, 0.0],
            'reprojection_rms_m': 0.0,
            'reprojection_max_m': 0.0,
            'layout_file': str(layout_path),
        }
        write_text_atomic(
            str(npy_path.with_suffix('.json')),
            json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')

    # ---- 2. 주차면 / 대기영역 -------------------------------------------
    y_near = map_h - 1.8
    y_far = map_h - 0.2
    half_width = 0.35
    # cam0 전용 2칸, 겹침 1칸, cam2 전용 1칸
    plan = [('P1', 0.8), ('P2', 1.8), ('P3', split), ('P4', map_w - 1.4)]
    slots, polygons = [], []
    for slot_id, center_x in plan:
        slot, corners = make_slot(slot_id, center_x, y_near, y_far, half_width)
        slots.append(slot)
        polygons.append(corners)

    waiting_half = overlap / 2.0
    waiting = [
        (split - waiting_half, 0.3),
        (split + waiting_half, 0.3),
        (split + waiting_half, 1.1),
        (split - waiting_half, 1.1),
    ]

    write_text_atomic(str(layout_path), render_parking_layout_yaml(
        slots, waiting, slot_polygons=polygons,
        map_width_m=map_w, map_height_m=map_h,
        map_resolution_m=float(args.map_resolution)))

    # ---- 3. 자체 검증 ----------------------------------------------------
    coverage = {
        name: image_corner_coverage(
            matrix, args.width, args.height,
            margin_px=float(args.coverage_margin_px))
        for name, matrix in cameras.items()
    }

    print('생성 완료:', out_dir)
    for name in cameras:
        print(f'  homography_{name}_rectified.npy (+ .json)')
    print(f'  parking_layout.yaml')
    print()
    print(f'map {map_w:.1f} x {map_h:.1f} m | 영상 {args.width}x{args.height}px')
    for name, polygon in coverage.items():
        xs = [p[0] for p in polygon]
        ys = [p[1] for p in polygon]
        print(f'  {name} 시야: x {min(xs):.2f}~{max(xs):.2f} m, '
              f'y {min(ys):.2f}~{max(ys):.2f} m')
    ox = (max(min(p[0] for p in coverage['cam2']) for _ in [0]),
          min(max(p[0] for p in coverage['cam0']) for _ in [0]))
    print(f'  겹침 구간: x {ox[0]:.2f}~{ox[1]:.2f} m (폭 {ox[1]-ox[0]:.2f} m)')
    print()
    print('슬롯별 관측 카메라:')
    ok = True
    for slot, polygon in zip(slots, polygons):
        centroid = polygon_centroid(polygon)
        seen = [name for name, cov in coverage.items()
                if point_in_polygon(centroid[0], centroid[1], cov)]
        if not seen:
            ok = False
        print(f'  {slot.slot_id} 중심 ({centroid[0]:.2f}, {centroid[1]:.2f}) '
              f'-> {", ".join(seen) if seen else "!! 어떤 카메라도 못 봄"}')
    centroid = polygon_centroid(waiting)
    seen = [name for name, cov in coverage.items()
            if point_in_polygon(centroid[0], centroid[1], cov)]
    print(f'  대기영역 중심 ({centroid[0]:.2f}, {centroid[1]:.2f}) '
          f'-> {", ".join(seen) if seen else "!! 어떤 카메라도 못 봄"}')
    if not ok:
        print('\n경고: 시야 밖 슬롯이 있습니다. --overlap 또는 map 크기를 조정하세요.')
        return 1

    print()
    print('주의: 이 H는 실제 카메라와 무관한 임시값입니다.')
    print('      좌표는 의미 없고, 파이프라인 연결 확인 용도로만 쓰세요.')
    print('      실측 후 bev_layout_calibration.launch.py가 그대로 덮어씁니다.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
