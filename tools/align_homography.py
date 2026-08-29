#!/usr/bin/env python3
"""두 CCTV 가 같은 ArUco 를 다른 위치로 보고할 때, 그 차이를 없앤다.

두 카메라가 같은 마커를 서로 다른 map 좌표로 낸다면 원인은 둘 중 하나다.

  1. 시차   — 물체 높이 때문. 위치마다 크기와 방향이 달라진다.
  2. 등록   — 기준점 실측이 어긋남. 위치와 무관하게 **일정한** 차이가 난다.

이 도구는 2번만 다룬다. 여러 마커의 차이가 서로 비슷하면 강체 평행이동으로
보고, 기준 카메라에 맞추도록 다른 카메라의 homography 에 평행이동을
곱한다. 차이가 마커마다 크게 다르면 평행이동으로 설명되지 않으므로
적용을 거부하고 그 사실을 알린다 — 그 경우는 재등록이나 회전/축척 문제다.

    H_new = [[1,0,dx],[0,1,dy],[0,0,1]] @ H_old

homography 파일을 직접 고치므로 preview 뿐 아니라 yolo_bev_map,
cctv_robot_marker 등 그 파일을 쓰는 모든 노드가 함께 맞춰진다.

사용법:
    # 먼저 확인만 (파일을 바꾸지 않음)
    python3 align_homography.py

    # 실제로 적용
    python3 align_homography.py --apply

    # 기준 카메라를 바꾸려면 (기본: cctv0)
    python3 align_homography.py --reference cctv2 --apply
"""

from __future__ import annotations

import argparse
import datetime
import json
import math
from pathlib import Path
import shutil
import sys
import urllib.request

import numpy as np


DEFAULT_URL = 'http://127.0.0.1:5008/api/info'
DEFAULT_DIR = Path('~/.ros/adaptive_valet_bot').expanduser()
# 마커마다 차이가 이보다 더 흩어지면 평행이동으로 설명되지 않는다.
SPREAD_LIMIT_M = 0.05


def fetch(url, samples, delay):
    """여러 번 읽어 마커 위치를 평균낸다. 한 프레임은 흔들린다."""
    import time
    acc = {}
    for i in range(samples):
        with urllib.request.urlopen(url, timeout=5) as response:
            info = json.load(response)
        for camera in info.get('cameras', []):
            label = camera['label']
            for marker in camera.get('markers') or []:
                world = marker.get('world')
                if world is None:
                    continue
                acc.setdefault(label, {}).setdefault(
                    int(marker['id']), []).append(tuple(world))
        if i + 1 < samples:
            time.sleep(delay)
    return {
        label: {mid: (sum(p[0] for p in pts) / len(pts),
                      sum(p[1] for p in pts) / len(pts))
                for mid, pts in markers.items()}
        for label, markers in acc.items()
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', default=DEFAULT_URL)
    ap.add_argument('--dir', default=str(DEFAULT_DIR))
    ap.add_argument('--reference', default='cctv0',
                    help='이 카메라를 참이라고 보고 나머지를 맞춘다')
    ap.add_argument('--samples', type=int, default=20)
    ap.add_argument('--delay', type=float, default=0.2)
    ap.add_argument('--apply', action='store_true',
                    help='없으면 계산만 하고 파일을 바꾸지 않는다')
    ap.add_argument('--force', action='store_true',
                    help='흩어짐이 커도 적용한다 (권장하지 않음)')
    args = ap.parse_args()

    base = Path(args.dir).expanduser()
    print(f'{args.samples}회 샘플링 중…')
    try:
        seen = fetch(args.url, args.samples, args.delay)
    except Exception as exc:                        # noqa: BLE001
        raise SystemExit(
            f'{args.url} 를 읽지 못했습니다: {exc}\n'
            'camera_preview 가 떠 있는지 확인하세요.')

    if args.reference not in seen:
        raise SystemExit(
            f'기준 카메라 {args.reference} 가 마커를 못 봤습니다. '
            f'본 카메라: {sorted(seen) or "없음"}')
    reference = seen[args.reference]
    print(f'\n기준 = {args.reference}')
    for mid, point in sorted(reference.items()):
        print(f'  ID{mid}: ({point[0]:.3f}, {point[1]:.3f})')

    exit_code = 0
    for label, markers in sorted(seen.items()):
        if label == args.reference:
            continue
        shared = sorted(set(markers) & set(reference))
        print(f'\n=== {label} ===')
        if not shared:
            print('  기준 카메라와 공유하는 마커가 없습니다 — 건너뜁니다.')
            continue
        diffs = []
        for mid in shared:
            a, b = reference[mid], markers[mid]
            d = (a[0] - b[0], a[1] - b[1])
            diffs.append(d)
            print(f'  ID{mid}: {label}({b[0]:.3f}, {b[1]:.3f}) -> '
                  f'기준({a[0]:.3f}, {a[1]:.3f})  차이 '
                  f'({d[0]:+.3f}, {d[1]:+.3f}) = {math.hypot(*d) * 100:.1f} cm')

        dx = sum(d[0] for d in diffs) / len(diffs)
        dy = sum(d[1] for d in diffs) / len(diffs)
        spread = max((math.hypot(d[0] - dx, d[1] - dy) for d in diffs),
                     default=0.0)
        print(f'  평균 이동량 ({dx:+.3f}, {dy:+.3f}) = '
              f'{math.hypot(dx, dy) * 100:.1f} cm')
        print(f'  마커 간 흩어짐 {spread * 100:.1f} cm '
              f'(기준 {SPREAD_LIMIT_M * 100:.0f} cm)')

        if len(shared) < 2:
            print('  [주의] 마커가 하나뿐이라 평행이동인지 확인할 수 없습니다.')
        if spread > SPREAD_LIMIT_M and not args.force:
            print('  [거부] 차이가 마커마다 다릅니다. 단순 평행이동이 아니므로')
            print('         회전/축척 오차이거나 시차 보정이 덜 된 것입니다.')
            print('         재등록하거나 높이 값을 먼저 맞추세요.')
            exit_code = 1
            continue

        path = base / f'homography_{label.replace("cctv", "cam")}_rectified.npy'
        if not path.is_file():
            print(f'  [건너뜀] 파일 없음: {path}')
            exit_code = 1
            continue
        if not args.apply:
            print(f'  (--apply 를 주면 {path.name} 에 반영합니다)')
            continue

        stamp = datetime.datetime.now().strftime('%m%d_%H%M%S')
        backup = path.with_suffix(f'.npy.bak_{stamp}')
        shutil.copy2(path, backup)
        H = np.load(path).astype(float)
        T = np.array([[1.0, 0.0, dx], [0.0, 1.0, dy], [0.0, 0.0, 1.0]])
        np.save(path, T @ H)
        print(f'  적용 완료 — 백업 {backup.name}')

    if not args.apply and exit_code == 0:
        print('\n확인만 했습니다. 반영하려면 --apply 를 붙이세요.')
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
