#!/usr/bin/env bash
set -eu
cd "$(dirname "$0")"
MODE="all"
COPY_LAYOUT="false"
if [ "${1:-}" = "--cam2-only" ]; then
  MODE="cam2"
  shift
fi
if [ "${1:-}" = "--include-layout" ]; then
  COPY_LAYOUT="true"
  shift
fi
DEST="${1:-$HOME/.ros/adaptive_valet_bot}"
mkdir -p "$DEST"

if [ "$MODE" = "cam2" ]; then
  for f in \
    output/homography_cam0_rectified.npy \
    output/homography_cam2_rectified.npy \
    output/dual_homography_summary.json; do
    [ -f "$f" ] || { echo "missing: $f"; exit 1; }
  done
  if [ ! -f "$DEST/homography_cam0_rectified.npy" ]; then
    echo "runtime CAM0 missing: $DEST/homography_cam0_rectified.npy"
    echo "CAM0+CAM2 전체 복사를 먼저 실행하세요."
    exit 2
  fi
  if ! cmp -s \
      output/homography_cam0_rectified.npy \
      "$DEST/homography_cam0_rectified.npy"; then
    echo "refusing CAM2-only copy: output/runtime CAM0 H mismatch"
    echo "검증한 H 쌍을 유지하려면 CAM0+CAM2 전체 복사를 실행하세요."
    exit 2
  fi
  STAMP="$(date +%Y%m%d_%H%M%S)"
  if [ -f "$DEST/homography_cam2_rectified.npy" ]; then
    cp -p "$DEST/homography_cam2_rectified.npy" \
      "$DEST/homography_cam2_rectified.backup_${STAMP}.npy"
  fi
  cp output/homography_cam2_rectified.npy "$DEST/"
  cp output/dual_homography_summary.json "$DEST/"
  echo "CAM2 only copied to: $DEST (CAM0 untouched)"
  exit 0
fi

for f in \
  output/homography_cam0_rectified.npy \
  output/homography_cam2_rectified.npy \
  output/dual_homography_summary.json; do
  [ -f "$f" ] || { echo "missing: $f"; exit 1; }
done
cp output/homography_cam0_rectified.npy "$DEST/"
cp output/homography_cam2_rectified.npy "$DEST/"
cp output/dual_homography_summary.json "$DEST/"
if [ "$COPY_LAYOUT" = "true" ]; then
  [ -f output/parking_layout.yaml ] || {
    echo "missing: output/parking_layout.yaml"; exit 1;
  }
  STAMP="$(date +%Y%m%d_%H%M%S)"
  if [ -f "$DEST/parking_layout.yaml" ]; then
    cp -p "$DEST/parking_layout.yaml" \
      "$DEST/parking_layout.backup_${STAMP}.yaml"
  fi
  cp output/parking_layout.yaml "$DEST/"
  echo "layout copied explicitly (previous layout backed up)"
else
  echo "runtime parking_layout.yaml preserved; use --include-layout to replace it"
fi
echo "copied to: $DEST"
