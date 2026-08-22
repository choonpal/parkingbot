#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
DEST="${1:-$HOME/.ros/adaptive_valet_bot}"
mkdir -p "$DEST"
for f in \
  output/homography_cam0_rectified.npy \
  output/homography_cam2_rectified.npy \
  output/dual_homography_summary.json \
  output/parking_layout.yaml; do
  [ -f "$f" ] || { echo "missing: $f"; exit 1; }
done
cp output/homography_cam0_rectified.npy "$DEST/"
cp output/homography_cam2_rectified.npy "$DEST/"
cp output/dual_homography_summary.json "$DEST/"
cp output/parking_layout.yaml "$DEST/"
echo "copied to: $DEST"
