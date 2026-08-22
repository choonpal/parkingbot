#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
exec python3 dual_tile_homography_gui.py "$@"
