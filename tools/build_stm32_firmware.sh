#!/usr/bin/env bash
set -euo pipefail

usage() {
  printf 'Usage: %s front|rear|all\n' "$0" >&2
  printf 'Set ARM_NONE_EABI_ROOT when arm-none-eabi-gcc is not on PATH.\n' >&2
}

PARKINGBOT_PROFILE_MODE="${1:-}"
case "$PARKINGBOT_PROFILE_MODE" in
  front|rear|all) ;;
  *)
    usage
    exit 2
    ;;
esac

PARKINGBOT_REPOSITORY=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
PARKINGBOT_FIRMWARE_DIR="$PARKINGBOT_REPOSITORY/stm32/parking_robot"
PARKINGBOT_ARTIFACT_DIR="$PARKINGBOT_FIRMWARE_DIR/build/production/artifacts"

cd "$PARKINGBOT_FIRMWARE_DIR"
cmake --preset production

case "$PARKINGBOT_PROFILE_MODE" in
  front)
    cmake --build --preset front --parallel
    PARKINGBOT_ARTIFACTS=(parking_robot_front)
    ;;
  rear)
    cmake --build --preset rear --parallel
    PARKINGBOT_ARTIFACTS=(parking_robot_rear)
    ;;
  all)
    cmake --build --preset all-profiles --parallel
    PARKINGBOT_ARTIFACTS=(parking_robot_front parking_robot_rear)
    ;;
esac

for PARKINGBOT_ARTIFACT in "${PARKINGBOT_ARTIFACTS[@]}"; do
  for PARKINGBOT_EXTENSION in elf hex bin; do
    PARKINGBOT_OUTPUT="$PARKINGBOT_ARTIFACT_DIR/$PARKINGBOT_ARTIFACT.$PARKINGBOT_EXTENSION"
    test -s "$PARKINGBOT_OUTPUT"
    sha256sum "$PARKINGBOT_OUTPUT"
  done
done
