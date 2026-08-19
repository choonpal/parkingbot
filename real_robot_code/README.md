# Real robot code snapshot

Snapshot date: 2026-08-19

This directory preserves the current PC-side code used for the real parking robots before it is integrated into the existing `stm32/` and `ros2/` code.

- `stm32/parking_robot_f401/`: current STM32F401 project working tree
- `rpi/tools/`: Raspberry Pi manual-drive and hardware smoke-test tools

The `robot-2` stop-only serial test passed before this snapshot was made. It received 26 telemetry frames with all wheel RPM/PWM values at zero and the ArUco robot servo start position at `2600/400 us`.

The STM32 project currently selects the ArUco profile with `ROBOT_HAS_ARUCO_MARKER=1`. Change that build profile deliberately when flashing the no-marker robot.
