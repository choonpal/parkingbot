# Real robot code snapshot

Snapshot date: 2026-08-19
Integration status updated: 2026-08-20

This directory preserves the PC-side code recovered from the real parking robots before integration. The flash target after integration is `stm32/parking_robot`; the ROS deployment source is `ros2/cooperative_parking_robot`.

- `stm32/parking_robot_f401/`: current STM32F401 project working tree
- `rpi/tools/`: Raspberry Pi manual-drive and hardware smoke-test tools

Additional bounded real-robot diagnostics are kept in `rpi/tools`:

- `single_wheel_probe.py`: one wheel only, maximum absolute PWM 120 and maximum 2 seconds
- `front_keyboard_floor_test.sh`: starts the Front bridge and ROS keyboard teleop together, then sends stop/manual-off and shuts the bridge down on exit

The `robot-2` stop-only serial test passed before this snapshot was made. It received 26 telemetry frames with all wheel RPM/PWM values at zero and the ArUco robot servo start position at `2600/400 us`.

The recovered STM32 snapshot selects the ArUco profile with `ROBOT_HAS_ARUCO_MARKER=1`. Do not flash this historical snapshot as the integrated firmware. Select the Rear profile deliberately in `stm32/parking_robot` before flashing the no-marker robot.
