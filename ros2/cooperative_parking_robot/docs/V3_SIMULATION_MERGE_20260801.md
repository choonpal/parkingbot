# V3 simulation findings merged into the hardware package

This 1.7.0 package is based on
`v8_front_first_logic_review_20260725.zip`. The original archive is kept
unchanged. Only changes that remain meaningful on the real robots were
merged; Gazebo worlds, adapters, truth topics, and synthetic dimensions were
not copied.

## Merged behavior

- Front and Rear can stage and start the longitudinal axle scan concurrently.
- Split-nearest-end exit remains the default. Shared-direction synchronized
  exit is available through a launch parameter.
- OpenCV 4.6 uses the safe legacy ArUco parameter factory instead of the
  constructor combination that crashed during image detection.
- Rear ArUco brightness gain and yaw convention are configurable. Hardware
  compatible defaults remain `gray_gain=1.0`, `yaw_sign=1.0`.
- PoseEKF publishes reliable odometry so reliable mission/control subscribers
  cannot become silently disconnected by a QoS mismatch.

## Launch defaults

- `simultaneous_entry:=true`
- `same_direction_exit:=false`
- `same_direction_exit_sign:=1`
- `exit_sync_gain:=0.15`
- `yaw_sign:=1.0` and `gray_gain:=1.0` in `rear_robot.launch.py`

Use `simultaneous_entry:=false` to recover the original Front-first entry.

## Deliberately not merged

- Gazebo `sim_io`, wheel-odometry adapter, worlds, marker textures, and truth
  validation topics.
- The simulation-only 1.5x speed scale and enlarged wheel-track geometry.
- Gazebo camera calibration and homography files.
- A simulated vehicle width value. The real deployment must set measured
  outer dimensions including tyres and moving gripper/robot projections.

## Known follow-up from off-axis testing

The validated controller reaches staging and completes the scan from a
25 cm lateral offset. However, staging currently uses a 25 mm radial position
tolerance, and the transition to `SCAN_IN` does not independently re-check the
12 mm centerline tolerance. One test entered with a 16.3 mm Rear lateral
estimate and corrected it during scanning. Add a stable pre-scan alignment
gate before claiming strict centerline-before-entry behavior.

## Validation

Run from this package root:

```bash
python3 -m pytest -q
```

The simulation evidence remains under the separate V3 workspace:
`${HOME}/gz_ws_v8_ekf_scanin_v3_20260731/validation`.
