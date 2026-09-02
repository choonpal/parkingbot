# v1.11.1 — Front/Rear relative synchronization hardening

## Why this change was required

The previous `rigid_body_sync_node` cached the most recent ID0 ArUco pose for
`aruco_timeout_s=0.30` and called `ScalarKalman.update()` on every 50 Hz control
cycle. A 12 FPS camera frame could therefore be treated as several independent
measurements, and one stalled frame could be reused up to about 15 times.
The same loop also used CCTV-corrected `/front/odom` and `/rear/odom` as if they
were pure encoder propagation and re-used that pair as a second measurement
while overhead markers were visible.

## Production behavior after v1.11.1

The `rigid_body_sync` console entry point now starts
`rigid_body_sync_safe_node.py`. The previous implementation remains available as
`rigid_body_sync_legacy` only for comparison and rollback.

The hardened estimator applies these contracts:

1. An ID0 source timestamp is consumed at most once.
2. Relative predict uses the cumulative pure `/front/wheel_odom` and
   `/rear/wheel_odom` poses, anchored to the map pose at mission initialization.
3. `/front/odom` and `/rear/odom` are only a temporary fallback if a raw wheel
   stream is unavailable or stale.
4. Direct `/front/cctv_pose` and `/rear/cctv_pose` observations are timestamp
   paired and consumed once; cached visibility booleans are not repeatedly
   corrected at 50 Hz.
5. Distance and yaw use hard innovation limits plus a sigma gate. One-frame
   glitches are rejected. A bounded, mutually consistent sequence can
   deliberately re-acquire a drifted filter.
6. Yaw predict/update uses shortest-angle residuals across the ±π boundary.
7. Process covariance grows only on a new odometry observation and scales with
   elapsed time and motion, not with controller-loop frequency.
8. Small PID deadbands reduce stationary hunting. Safety limits still use the
   unmodified raw error.

## Parameters that still require real-robot calibration

The `sync_*process*`, `sync_*measurement*`, innovation, consistency, and
re-acquisition values in `config/sync_params.yaml` are conservative startup
values. Before increasing speed, record at least:

- 30–60 seconds of stationary ID0 distance/yaw at the actual mounting distance;
- straight and lateral wheel-only motion with ID0 temporarily covered;
- repeated ID0 loss/re-acquisition near the nominal 0.785 m separation;
- stationary motor commands to confirm the 3 mm / 0.5° deadbands suppress
  hunting without hiding a mechanically meaningful error.

The status JSON now includes `relative_predictor`, `visual_decision`,
`visual_reason`, `visual_age_s`, `dist_std_mm`, and `yaw_std_deg` for telemetry
verification.
