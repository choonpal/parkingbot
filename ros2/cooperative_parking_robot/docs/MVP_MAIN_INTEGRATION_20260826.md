# MVP map/HOME integration on hardened `main`

This integration applies the selected behavior from
`feature/retrieval-integration-mvp` on top of the current hardened `main`
without replacing the latest camera, coverage, calibration, or deployment
safety fixes.

## Integrated behavior

- OccupancyGrid coverage is `x=-0.40..4.40 m`, `y=-0.80..3.83 m`
  (`4.80 x 4.63 m`) while the registered waiting pose and slot coordinates stay
  in the existing map frame.
- A*, footprint checks, corridor checks, source-vehicle clearing, YOLO map
  rasterization, and dual-CCTV map rasterization all use
  `OccupancyGrid.info.origin` consistently.
- The BEV registration page exposes map origin, width, and height; saving or
  appending a layout preserves these values in YAML, metadata, and preview.
- Fleet planning supports `planning_validation_mode=enforce|warn_only`.
  The MVP runtime layout selects `warn_only`; A* absence, stale/missing mission
  data, E-stop, watchdog, hardware faults, and live rigid-body limits remain
  blocking conditions.
- Robot HOME poses are Front `(3.60, 0.60, 180 deg)` and Rear
  `(3.60, 0.20, 180 deg)`. The final HOME route leg restores heading before
  `return_done` is published.
- Front-first approach first translates away from the peer at HOME and leaves
  vehicle-axis yaw convergence to `PRE_ALIGN` after clearance is available.
- Parking and retrieval retain staged rotation: A* ends outside the destination,
  the loaded pair aligns yaw at staging, then inserts along the destination axis.

## Compatibility layer

`mvp_integration_nodes.py` subclasses the latest `main` nodes instead of copying
older whole files. Console entry points for `bev_layout_calibrator`,
`cctv_merge`, `yolo_bev_map`, and `individual_move` route through this layer.
The original latest-main node files remain unchanged. This preserves current
behavior for:

- required-camera fail-closed operation;
- unknown occupancy outside live CCTV coverage;
- exact rectified-camera resolution checks;
- stale target/coverage rejection;
- current ArUco and deployment hardening.

## Runtime configuration

The packaged reference layout contains the expanded map, but real deployments
load the runtime file by default:

```text
~/.ros/adaptive_valet_bot/parking_layout.yaml
```

Copy the packaged layout or regenerate it with the updated registration launch,
then verify:

```yaml
map_origin_x_m: -0.400000
map_origin_y_m: -0.800000
map_width_m: 4.800000
map_height_m: 4.630000
planning_validation_mode: "warn_only"
```

## Verification

Run from the ROS 2 workspace after rebuilding:

```bash
colcon build --symlink-install --packages-select cooperative_parking_robot
source install/setup.bash
python3 -m pytest -q src/parkingbot/ros2/cooperative_parking_robot/test
ros2 topic echo /parking/map --once
ros2 topic echo /fleet/state
```

Check that `/parking/map.info.origin.position` is `(-0.4, -0.8)`, the Fleet
state reports `planning_validation_mode=warn_only`, and both robots finish HOME
at the registered positions with yaw near 180 degrees.
