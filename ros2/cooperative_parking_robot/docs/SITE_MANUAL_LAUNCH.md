# One-line manual production launch

The manual site launchers reuse the same measured-value file as `robotctl`:

`~/.config/parkingbot/production_hosts.env`

Copy `tools/production_hosts.env.example` to that path on Jetson, Rear RPi and
Front RPi, then fill the site values relevant to that machine. Keeping the
same file format means measured USB paths and geometry do not need to be typed
again on every `ros2 launch` command.

After sourcing ROS 2 Humble and the workspace overlay, start in this order:

```bash
# Jetson
ros2 launch cooperative_parking_robot site_jetson.launch.py

# Rear / robot-1
ros2 launch cooperative_parking_robot site_rear.launch.py

# Front / robot-2
ros2 launch cooperative_parking_robot site_front.launch.py
```

`site_jetson.launch.py` includes `cctv_server_dual.launch.py`,
`site_rear.launch.py` includes `rear_robot.launch.py`, and
`site_front.launch.py` includes `front_robot.launch.py`. They only translate
stored site values into the existing production launch arguments; control
algorithms and the underlying launches remain unchanged.

Rear camera behavior follows `REAR_ENABLE_INTERNAL_CAMERA`. When true, the
Rear launch opens the camera itself (current default 640x480). When false,
`REAR_EXTERNAL_CAMERA_COMMAND` is started by the wrapper and the underlying
Rear launch only consumes `REAR_CAMERA_TOPIC`.
