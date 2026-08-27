# Production operation tooling

## 목적

분산 Jetson/Front/Rear production stack을 긴 ROS launch 명령 없이 기동하고,
현장에서 `robot_state`와 `robot_logs`만으로 readiness와 최초 fault를 빠르게
확인할 수 있도록 운용 계층을 추가했다. 제어/PID/STM32 protocol/mission 알고리즘은
변경하지 않았다.

## 추가 명령

- `robot_start`: host/config/revision 확인, Jetson → Rear → Front tmux 기동,
  ROS graph 확인, monitor 시작
- `robot_state [-w|--watch] [--json]`: 실제 safety/status topic의 bounded parallel
  snapshot, blocker 집계
- `robot_logs [jetson|front|rear|sync] [--follow]`: 최신 run과 remote log tail/follow
- `robot_stop`: `/emergency_stop` 발행 후 Front → Rear → Jetson session 종료
- `robot_restart`: stop 후 start. FAULT/STM32 latch는 reset하지 않음
- `robot_doctor`: SSH, ROS/overlay/package, tmux, NTP, serial/camera/runtime asset 확인

`tools/install_robot_commands.sh`가 `~/.local/bin`에 위 명령과 `robotctl`을
설치한다. `.bashrc`는 수정하지 않는다.

## Production 구조 조사 결과

- Jetson: `cctv_server_dual.launch.py`가 두 camera/rectify/YOLO, merge, Fleet,
  CCTV robot marker와 operator UI를 실행한다.
- Rear(`robot-1`): `rear_robot.launch.py`가 ID0 camera/tracker, individual control,
  state machine, STM32 bridge, ultrasonic과 pose fusion을 실행한다.
- Front(`robot-2`): `front_robot.launch.py`가 production rigid-body sync,
  individual control, state machine, STM32 bridge, ultrasonic과 pose fusion을 실행한다.
- 저장소에는 authoritative SSH address/user와 세 장비 workspace 절대경로가 없다.
  따라서 이를 추측하지 않고 site config 필수값으로 남겼다.

## Configuration

Installer가 `~/.config/parkingbot/production_hosts.env`를 mode 0600으로 생성한다.
host, controller/remote colcon workspace, device/model/calibration path와 runbook의
실측 geometry를 한 곳에서 읽는다. 경로는 remote `$HOME` 혼동을 막기 위해 절대
경로만 허용한다. 빈 필수값은 `[BLOCKED] KEY is not configured`로 종료된다.

## 상태와 logging architecture

상태는 `/fleet/state`, target/spec, `/cctv/merge_status`, 양쪽 robot state,
hardware ready/status, motion fault, localization, CCTV marker, `/sync/*`와
odom/map stream을 짧은 timeout으로 병렬 조회한다. watch는 화면을 refresh하며,
JSON mode는 incident 자동화에 사용한다.

각 장비 stdout/stderr와 manifest는
`~/.ros/parkingbot_logs/<run-id>/<role>/`에 저장된다. manifest에는 timestamp,
hostname, branch/SHA/status, ROS domain/RMW, workspace와 launch command가 들어간다.
중앙 monitor는 `state/state_monitor.jsonl`을 쓰고, startup grace 이후 새 motion/
sync/hardware/process-exit fault가 생기면 state, summary, ROS graph와 remote log
tail을 `incidents/`에 저장한다.

## 테스트 결과

- `PYTHONPATH=tools python3 -m pytest -q tools/test_robotctl.py`: 11 passed
- 기존 CCTV/runtime/P0 safety 관련 pytest: 52 passed
- Python `py_compile`: 통과
- `bash -n tools/install_robot_commands.sh`: 통과
- temporary HOME installer/wrapper/parser 확인: 통과
- `git diff --check`: 통과

ROS 2/SSH/tmux 실제 분산 통합은 현재 개발 shell에 `rclpy/ros2`와 세 장비 연결이
없어 실행하지 못했다. PASS로 간주하지 않는다.

## 실차 확인 항목

1. site config에 실제 SSH endpoint, 절대 workspace 및 실측값 입력
2. `robot_doctor`의 camera/serial/NTP/runtime asset 전 항목 통과
3. 세 장비 remote tmux와 log/manifest 생성 확인
4. DDS에서 state snapshot timeout과 watch 갱신 확인
5. 받침대/물리 ESTOP 준비 후 fault 주입으로 incident directory와 remote tail 확인
6. `/emergency_stop` 이후 STM32 manual reset 계약 확인
