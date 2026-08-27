# Production startup heartbeat·freshness·operations 복구

## 목적

2026-08-28 production restart에서 Front/Rear가 공통으로
`SERVO_ATTACH ACK → HEARTBEAT_TIMEOUT → hardware_ready=true → FAULT/ESTOP`
순서로 실패한 원인을 실제 로그와 firmware/ROS/운용 코드를 대조해 제거했다.
Watchdog, ESTOP latch, command freshness threshold는 완화하지 않았다.

## 확인된 root cause

### 1. 이전 Linux 세션과 새 bridge startup의 경계 부재

- STM32는 첫 `HB`와 `V`를 받은 뒤에는 각각 300 ms/250 ms watchdog을 계속
  감시한다. 전원 최초 부팅에서 아직 두 frame을 받지 않은 상태는 startup guard가
  모터를 정지한 채 기다리므로, 단순히 ROS가 늦게 뜬 것만으로 timeout되지 않는다.
- 계획 restart는 STM32 전원을 끄지 않는다. 이전 bridge가 사라진 뒤에도 STM32
  RAM의 `heartbeat_seen/command_seen`은 남아 300 ms 뒤
  `HEARTBEAT_TIMEOUT`을 queue한다.
- 기존 bridge는 serial open 직후 `SERVO_ATTACH`를 먼저 보내고, heartbeat ACK가
  아닌 attach ACK도 `last_ack_time`으로 인정해 `hardware_ready=true`를 만들었다.
- state machine은 모든 `ERR,*`를 영구 hardware fault로 바꾸고 `/emergency_stop`을
  발행한다. 따라서 이전 세션 timeout 한 건이 실제 STM32 ESTOP latch까지 확대됐다.
- production UART 로그에는 `ACK,SERVO_ATTACH`가 있었지만 main branch firmware에는
  `S,attach` parser가 없었다. 배포 binary와 authoritative source가 달랐으며, 검증된
  attach parser를 main firmware로 회수했다.

### 2. stale stamp는 NTP/기본 RTT가 아니라 backlog와 진단 부하였다

- `rigid_body_sync_node.publish_twist`,
  `individual_move_node.new_velocity_command`, `fleet_manager.publish_state`는 모두
  publish 직전에 새 stamp를 만든다. message/stamp 재사용은 없었다.
- pose fusion의 `/odom`은 50 Hz인데 RELIABLE depth 20이었다. 느려진 consumer가
  최대 0.4 s의 과거 pose를 순서대로 받을 수 있었고, 역행/중복 wheel stamp를
  폐기하면서도 그 과거 stamp로 `/odom`을 한 번 더 publish했다.
- `/fleet/state`도 1 Hz RELIABLE depth 10이라 superseded state를 queue할 수 있었다.
- 기존 상태 수집은 최대 23개의 `ros2 topic echo`를 worker 3개로 12초씩 실행했다.
  실제 run의 `state_monitor.jsonl` 연속 record 사이에는 약 66초가 있었고, 세 개의
  DDS participant가 거의 상시 교체되며 저전력 RPi까지 discovery 압력을 줬다.
- `/cmd_vel` endpoint 자체는 이미 BEST_EFFORT depth 1이고 publisher도 publish 직전
  stamp를 생성했다. cmd/odom/ultrasonic이 같은 run에서 함께 늦어진 사실은 개별
  stamp 생성 버그가 아니라 executor/DDS pressure라는 결론과 일치한다.

### 3. local ROS CLI 환경 누락

`snapshot`과 `incident_snapshot`이 호출 shell의 `ros2`를 직접 실행했다. PATH에
실행 파일만 있고 Python underlay가 source되지 않은 shell에서는
`PackageNotFoundError: ros2cli`가 발생했다. `ros2` 존재 여부만 보던 재실행 guard도
이 경우를 검출하지 못했다.

### 4. PROCESS_EXITED 오판

원격 stack은 tmux detached session 안에서 유지되어 SSH 종료와 독립적이었다.
그러나 monitor는 `tmux ... | grep`의 모든 non-zero 결과(SSH timeout, session
조회 실패, 실제 exit)를 같은 `False`로 바꿔 `PROCESS_EXITED`를 만들었다. pane PID,
dead status, child return code는 저장하지 않아 사후 구분도 불가능했다. Readiness
timeout 자체는 stack을 정리하지 않았고, launch command 실패 때만 이미 시작한
session을 정리했다.

## 변경 내용

### STM32/bridge handshake

새 startup 순서는 다음과 같다.

```text
SERIAL_CONNECTED
→ HELLO(session_id)
→ matching HELLO ACK
→ heartbeat 전송 및 matching ACK
→ zero V로 command watchdog 시작
→ SERVO_ATTACH 및 ACK
→ 양쪽 ultrasonic fresh
→ hardware_ready=true / IDLE 유지
```

- `HELLO`는 먼저 모터를 즉시 정지하고 이전 세션의 pending
  `HEARTBEAT_TIMEOUT`/`COMMAND_TIMEOUT`만 정리한다.
- physical `estop_latched`와 다른 hardware error는 절대 clear하지 않는다.
- HELLO 전에 UART로 이미 전송된 위 두 comm timeout만 bridge가 startup 정보로
  격리한다. `ESTOP_LATCHED`와 다른 error, HELLO ACK 이후의 timeout은 기존처럼
  state machine에 `ERR`로 전달된다.
- `hardware_ready`는 matching HELLO/HB ACK, command channel, servo attach,
  heartbeat freshness, ultrasonic freshness, no active fault를 모두 요구한다.
- UART read/write/overflow는 command·servo channel을 fail-closed하고 port를 닫는다.
  운용 중 transport fault는 자동 reconnect로 숨기지 않고 bridge 재시작의 새 HELLO를
  요구한다.
- watchdog 상수는 heartbeat 300 ms, command 250 ms로 유지했다.

### Freshness/QoS

- wheel odom, fused odom, ultrasonic 및 모든 production odom consumer를
  BEST_EFFORT KEEP_LAST depth 1로 통일했다.
- `/fleet/state`와 consumer는 RELIABLE KEEP_LAST depth 1로 통일했다.
- hardware fault/status는 RELIABLE TRANSIENT_LOCAL depth 1로 보존한다.
- 역행/중복 wheel odom은 과거 stamp를 재발행하지 않고 완전히 폐기한다.
- safety freshness threshold는 변경하지 않았다.

### Operations/diagnostics/lifecycle

- 모든 local ROS CLI는 공통 helper를 통해 `/opt/ros/humble/setup.bash`, controller
  workspace overlay, domain/local-only/RMW를 source/export한 `bash -lc`에서 실행한다.
- 상태 snapshot은 23개 CLI process 대신 한 개의 bounded rclpy observer로 수집한다.
- tmux pane에 `remain-on-exit`을 설정하고 PID/dead status/return code를 조회한다.
  SSH 실패는 `UNKNOWN`, 외부 session 제거는 `SESSION_MISSING`, pane dead만
  `EXITED`로 분류한다.
- incident에 role, reason, pid, process state/alive, return code, hardware ready,
  robot state, motion fault, timestamp, launch command를 기록한다.
- startup은 조건별 `[WAIT]/[OK]/[FAIL]`을 출력하고 3초 discovery grace 뒤 이미
  명확한 hardware fault가 있으면 전체 timeout 전에 fail-fast한다.

## Safety 영향

- heartbeat/command watchdog과 모든 기존 threshold 유지
- 실제 heartbeat loss는 여전히 `ERR,HEARTBEAT_TIMEOUT → FAULT → ESTOP`
- 실제 ESTOP latch는 HELLO/restart로 복구 불가; 수동 MCU reset/power-cycle 필요
- unknown firmware protocol, bad attach pulse, UART error는 ready가 되지 않음
- planned restart만 ESTOP을 새로 latch하지 않으며 bridge zero command와 STM32
  250/300 ms watchdog이 정지를 보장한다. `robot_stop`은 계속 ESTOP을 발행한다.

## 테스트

- STM32 C source를 host gcc로 compile/run하여 다음을 검증했다.
  - 이전 세션 heartbeat timeout → HELLO로 comm state만 복구
  - HELLO → HB ACK → zero V → attach 정상 startup
  - 현재 세션 heartbeat 301 ms loss → `HEARTBEAT_TIMEOUT`
  - ESTOP 뒤 HELLO → `ESTOP_LATCHED` 유지
- bridge unit test에서 frame 순서, stale pre-session fault 격리 범위, attach/readiness,
  real ESTOP을 검증했다.
- clean environment에서 `ros2 node list`, `topic list`, `topic echo` helper가 underlay와
  overlay를 source하는지 검증했다.
- detached tmux pane이 start shell 반환 뒤에도 살아 있고, 종료 후 실제 exit code
  `7`이 보존되는지 통합 검증했다.
- clean/non-interactive ROS 환경에서 실제 `ros2 node list`, `ros2 topic list -t`,
  `ros2 topic echo --once`와 단일 snapshot helper를 실행해 모두 exit code 0을
  확인했다.
- 전체 package/operations pytest는 `603 passed, 2 skipped`, ROS package colcon
  build는 `1 package finished`로 통과했다. skip 2건은 optional dependency와
  기본 sandbox에서 허용되지 않는 tmux socket 통합 시험이며, 후자는 권한을 갖춘
  별도 실행에서 통과했다.

## 배포 및 실차 확인

1. Front/Rear firmware와 ROS package를 같은 release로 배포한다. 구 firmware는
   HELLO를 `UNKNOWN_COMMAND`로 거부하므로 혼합 배포는 fail-safe지만 기동 불가다.
2. 현재 STM32가 이미 ESTOP을 받은 상태라면 모터를 받침대에 올리고 물리 안전을
   확보한 뒤 MCU reset/power-cycle한다. 새 firmware flash/reset도 RAM latch를
   초기화한다.
3. `robot_doctor`, `robot_restart` 후 HELLO/HB/attach/ready 순서를 양쪽 로그에서
   확인한다.
4. 운용 중 heartbeat를 의도적으로 끊는 시험은 받침대에서 수행하고 300 ms 이후
   FAULT/ESTOP 및 수동 reset 요구를 확인한다.

## 남은 위험

- 실제 UART scheduling, USB serial reset, Fast DDS Wi-Fi 손실률은 실차 계측이
  필요하다.
- 3선식 servo는 실제 각도를 읽을 수 없으므로 restart는 반드시 무하중/IDLE에서
  수행해야 한다.
- tmux pane과 remote log metadata는 새 run부터만 유효해 과거 incident의 실제
  종료 원인을 소급 확정할 수 없다.
