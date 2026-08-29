# 목표 차량 하부 진입·이탈 설계

이 문서는 실차용 `individual_move`, `ultrasonic_edge`,
`state_machine` 사이 계약을 설명한다.

## 좌표계와 진입 방식

`/parking/target_pose`의 위치와 yaw로 차량 좌표계를 만든다.

- `s`: 차량 앞쪽을 +로 하는 종축
- `d`: 차량 왼쪽을 +로 하는 횡축
- front 로봇: `-s` 차량 뒤에서 `+s`로 진입해 두 번째(front) 축 선택
- rear 로봇: Front 완료 뒤 `-s` 차량 뒤에서 `+s`로 진입해 첫 번째(rear) 축 선택

로봇은 차량 측면에서 우회 진입하지 않는다. 시작 위치부터 차량 뒤쪽 종축에
두며, Front는 rear-side standoff, Rear는 그보다 한 휠베이스 뒤의 ID0 관측
queue로 직접 이동한다. 시작점이 차량 뒤 보호경계 밖이 아니거나 이 직선이
차량 보호영역을 통과하면 진입을 거부한다. 초음파 쌍 에지는 `d=0` 종축으로
차량 하부에 들어갈 때만 검출한다.

## 명시적 모션 phase

- APPROACH: Front는 `WAIT_TARGET → TO_REAR_STAGING → READY_TO_SCAN`.
  Rear는 `WAIT_FRONT_STAGED → TO_REAR_STAGING(queue) → READY_TO_SCAN`.
- ALIGN: Front는 `WAIT_REAR_OBSERVATION(ID0 확인) → SCAN_IN(첫 축 통과,
  두 번째 축 선택) → CENTER_AXLE → ALIGNED`. Rear는
  `WAIT_FRONT_ALIGNED → SCAN_IN(첫 축 선택) → CENTER_AXLE → ALIGNED`.
- RETURN: `EXIT_UNDERBODY → EXIT_TO_SIDE → RETURN_HOME → RETURNED`

## 단계별 센서 활성화와 heartbeat

안전·상태·pose 노드와 STM32 bridge는 로봇 runtime 동안 계속 동작한다.
특히 STM32 heartbeat는 ROS timer가 아니라 bridge 내부 전용 producer가 100 ms
주기로 단일 UART writer에 넣는다. STM32의 heartbeat watchdog 300 ms와 command
watchdog 250 ms는 변경하지 않는다. 속도 상태 프레임은 기본 20 Hz이며 오래된
프레임을 쌓지 않고 최신 한 개만 유지한다.

계산량과 장치 부하는 다음처럼 phase에 맞춰 제한한다.

- 초음파: 기존과 같이 `PRE_ALIGN` 직전에 켜고 진입·차축 중심 맞춤이 끝나면 끈다.
- Rear ID0 카메라: 프로세스 시작 때 장치를 열어 두되 대기 중에는 기본 1 Hz로
  프레임만 버려 UVC를 예열한다. 활성화 전에는 영상 토픽을 발행하지 않는다.
- Rear ArUco: 대기 중에는 수신 영상을 처리하지 않는다. Rear가
  `READY_TO_SCAN`에 들어가면 `/rear/relative_vision_enable=true`로 카메라와
  ArUco를 함께 활성화한다.
- `approach_done`은 `/rear/relative_vision_ready=true`와 현재 ID0 pose/가시성이
  모두 확인된 뒤에만 발행한다. 준비 실패나 마커 부재는 시간 제한 뒤
  fail-closed로 끝나며 영상 없이 진입하지 않는다.
- `ALIGNED` 유지와 차량 하부 이탈까지 ID0를 사용하고, `EXIT_TO_SIDE`부터 다시
  standby로 내린다. Front 노드는 enable 토픽을 발행하지 않으며 Rear 이동
  노드 하나만 이 수명주기를 소유한다.

Rear launch는 카메라와 ArUco의 무거운 cold import를 먼저 끝낸 뒤 bridge를
시작한다. 조정 가능한 기본값은 `rear_camera_standby_fps=1.0`,
`rear_camera_activation_drop_frames=2`, `velocity_tx_rate_hz=20.0`,
`serial_write_timeout_s=0.05`이다. write timeout이나 partial write는 여전히 즉시
transport fault로 latch되며, 진단에는 실패 frame 종류·queue 대기·deadline
지연·pending 깊이를 포함한다.

실차 카메라는 숫자 index보다 `rear_camera_device:=/dev/v4l/by-id/...-video-index0`
를 사용한다. `rear_camera_device`가 비어 있을 때만 `rear_camera_id`가 fallback이
되며, 순간적인 숫자 장치 순서를 코드에 고정하지 않는다.
현재 Rear UVC는 MJPEG 1280x720@30만 안정적으로 협상되므로 장치 입력은
`rear_camera_fourcc:=MJPG rear_camera_capture_fps:=30.0`으로 열고, ROS 처리율은
별도 `rear_camera_fps:=8.0`으로 제한한다. 전체 주차장 CCTV는 상시 활성이고,
이 phase gate는 Rear 마커 카메라와 ArUco에만 적용된다.

실차 RPi launch는 프로세스 단위 CPU affinity를 강제하지 않는다. STM32 bridge에는
UART writer와 heartbeat producer뿐 아니라 DDS·executor thread도 함께 있으므로
프로세스 전체를 한 코어에 고정하면 오히려 300 ms watchdog을 넘길 수 있다. 전용
heartbeat thread와 bounded UART scheduler는 유지하되 Linux가 네 코어에 thread를
분산하도록 둔다.

## 정렬 후 정지 commissioning 모드

차량 하부 진입만 검증할 때 Front와 Rear launch에
`stop_after_align:=true`를 함께 지정한다. 이 모드에서는 각 로봇이
`wheel_aligned=true`에 도달한 뒤 `robot_state=ALIGN`,
`motion_phase=ALIGNED`에서 0속도를 유지하고 `/{role}/aligned_hold=true`를
발행한다. 상태기는 `LIFT` ready/commit을 발행하지 않으며, 같은 mission의 외부
`LIFT` commit도 무시하므로 grip·lift·drive 단계로 진행하지 않는다.

운용 성공 조건은 `/front/aligned_hold`와 `/rear/aligned_hold`가 모두 `true`이고,
양쪽 `cmd_vel`과 실제 바퀴가 정지했으며 `/mission/commit`에 `LIFT`가 없는 것이다.
이 상태에는 자동 이탈이 없으므로 차량을 들어 올리지 말고, 별도로 검증된 수동
회수 절차로 두 로봇을 꺼낸다.

현재 commissioning 기본 측면 offset은 `entry_side_offset_m=0.50`이다. 이 값과
아래 초음파-그리퍼 offset은 역할이 다르다. 특히
`left_sensor_to_gripper_x_m`와 `right_sensor_to_gripper_x_m`가 임시 0이거나
미측정이면 차축 중심 계산을 신뢰할 수 없으므로 자동 진입하지 않는다.

현재 phase는 `/{role}/motion_phase`에 발행한다. 이동 노드가 고정한 차량
좌표계는 `/{role}/active_target_pose`로 초음파 노드에 전달하므로, CCTV의
후속 jitter가 스캔 좌표계를 바꾸지 않는다.

## 안전 종료 조건

- 목표 자세 미수신·stale·비정상 quaternion
- 보호 영역을 피하는 접근/귀환 경로를 만들 수 없음
- 각 모션 phase 시간 초과
- 스캔 허용 거리 안에서 좌우 초음파 pair 미완성
- ALIGN 중 좌/우 Range 스트림 단절
- 이동 노드와 초음파 노드 사이 active target frame 미수신
- 이탈 후 전달 차량을 우회하는 데 필요한 `/parking/slot_pose` 미수신

이 조건은 `/{role}/motion_fault`로 발행되고 상태기는 즉시 `FAULT`와
`/emergency_stop=true`로 전환한다.

## 실차에서 반드시 측정할 launch 값

- `wheelbase`
- `vehicle_half_length_m`, `vehicle_half_width_m`
- `robot_clearance_m`
- `entry_standoff_m`; `entry_side_offset_m`, `entry_side`는 귀환 우회용
- `exit_distance_m`, `scan_overshoot_m`
- `left_sensor_to_gripper_x_m`, `right_sensor_to_gripper_x_m`
- `ultrasonic_threshold_m`, `ultrasonic_exit_hysteresis_m`

시작 시 다음 기하 조건을 검사한다.

1. standoff가 `vehicle_half_length + robot_length/2 + clearance`보다 커야 한다.
2. side offset이 `vehicle_half_width + robot_width/2 + clearance`보다 커야 한다.
3. standoff가 배정 축보다 차량 바깥쪽이어야 한다.
4. 축 정렬점에서 `exit_distance`만큼 나갔을 때 보호 영역 밖이어야 한다.
5. `wheelbase - robot_length`가 최소 로봇 간격보다 커야 한다.

기본값은 코드 검증용이며 실차 치수를 대신하지 않는다.

## rosbag 검증

다음 토픽을 한 bag에 기록한다.

```bash
ros2 bag record \
  /parking/target_pose /parking/slot_pose \
  /front/active_target_pose /rear/active_target_pose \
  /front/robot_state /rear/robot_state \
  /front/motion_phase /rear/motion_phase \
  /front/motion_fault /rear/motion_fault \
  /rear/relative_vision_enable /rear/marker_camera_ready \
  /rear/relative_vision_ready /sync/marker_visible /sync/relative_pose \
  /front/heartbeat_diagnostics /rear/heartbeat_diagnostics \
  /front/odom /rear/odom \
  /front/cmd_vel /rear/cmd_vel \
  /front/ultrasonic_left /front/ultrasonic_right \
  /rear/ultrasonic_left /rear/ultrasonic_right \
  /front/wheel_center_s /rear/wheel_center_s \
  /front/wheel_detected /rear/wheel_detected \
  /front/wheel_aligned /rear/wheel_aligned \
  /front/aligned_hold /rear/aligned_hold \
  /mission/commit \
  /emergency_stop
```

합격 기준은 다음과 같다.

- `TO_REAR_STAGING`이 차량 보호 영역 밖의 종축 standoff/queue에서 끝난다.
- Front `SCAN_IN` 전에 Rear queue 완료와 ID0 가시성이 확인된다.
- `SCAN_IN` 동안 차량 좌표 `d`와 yaw 오차가 허용치 안으로 수렴한다.
- `wheel_detected=true` 전에 좌우 센서 모두 entry와 exit edge를 만든다.
- 한쪽 센서만 검출되면 `wheel_aligned`가 나오지 않고
  `WHEEL_PAIR_NOT_DETECTED` 또는 스트림 timeout으로 FAULT가 난다.
- `stop_after_align=true`이면 양쪽 `aligned_hold=true`, 양쪽 속도 0이며
  `/mission/commit`에 LIFT가 없어야 한다.
- release 후 반드시 `EXIT_UNDERBODY`, `EXIT_TO_SIDE` 순서로 차량을 벗어난다.

## 알려진 입력 제한

제어기는 이제 목표 차량 yaw를 사용하지만, 현재 기본
`yolo_bev_map_node`는 `/parking/target_pose`의 orientation을 항상 yaw 0으로
발행한다. 차량이 x축과 평행하지 않은 실차 운용에는 상류 비전에서 실제
yaw를 추정해 넣어야 한다. 이 입력이 없으면 좌표계 기반 제어도 0도 차량으로
해석한다.
