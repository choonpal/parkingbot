# 목표 차량 하부 진입·이탈 설계

이 문서는 실차용 v8의 `individual_move`, `ultrasonic_edge`,
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
  /front/odom /rear/odom \
  /front/cmd_vel /rear/cmd_vel \
  /front/ultrasonic_left /front/ultrasonic_right \
  /rear/ultrasonic_left /rear/ultrasonic_right \
  /front/wheel_center_s /rear/wheel_center_s \
  /front/wheel_detected /rear/wheel_detected \
  /front/wheel_aligned /rear/wheel_aligned \
  /emergency_stop
```

합격 기준은 다음과 같다.

- `TO_REAR_STAGING`이 차량 보호 영역 밖의 종축 standoff/queue에서 끝난다.
- Front `SCAN_IN` 전에 Rear queue 완료와 ID0 가시성이 확인된다.
- `SCAN_IN` 동안 차량 좌표 `d`와 yaw 오차가 허용치 안으로 수렴한다.
- `wheel_detected=true` 전에 좌우 센서 모두 entry와 exit edge를 만든다.
- 한쪽 센서만 검출되면 `wheel_aligned`가 나오지 않고
  `WHEEL_PAIR_NOT_DETECTED` 또는 스트림 timeout으로 FAULT가 난다.
- release 후 반드시 `EXIT_UNDERBODY`, `EXIT_TO_SIDE` 순서로 차량을 벗어난다.

## 알려진 입력 제한

제어기는 이제 목표 차량 yaw를 사용하지만, 현재 기본
`yolo_bev_map_node`는 `/parking/target_pose`의 orientation을 항상 yaw 0으로
발행한다. 차량이 x축과 평행하지 않은 실차 운용에는 상류 비전에서 실제
yaw를 추정해 넣어야 한다. 이 입력이 없으면 좌표계 기반 제어도 0도 차량으로
해석한다.
