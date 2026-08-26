# robot-2 키보드 · robot-1 ArUco 추종 시험

이 모드는 robot-2를 기준으로 키보드에서 전후·횡·회전을 입력하면 robot-1이
함께 움직이면서 두 로봇의 ArUco 상대 자세를 유지하는 빈 차체 시험이다. 전체
주차 상태기계, 그리퍼, 차량 하중 제어는 실행하지 않는다.

## 간격 기준

`추종 준비`를 누른 순간 Rear 카메라가 읽은 ID0의 `forward`, `lateral`, `yaw`를
그대로 기준값으로 저장한다. 특히 목표 축간 거리는 **그 순간 ArUco가 판단한
forward 값 자체**다. 로봇 길이, 카메라 offset, 명목 wheelbase를 더하지 않는다.

W/S/A/D 평행이동에서는 두 로봇에 같은 기본 속도를 주고, Q/E 회전에서는 저장한
ArUco forward 간격으로 강체 속도를 분배해 두 로봇 중점을 중심으로 돈다. 기준
대비 간격·좌우 오차에는 반대 방향의 작은 보정을 양쪽에 절반씩 적용한다.

## 안전 제한

- 한 키 입력의 deadman은 0.30초다. 브라우저 반복 입력이 끊기면 양쪽에 0속도를
  보낸다.
- ArUco, 양쪽 wheel odometry, `hardware_ready`, 수동 제어권 중 하나라도 오래되면
  둘 다 정지한다.
- 기준 대비 forward 또는 lateral 변화가 3cm, 상대 yaw 변화가 5도를 넘으면
  `FAULT`로 정지한다.
- 한 번 준비한 세션에서 어느 로봇이든 누적 30cm를 이동하면 정지한다. 더
  움직이려면 제어권을 해제하고 현재 ArUco 자세를 새 기준으로 다시 준비한다.
- `/front|rear/cmd_vel` 자동 발행자나 이 노드 외의 manual 명령 발행자가 있으면
  준비되지 않는다.
- 그리퍼 키 T/G는 비활성화되어 있다. 웹 비상정지는 STM32에 고정되므로 원인
  제거 뒤 전원을 다시 인가해야 한다.

## 실행

기존 3cm/10cm 시험 화면이 실행 중이면 먼저 해당 Rear/Front launch를 종료한다.
두 제어기를 동시에 실행하면 안 된다. 같은 격리 domain을 양쪽에 적용한다.

robot-2 Front:

```bash
export ROS_DOMAIN_ID=142
ros2 launch cooperative_parking_robot cooperative_drive_test_front.launch.py \
  serial_port:=/dev/serial/by-id/FRONT_STABLE_SERIAL
```

robot-1 Rear:

```bash
export ROS_DOMAIN_ID=142
ros2 launch cooperative_parking_robot cooperative_drive_test_rear.launch.py \
  serial_port:=/dev/serial/by-id/REAR_STABLE_SERIAL \
  camera_device:=/dev/v4l/by-path/REAR_WHITE_CAMERA-video-index0 \
  camera_calib:="$HOME/ov2710_calib_23mm_white.npz" \
  enable_drive_test_dashboard:=false \
  enable_keyboard_follow:=true
```

브라우저에서 다음을 연다.

- 키보드 추종과 판단: `http://robot-1.local:5007/`
- 카메라만 보기: `http://robot-1.local:5005/`

페이지를 한 번 클릭한 뒤 `추종 준비`를 누른다. 목표 간격이 현재 ArUco 값으로
표시되고 상태가 `키보드 추종 준비 완료`가 된 뒤에만 키를 사용한다.

| 키 | 동작 |
|---|---|
| W / S | 두 로봇 전진 / 후진 |
| A / D | 두 로봇 좌 / 우 횡이동 |
| Q / E | 현재 ArUco 간격을 반지름 관계로 사용한 중점 회전 |
| Space | 즉시 0속도, 준비 상태 유지 |
| 정지·제어권 해제 | 0속도 후 양쪽 manual 제어권 반환 |

## 가벼운 첫 확인 순서

1. `추종 준비` 직후 목표 간격과 현재 간격이 같은지 확인한다.
2. W를 짧게 한 번 눌러 양쪽이 같은 방향으로 움직이고 손을 떼면 즉시 서는지
   본다.
3. W와 S를 각각 짧게 눌러 기준 간격이 1cm 안팎으로 돌아오는지 본다.
4. A와 D를 한 번씩 확인한다.
5. Q/E는 마지막에 짧게 한 번만 확인한다. Front와 Rear의 횡속도가 반대가 되는
   것이 정상이며, 두 로봇이 각각 제자리 회전하는 동작은 정상이 아니다.

어느 단계든 화면이 `FAULT`가 되면 다시 키를 누르지 말고 표시된 간격·좌우·각도와
카메라 영상을 먼저 확인한다.
