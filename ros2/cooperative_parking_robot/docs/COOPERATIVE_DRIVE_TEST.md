# 두 로봇 10 cm 협동 직진 시험

이 시험은 전체 주차 미션 전에 Front와 Rear가 같은 방향으로 움직이고, ArUco와
엔코더를 이용한 상대 간격 감시가 실제 하드웨어에서도 동작하는지 확인한다.
전체 상태머신, 그리퍼 제어, Fleet Manager, `rigid_body_sync`는 실행하지 않는다.

## 안전 전제

- robot-1과 robot-2는 메인 전원을 부분적으로 나눠 켤 수 없다. 전원을 켜기 전에
  두 로봇의 모든 바퀴를 받침대로 바닥에서 띄운다.
- 첫 실행은 반드시 공중 방향 확인으로 한다. 두 로봇 모두 같은 ROS `+x` 명령에서
  같은 실제 방향으로 회전하는 것을 확인한 후 바닥 시험으로 넘어간다.
- 웹 정지는 정상 정지용이다. 위험하면 물리 E-STOP과 모터 전원 차단을 먼저 쓴다.
- 시험 노드는 기본 정지 상태이며 웹에서 `시험 준비`와 `시작`을 순서대로 눌러야
  움직인다. 시험 거리는 10 cm, 속도는 0.0628 m/s, 최대 시간은 4초다.

0.0628 m/s는 현재 펌웨어에서 확인된 12 rpm 정상 구동점이다. 0.03 m/s처럼 더
낮은 명령은 모터가 부드럽게 저속 회전하는 대신 스톨 근처에서 덜컥거릴 수 있어
첫 시험 기본값으로 사용하지 않는다.

## 실행 구성

두 장비 모두 같은 격리된 `ROS_DOMAIN_ID`를 사용한다. 아래 예시는 142다.
대문자 값은 각 로봇에서 확인한 실측값과 안정 `/dev/serial/by-id/...` 경로로
바꿔야 한다.

robot-2 Front에서 먼저 실행한다.

```bash
export ROS_DOMAIN_ID=142
ros2 launch cooperative_parking_robot cooperative_drive_test_front.launch.py \
  serial_port:="FRONT_STABLE_SERIAL_BY_ID" \
  wheel_radius:="FRONT_MEASURED_WHEEL_RADIUS" \
  encoder_ppr:="FRONT_ENCODER_PPR" \
  lx:="FRONT_LX" ly:="FRONT_LY"
```

robot-1 Rear에서는 흰색 OV2710의 안정 `by-path`를 넣어 실행한다.
기본 운용값은 `1280x720 @ 12 fps`이며, ID0의 검은 정사각형 한 변은
`0.10 m`이다.

```bash
export ROS_DOMAIN_ID=142
ros2 launch cooperative_parking_robot cooperative_drive_test_rear.launch.py \
  serial_port:="REAR_STABLE_SERIAL_BY_ID" \
  camera_device:="/dev/v4l/by-path/REAR_WHITE_OV2710-video-index0" \
  camera_calib:="$HOME/ov2710_calib_23mm_white.npz" \
  width:=1280 height:=720 fps:=8.0 marker_size_m:=0.10 \
  wheel_radius:="REAR_MEASURED_WHEEL_RADIUS" \
  encoder_ppr:="REAR_ENCODER_PPR" \
  lx:="REAR_LX" ly:="REAR_LY"
```

같은 내부망의 브라우저에서 `http://robot-1.local:5006/`을 연다. 화면 하나에서
다음을 볼 수 있다.

- robot-1 카메라와 ID0 ArUco 오버레이
- 전방 거리, 좌우 편차, 상대 각도와 관측 신선도
- 양쪽 `hardware_ready`, 수동 제어권 ACK, 엔코더 odometry
- Front/Rear 이동거리와 현재 속도 명령
- 현재 판단, 시작을 막는 조건, 자동 정지 이유

## 시험 순서

1. 두 로봇 바퀴가 모두 공중에 있고 ID0 마커가 카메라에 보이는지 확인한다.
2. 화면에서 양쪽 하드웨어와 odometry가 `정상`인지 확인한다.
3. `1. 시험 준비`를 누른다. 양쪽 STM32가 수동 제어권을 받으면 `준비 완료`가 된다.
4. 손을 로봇에서 치우고 `2. 10 cm 시작`을 누른다.
5. 공중 시험에서 두 로봇 바퀴 방향이 같고 정지 후 계속 회전하지 않는지 확인한다.
6. 전원을 끈 상태에서만 받침대를 제거하고, 넓고 평평한 바닥에서 10 cm 시험을
   한 번 반복한다.

시험은 다음 중 하나라도 발생하면 양쪽에 0속도를 계속 보낸다.

- ArUco 또는 엔코더 정보가 오래됨
- 한쪽 하드웨어 준비나 수동 제어권 ACK가 끊김
- 초기 대비 간격 또는 좌우 변화가 3 cm 초과
- 초기 대비 상대 각도 변화가 5도 초과
- 양쪽 엔코더 이동거리 차이가 3 cm 초과
- 한쪽이 반대 방향으로 1 cm 넘게 움직임
- 한쪽이 10 cm에 도달하거나 최대 시간 4초 초과

`FAULT`, `STOPPED`, `COMPLETED`에서는 시험 노드가 수동 제어권을 유지하면서
0속도를 계속 발행한다. 상태를 확인한 뒤 `제어권 해제`를 눌러 `IDLE`로 돌아간다.
비상정지를 사용한 경우에는 소프트웨어로 해제하지 않는다.
