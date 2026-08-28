# 강체 쌍 키보드 주행 빠른 가이드

robot-2 **Front**와 robot-1 **Rear**를 ArUco로 정렬한 뒤, 두 로봇을 하나의
강체처럼 `W/A/S/D/Q/E`로 움직이는 현장 시험 절차입니다.

> **현재 상태 (2026-08-28)**
>
> 오늘 시험 branch를 양쪽 로봇에 임시 배포해 바퀴를 띄운 상태의 `W/S` 방향과
> 정지 동작을 확인했습니다. `A/D/Q/E`, 지면 주행, 장시간 marker dropout 및 열
> 안정성은 아직 미확인입니다. robot-1이 중복 production launch와 함께 실행될 때
> 78.8°C까지 올라 최종 시험을 중단했습니다. 최신 main 통합본을 다시 배포하고
> 아래 전체 체크리스트를 통과하기 전에는 production 검증 완료로 간주하지 않습니다.
> 이 시험은 기존 `keyboard_follow`가 아닌 별도 `rigid_pair_teleop`을 사용합니다.

## 안전 원칙

- 첫 시험은 반드시 **바퀴를 지면에서 띄운 상태**로 수행합니다.
- 로봇 옆에 물리 E-STOP을 즉시 누를 사람이 있어야 합니다.
- 다른 주행 launch와 `cmd_vel` 발행자는 모두 종료합니다.
- 이상 동작 시 키보드나 웹 화면보다 **물리 E-STOP을 우선**합니다.
- 실제 카메라·ArUco·STM32를 사용하며 가짜 토픽은 발행하지 않습니다.

## 1. 현장 설정값

| 구분 | 값 |
| --- | --- |
| Rear | `robot@robot-1.local` |
| Front | `robot@robot-2.local` |
| 격리 시험 ROS domain | `142` |
| 오늘 검증한 로봇 workspace | `/home/robot/parkingbot` |
| 목표 중심 간격 | `78.5 cm` |
| Rear 카메라에서 보이는 ID0 raw forward | 약 `21.5 cm` |
| Front serial | `/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0667FF485270535067112920-if02` |
| Rear serial | `/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066AFF485270535067112511-if02` |
| Rear camera | `/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0` |
| Rear calibration | `/home/robot/ov2710_calib_23mm_white.npz` |
| Rear camera mode | `1280x720 @ 12 fps` |
| Front rear-face marker | `DICT_4X4_50 ID0`, black square `0.10 m` |

IP는 DHCP로 바뀌므로 SSH와 웹 접속에는 항상 `*.local` 이름을 사용합니다.

## 2. 배포 후 최초 1회 확인

양쪽 로봇에서 새 소스를 배포·빌드한 뒤 실행합니다.

```bash
cd /home/robot/parkingbot
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cooperative_parking_robot
source install/setup.bash

ros2 pkg executables cooperative_parking_robot | grep rigid_pair_teleop
```

Rear에서는 launch 인자도 확인합니다.

```bash
ros2 launch cooperative_parking_robot \
  cooperative_drive_test_rear.launch.py --show-args | \
  grep enable_rigid_pair_teleop
```

둘 중 하나라도 결과가 없으면 구버전이므로 다음 단계로 진행하지 않습니다.

## 3. 시작 전 배치

1. 두 로봇의 바퀴를 띄우고 물리 E-STOP을 준비합니다.
2. Front를 Rear 앞쪽에 대략 평행하게 놓습니다.
3. Rear 카메라가 Front의 ID0 marker 전체를 보도록 맞춥니다.
4. 두 로봇에서 다른 주행 프로그램이 모두 종료됐는지 확인합니다.
5. 운영 PC에서 SSH 터미널 두 개를 엽니다.

```bash
ssh robot@robot-2.local   # Front 터미널
ssh robot@robot-1.local   # Rear 터미널
```

robot-1에서는 domain이 달라도 CPU·카메라·STM32를 공유하므로 production launch가
함께 실행되면 안 됩니다. 다음 결과에는 이번 시험 launch 하나만 있어야 합니다.

```bash
pgrep -af 'rear_robot.launch.py|cooperative_drive_test_rear.launch.py'
vcgencmd measure_temp
vcgencmd get_throttled
```

가능하면 60°C 이하에서 시작하고 75°C에 접근하면 즉시 정지·종료합니다.

## 4. Front 실행

robot-2 터미널에서 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source /home/robot/parkingbot/install/setup.bash
export ROS_DOMAIN_ID=142

ros2 launch cooperative_parking_robot cooperative_drive_test_front.launch.py \
  serial_port:=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0667FF485270535067112920-if02 \
  wheel_radius:=0.05 encoder_ppr:=5182.0 \
  lx:=0.2225 ly:=0.21
```

## 5. Rear 실행

robot-1 터미널에서 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source /home/robot/parkingbot/install/setup.bash
export ROS_DOMAIN_ID=142

ros2 launch cooperative_parking_robot cooperative_drive_test_rear.launch.py \
  serial_port:=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066AFF485270535067112511-if02 \
  camera_device:=/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0 \
  camera_calib:=/home/robot/ov2710_calib_23mm_white.npz \
  width:=1280 height:=720 fps:=8.0 marker_size_m:=0.10 \
  wheel_radius:=0.05 encoder_ppr:=5182.0 \
  lx:=0.2225 ly:=0.21 \
  enable_drive_test_dashboard:=false \
  enable_rigid_pair_teleop:=true
```

`enable_drive_test_dashboard:=false`는 기존 주행 UI와 명령이 충돌하지 않게 하는
필수 설정입니다.

제어용 ID0 검출은 `1280x720 @ 8 fps` 원본을 그대로 사용합니다. 5007에 삽입되는
영상만 별도 `640x360 @ 4 fps` 토픽을 사용해 UI 때문에 pose가 밀리지 않게 합니다.

## 6. 카메라와 정렬 확인

운영 PC의 브라우저에서 엽니다.

- 카메라: `http://robot-1.local:5005/`
- 정렬·제어: `http://robot-1.local:5007/`

`5005`에서 영상이 계속 갱신되고 ID0 marker가 안정적으로 인식되는지 확인합니다.
그다음 로봇을 손으로 조금씩 옮겨 `5007`의 값을 맞춥니다.

| 항목 | 목표 |
| --- | ---: |
| 추정 중심 간격 | `78.5 ± 1.5 cm` |
| raw lateral | `0 ± 1.5 cm` |
| raw yaw | `0 ± 2°` |

화면에 `정렬 후보`가 나타나면 실제 차체도 평행한지, 케이블이나 차체가 간섭하지
않는지 눈으로 한 번 더 확인합니다. 이 표시는 안전을 보증하지 않습니다.

## 7. Arm과 키 조작

1. `5007`에서 blocker가 없는지 확인합니다.
2. **현재 자세 기준 준비**를 한 번 누릅니다.
3. `강체 쌍 제어 준비 완료`가 표시될 때까지 키를 누르지 않습니다.
4. 브라우저 화면을 클릭한 뒤 키를 짧게 사용합니다.

| 키 | 동작 |
| --- | --- |
| `W` / `S` | 전진 / 후진 |
| `A` / `D` | 왼쪽 / 오른쪽 횡이동 |
| `Q` / `E` | 중점 기준 반시계 / 시계 회전 |
| `Space` | 즉시 0속도, Arm 유지 |
| **정지·제어권 해제** | 정지 후 IDLE |
| **양쪽 비상정지** | 양쪽 STM32 E-STOP 고정 |

키 입력이 약 `0.30초` 끊기면 양쪽에 0속도가 발행됩니다.

## 8. 바퀴를 띄운 첫 시험

각 키를 **아주 짧게** 누르고 매 단계 `Space`로 정지합니다.

- [ ] `W`: 양쪽 모두 전진 방향
- [ ] `S`: 양쪽 모두 후진 방향
- [ ] `A`, `D`: 횡이동 방향 정상
- [ ] `Q`, `E`: 두 로봇이 서로 싸우거나 급격히 비틀리지 않고 중점 회전
- [ ] 키를 놓으면 즉시 감속하고 약 `0.30초` 안에 정지
- [ ] `Space`와 **정지·제어권 해제**가 모두 정상 동작
- [ ] 카메라·ArUco·wheel odom·STM32 ready가 끊기면 주행이 차단됨

방향이 반대이거나 진동·급가속·계속 움직임이 하나라도 보이면 물리 E-STOP을 누르고
시험을 종료합니다. 원인을 고치기 전에는 바퀴를 지면에 내리지 않습니다.

## 9. 지면 첫 시험

바퀴를 띄운 시험을 모두 통과한 뒤에만 진행합니다.

1. 사람과 장애물을 치우고 평평한 공간을 확보합니다.
2. 다시 정렬하고 Arm합니다.
3. `W`를 짧게 눌러 수 cm만 이동한 뒤 `Space`로 정지합니다.
4. 중심 간격과 lateral/yaw가 유지되는지 확인합니다.
5. `S`, `A`, `D`를 차례로 짧게 확인합니다.
6. `Q`, `E`는 마지막에 최소 입력으로 확인합니다.

기본 설정은 한 세션에서 어느 한 로봇이라도 누적 `30 cm`를 이동하면 정지합니다.

## 10. 자주 막히는 경우

| 증상 | 우선 확인 |
| --- | --- |
| 웹 페이지가 안 열림 | Rear launch, 동일 Wi-Fi, `getent hosts robot-1.local` 결과 |
| `마커 찾기` | ID0 전체가 영상 안에 있는지, 가림·반사광 여부 |
| `hardware_ready` 없음 | serial 경로, STM32 전원, Front/Rear role |
| `수동 제어권 확인 안 됨` | 다른 teleop·dashboard·주행 launch 종료 |
| `다른 주행 발행자 존재` | 아래 명령으로 불필요한 publisher 확인 후 종료 |
| UI 응답이 간헐적으로 끊김 | 중복 launch, robot-1 온도, CPU 사용률을 먼저 확인 |

```bash
ros2 topic info /front/cmd_vel --verbose
ros2 topic info /rear/cmd_vel --verbose
```

`FAULT`가 발생하면 키를 놓고 원인을 해결한 뒤 **정지·제어권 해제 → 재정렬 → 재Arm**
순서로 다시 시작합니다. `ESTOP`은 원인 제거와 현장 전원 재인가 전까지 해제하지
않습니다.

## 11. 정상 종료

1. `Space`로 정지합니다.
2. **정지·제어권 해제**를 누릅니다.
3. 양쪽 로봇이 실제로 멈췄는지 확인합니다.
4. Rear launch, Front launch 순서로 `Ctrl+C`를 누릅니다.
5. 현장 절차에 따라 전원을 차단합니다.

브라우저만 닫는 것은 정상 종료가 아닙니다.
