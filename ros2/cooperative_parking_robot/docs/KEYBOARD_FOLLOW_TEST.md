# 강체 쌍 배치 확인 및 키보드 주행 가이드

이 문서는 robot-2 **Front**와 robot-1 **Rear**를 사람이 먼저 정렬한 뒤,
두 로봇을 하나의 가상 강체처럼 `W/A/S/D/Q/E`로 움직이는 현장 시험 절차입니다.

> **안전 원칙**
>
> - 첫 시험은 반드시 바퀴를 지면에서 띄운 상태로 수행합니다.
> - 물리 E-STOP을 즉시 누를 수 있는 사람이 로봇 옆에 있어야 합니다.
> - 웹 화면은 신뢰할 수 있는 격리 LAN에서만 엽니다.
> - `정렬 후보`는 카메라 기반 참고 표시일 뿐, 물리적 정렬이나 안전을 보증하지 않습니다.
> - 이 절차에서는 가짜 센서 토픽을 발행하지 않습니다. 실제 카메라와 실제 STM32
>   피드백을 사용합니다.

## 1. 기능의 동작 방식

`rigid_pair_teleop`은 한 로봇이 먼저 움직이고 다른 로봇이 따라가는 방식이 아닙니다.
키 입력을 두 로봇 **중점의 가상 강체 속도**로 해석하고, 같은 제어 주기와 같은 ROS
timestamp로 Front와 Rear의 명령을 계산해 발행합니다.

```text
                      Front (robot-2)
                 [ ID0 ArUco marker ]
                           ▲
                           │ Rear 카메라가 ID0를 관측
                           │ raw forward 약 0.215 m
                           ▼
                    [ Rear camera ]
                       Rear (robot-1)

              설정된 로봇 중심 간격: 0.785 m
```

Rear 카메라의 ID0 상대 pose는 다음 용도로 사용합니다.

- 배치 전: Front가 Rear에 대해 얼마나 앞뒤·좌우·yaw로 어긋났는지 표시
- Arm 시점: 현재 상대 pose를 주행 중 유지할 기준으로 저장
- 주행 중: 기준 대비 오차를 Front/Rear 양쪽에 대칭으로 보정

회전의 lever arm은 카메라에서 마커까지의 raw 거리 약 `0.215 m`가 아니라,
설정된 로봇 중심 간 거리 `0.785 m`입니다.

## 2. 현재 배치 기준

현재 설정은 다음 실측값을 사용합니다.

```text
추정 중심 종방향 간격
  = ID0 raw forward + aruco_distance_offset_m
  = 약 0.215 m + 0.570 m
  = 약 0.785 m
```

`aruco_distance_offset_m`는
[`config/id0_calibration.yaml`](../config/id0_calibration.yaml)에서 읽습니다.

### 2.1 `정렬 후보` 표시 기준

| 항목 | 목표 | 허용오차 | 화면 의미 |
| --- | ---: | ---: | --- |
| 추정 중심 종방향 간격 | `78.5 cm` | `±1.5 cm` | `raw forward + 57.0 cm` |
| raw lateral | `0 cm` | `±1.5 cm` | Front ID0의 Rear 기준 좌우 위치 |
| raw yaw | `0°` | `±2°` | Front와 Rear의 상대 회전 |

세 항목이 모두 범위 안이고 ID0 pose 3개가 안정적이면 `정렬 후보`가 표시됩니다.

### 2.2 `정렬 후보`와 Arm 조건의 차이

두 조건은 의도적으로 분리되어 있습니다.

| 구분 | 목적 | 주요 기준 |
| --- | --- | --- |
| `정렬 후보` | 사람이 정확히 배치하도록 돕는 표시 | 중심 `±1.5 cm`, lateral `±1.5 cm`, yaw `±2°` |
| Arm 조건 | 센서와 하드웨어가 최소한 안전하게 제어 가능한지 확인 | raw forward `10~100 cm`, lateral `≤10 cm`, yaw `≤15°`, ID0 3개 안정화 |

Arm 조건이 더 느슨하므로 `정렬 후보`가 아니어도 Arm 자체는 성공할 수 있습니다.
또한 Arm은 **누른 순간의 현재 ArUco pose를 유지 기준으로 저장**합니다. 따라서 배치가
틀어진 상태에서 Arm하면 그 틀어진 상태를 주행 중 유지할 수 있습니다. 반드시 먼저
배치를 끝내고 실제 간격과 평행 상태를 눈으로 확인하십시오.

## 3. 필요한 장비와 준비 사항

### 3.1 장비

- robot-2 Front와 Front STM32
- robot-1 Rear와 Rear STM32
- Rear의 보정된 ID0 카메라
- Front에 부착된 ID0 ArUco marker
- 동일한 ROS 2 네트워크에 연결된 두 호스트
- 웹 화면을 열 노트북 또는 태블릿
- 작동이 확인된 물리 E-STOP

### 3.2 시작 전 체크리스트

- [ ] 두 로봇의 바퀴가 처음에는 지면에서 떠 있다.
- [ ] Rear 카메라에서 Front ID0를 가리는 물체가 없다.
- [ ] Front가 Rear의 전방에 있고 두 로봇이 대략 평행하다.
- [ ] 두 호스트의 `ROS_DOMAIN_ID`가 같다.
- [ ] 두 호스트의 시간이 동기화되어 있다.
- [ ] Front/Rear STM32의 안정적인 `/dev/serial/by-id/...` 경로를 알고 있다.
- [ ] Rear 카메라의 `/dev/v4l/by-path/...-video-index0` 경로를 알고 있다.
- [ ] 카메라 calibration `.npz` 파일이 존재한다.
- [ ] 다른 주행 launch와 `/front|rear/cmd_vel` 발행자를 모두 종료했다.
- [ ] 물리 E-STOP이 손에 닿고 해제/전원 재인가 절차를 알고 있다.

장치 경로는 각 장치가 연결된 호스트에서 확인합니다.

```bash
ls -l /dev/serial/by-id/
ls -l /dev/v4l/by-path/
test -f "$HOME/ov2710_calib_23mm_white.npz" && echo "camera calibration OK"
```

`/dev/ttyACM0`이나 `/dev/video0`처럼 순서가 바뀔 수 있는 이름보다 위의 안정 경로를
사용하는 것을 권장합니다.

### 3.3 2026-08-28 현장 네트워크·장치 실측

아래 값은 운영 PC와 두 로봇에 직접 접속해 확인한 스냅샷입니다. DHCP 주소는 바뀔
수 있으므로 실행 당일에도 `*.local` 이름과 확인 명령을 우선 사용합니다.

| 역할 | hostname / SSH | 확인된 IP | Wi-Fi·경로 |
| --- | --- | --- | --- |
| 운영 PC | `robot-desktop` | `10.48.99.21/24` | SSID `minseo` |
| Rear | `robot@robot-1.local` | `10.48.99.229/24` | `wlan0`, workspace `/home/robot/cooperative_parking_robot_ws` |
| Front | `robot@robot-2.local` | `10.48.99.228/24` | `wlan0`, SSID `minseo`, 같은 workspace 경로 |

세 장치 모두 `10.48.99.0/24`, 기본 gateway `10.48.99.174`를 사용했고 운영 PC↔각
로봇 및 robot-1↔robot-2 ping은 손실 없이 성공했습니다. robot-1 이미지에는 SSID를
직접 조회할 `iw`가 없었지만 동일 wlan subnet/gateway, mDNS, 양방향 통신은
확인했습니다.

현장 설정 파일 기준 ROS domain과 실측 장치는 다음과 같습니다.

```text
ROS_DOMAIN_ID=42
Front serial=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0667FF485270535067112920-if02
Rear serial=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066AFF485270535067112511-if02
Rear camera=/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0
Rear calibration=/home/robot/ov2710_calib_23mm_white.npz
wheel_radius=0.05 m, encoder_ppr=5182, lx=0.2225 m, ly=0.21 m
```

> **현재 배포 상태:** 확인 당시 두 로봇의 overlay에는 구형 `keyboard_follow`만 있고
> 새 `rigid_pair_teleop` 및 최신 main의 MVP STM32 wrapper가 없었습니다. 아래 launch를
> 실행하기 전에 반드시 최신 검증 commit을 두 workspace에 배포·빌드하고 다음 확인을
> 통과해야 합니다.

```bash
source /opt/ros/humble/setup.bash
source /home/robot/cooperative_parking_robot_ws/install/setup.bash

ros2 pkg executables cooperative_parking_robot | \
  grep -E 'rigid_pair_teleop|camera_preview|stm32_bridge'
ros2 launch cooperative_parking_robot \
  cooperative_drive_test_rear.launch.py --show-args | \
  grep enable_rigid_pair_teleop
```

## 4. 코드 빌드와 자동 검증

코드를 새로 받은 뒤에는 먼저 개발 PC에서 빌드와 자동 테스트를 실행합니다.
자동 테스트는 격리된 ROS domain에서 가짜 입력을 사용하며 실제 STM32를 구동하지
않습니다.

```bash
cd /home/guitest/parkingbot_rigid_pair_latest
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select cooperative_parking_robot

cd /home/guitest/parkingbot_rigid_pair_latest/ros2/cooperative_parking_robot
scripts/run_feature_tests.sh rigid-pair
```

전체 기능을 함께 확인하려면 다음을 사용합니다.

```bash
cd /home/guitest/parkingbot_rigid_pair_latest/ros2/cooperative_parking_robot
scripts/run_feature_tests.sh all
```

현재 기능 브랜치는 위 별도 clean worktree에서 검증합니다. 병합 후 경로가 바뀌면
실제 checkout 경로를 사용하십시오. 로봇에 소스가 동기화된 뒤에는 **각 로봇에서**
다음처럼 빌드하고 overlay를 source합니다.

```bash
cd /home/robot/cooperative_parking_robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cooperative_parking_robot
source /home/robot/cooperative_parking_robot_ws/install/setup.bash
```

## 5. 두 호스트의 ROS 환경 준비

운영 PC에서 두 SSH 세션을 별도로 엽니다.

```bash
ssh robot@robot-2.local  # Front
ssh robot@robot-1.local  # Rear
```

Front와 Rear 호스트 **각각의 터미널**에서 현장 설정값 `42`를 사용합니다.

```bash
source /opt/ros/humble/setup.bash
source /home/robot/cooperative_parking_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=42

chronyc tracking
chronyc sources -v
```

권장 확인 기준은 다음과 같습니다.

- `chronyc tracking`의 절대 system-time offset이 `0.020 s` 미만
- `chronyc sources -v`에 선택된 source인 `^*`가 존재

시간 차이가 크면 source timestamp 검사가 입력을 오래됐거나 미래 값으로 판단할 수
있으므로 launch보다 시간 동기화를 먼저 해결합니다.

## 6. launch 실행

### 6.1 Front 호스트

Front 호스트에서 다음을 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source /home/robot/cooperative_parking_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch cooperative_parking_robot cooperative_drive_test_front.launch.py \
  serial_port:=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0667FF485270535067112920-if02 \
  wheel_radius:=0.05 encoder_ppr:=5182.0 \
  lx:=0.2225 ly:=0.21
```

이 launch는 Front STM32 bridge만 실행합니다. 자동 주행 노드나 별도의 `cmd_vel`
발행자는 시작하지 않습니다.

### 6.2 Rear 호스트

Rear 호스트에서 실제 장치 경로로 바꿔 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source /home/robot/cooperative_parking_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch cooperative_parking_robot cooperative_drive_test_rear.launch.py \
  serial_port:=/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066AFF485270535067112511-if02 \
  camera_device:=/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0 \
  camera_calib:=/home/robot/ov2710_calib_23mm_white.npz \
  wheel_radius:=0.05 encoder_ppr:=5182.0 \
  lx:=0.2225 ly:=0.21 \
  enable_drive_test_dashboard:=false \
  enable_rigid_pair_teleop:=true
```

`enable_drive_test_dashboard:=false`가 중요합니다. 기존 직선 주행 dashboard와 강체
키보드 제어기가 동시에 수동 명령을 소유하지 않도록 하는 설정입니다.

선택적으로 상단 CCTV 융합 결과까지 필수 조건으로 묶을 때만 아래 인자를 추가합니다.

```bash
  require_fused_odom:=true \
  require_cctv_marker:=true
```

기본 강체 키보드 시험에는 YOLO, Fleet, CCTV 융합이 필요하지 않습니다. 해당 토픽을
실제로 함께 실행하지 않으면서 위 옵션만 `true`로 설정하면 Arm이 차단됩니다.

## 7. 웹 화면 열기

Rear와 같은 신뢰된 LAN에 연결된 장치에서 다음 주소를 엽니다.

| 주소 | 용도 |
| --- | --- |
| `http://robot-1.local:5005/` | Rear 카메라 영상과 ID0 raw pose 확인 |
| `http://robot-1.local:5007/` | 배치 후보, 안전 blocker, Arm 및 키보드 제어 |

`robot-1.local`이 열리지 않으면 Rear 호스트의 IP를 사용합니다.

```text
http://<REAR_IP>:5005/
http://<REAR_IP>:5007/
```

두 웹 서버는 launch에서 LAN에 공개되므로 공용 Wi-Fi나 인터넷에 직접 노출하지
마십시오. 제어 POST는 같은 Origin만 허용하지만, 격리된 신뢰 LAN이 기본 전제입니다.

## 8. `5005` 카메라 화면 확인

먼저 `5005` 화면에서 다음을 확인합니다.

1. Rear 카메라 영상이 끊기지 않고 갱신된다.
2. Front의 ID0 marker 전체가 영상 안에 들어온다.
3. 상단 badge가 초록색이고 `ID0 raw 카메라→마커`가 표시된다.
4. forward, 좌우, 틀어짐 값이 크게 튀지 않는다.

초록색 badge는 상대 pose가 fresh하고 `/sync/marker_visible`이 명시적으로
`true`일 때만 표시됩니다. marker visibility가 오래됐거나 아직 수신되지 않은 경우에는
직전 수치가 있더라도 초록색으로 표시하지 않습니다.

카메라가 ID0를 안정적으로 못 보면 다음을 먼저 조정합니다.

- 마커를 가리는 케이블이나 차체 제거
- 강한 반사광과 역광 감소
- ID0 전체 사각형이 프레임 안에 들어오도록 대략적인 위치 조정
- 카메라 장치 경로와 calibration 파일 확인

## 9. `5007` 배치 안내 사용법

`5007`에도 카메라 영상이 표시되며, 오른쪽의 **배치 안내 (카메라 후보)**에서
보정된 중심 간격과 signed raw 값을 함께 확인할 수 있습니다.

### 9.1 상태별 의미

| 상태 | 의미 | 조치 |
| --- | --- | --- |
| `마커 찾기` | ID0가 없거나 pose/visibility가 오래됨 | `5005` 영상과 ID0 가림 여부 확인 |
| `안정화 중` | fresh pose가 있지만 최근 3개가 아직 안정적이지 않음 | 로봇을 움직이지 말고 잠시 기다림 |
| `보정값 없음` | forward offset YAML이 없거나 유효하지 않음 | calibration YAML과 launch 주입 확인 |
| `앞뒤 조정` | 추정 중심 종방향 간격이 허용범위 밖 | 전원을 안전하게 차단한 상태에서 앞뒤 간격 조정 |
| `좌우 조정` | 종방향은 맞지만 raw lateral이 허용범위 밖 | 좌우 위치 조정 |
| `yaw 조정` | 종·횡 위치는 맞지만 상대 yaw가 허용범위 밖 | 위에서 보며 두 로봇을 평행하게 조정 |
| `정렬 후보` | 세 표시값이 설정된 허용오차 안에 있음 | 실제 간격·평행·간섭을 눈으로 재확인 |

상태는 `앞뒤 → 좌우 → yaw` 순서로 가장 먼저 실패한 항목 하나를 표시합니다.
상태 문구만 보지 말고 아래의 세 signed 수치를 함께 확인하십시오.

### 9.2 signed 값 해석

- `종방향 기준 오차`가 `+`이면 추정 중심 간격이 `78.5 cm`보다 큽니다.
- `종방향 기준 오차`가 `-`이면 추정 중심 간격이 `78.5 cm`보다 작습니다.
- `raw lateral +`는 Front ID0가 Rear 기준 왼쪽에 있다는 뜻입니다.
- `raw yaw +`는 위에서 볼 때 Front가 Rear보다 반시계 방향으로 돌아가 있다는 뜻입니다.

lateral/yaw의 부호는 **측정 결과**이며 특정 로봇을 그 방향으로 움직이라는 지시가
아닙니다. Front 또는 Rear 중 어느 쪽을 손으로 조정할지는 실제 공간과 카메라 영상을
함께 보고 결정합니다. 목표는 두 값 모두 `0`입니다.

### 9.3 권장 수동 배치 순서

1. 모터가 움직이지 않도록 정지 상태를 확인합니다.
2. Front를 Rear 전방에 놓고 두 로봇을 대략 평행하게 맞춥니다.
3. `5005`에서 ID0가 계속 보이도록 합니다.
4. `5007`의 추정 중심 간격을 `78.5 cm` 근처로 맞춥니다.
5. raw lateral을 `0 cm` 근처로 맞춥니다.
6. 위에서 보고 두 로봇을 평행하게 만든 뒤 raw yaw를 `0°` 근처로 맞춥니다.
7. 로봇에서 손을 떼고 ID0 pose 3개가 안정화될 때까지 기다립니다.
8. `정렬 후보`가 나오는지 확인합니다.
9. 줄자 또는 실제 차체 기준으로 중심 간격, 차체 간섭, 케이블 여유를 다시 확인합니다.

현재 설정 기준으로 정상 배치 근처에서는 대략 다음 값이 보입니다.

```text
ID0 raw forward                 약 21.5 cm
forward offset                  57.0 cm
추정 중심 종방향 간격          약 78.5 cm
종방향 기준 오차               약 0.0 cm
raw lateral                    약 0.0 cm
raw yaw                        약 0.0°
```

raw forward `21.5 cm`는 현재 하드웨어 실측에 기반한 예상값입니다. marker나 카메라
장착 위치가 바뀌었다면 이 숫자를 억지로 맞추지 말고 먼저 calibration을 다시 측정합니다.

## 10. Arm 절차

배치를 완료한 뒤 다음 순서로 진행합니다.

1. 물리 E-STOP 담당자가 준비됐는지 확인합니다.
2. 아직 첫 시험이라면 바퀴가 지면에서 떠 있는지 다시 확인합니다.
3. `5007`에서 blocker 목록이 비어 있는지 확인합니다.
4. **현재 자세 기준 준비 (후보 무관)** 버튼을 한 번 누릅니다.
5. 상태가 `양쪽 제어권 확인 중`으로 바뀌는 동안 키를 누르지 않습니다.
6. 상태가 `강체 쌍 제어 준비 완료`가 된 뒤에만 이동 키를 사용합니다.

Arm 과정에서 확인하는 실제 입력은 다음과 같습니다.

- Rear ID0 `/sync/relative_pose`, `/sync/marker_visible`
- Front/Rear `/wheel_odom`
- Front/Rear `hardware_ready`
- Front/Rear `manual_active`
- `/emergency_stop`
- 다른 `/front|rear/cmd_vel` 및 manual command 발행자 존재 여부
- 설정했다면 fused odom과 CCTV marker freshness

Arm 대기 시간은 최대 10초입니다. 그 안에 조건이 충족되지 않으면 `FAULT`로
전환됩니다. blocker의 첫 문장을 해결한 뒤 **정지·제어권 해제**로 `IDLE`에 돌아가
다시 Arm합니다.

## 11. 키보드 조작

브라우저 페이지를 한 번 클릭한 다음 키를 사용합니다.

| 키/버튼 | 가상 강체 중점 동작 |
| --- | --- |
| `W` | 전진 |
| `S` | 후진 |
| `A` | 왼쪽 횡이동 |
| `D` | 오른쪽 횡이동 |
| `Q` | 중점 기준 반시계 회전 |
| `E` | 중점 기준 시계 회전 |
| `Space` 또는 화면의 `■` | 즉시 0속도, Arm 상태는 유지 |
| **정지·제어권 해제** | 0속도 발행 후 `IDLE`, 저장 기준 제거 |
| **양쪽 비상정지** | 양쪽 STM32에 고정 E-STOP 발행 |

기본 입력 속도는 다음과 같습니다.

- 선형: `0.0628 m/s`
- 각속도: `0.12 rad/s`
- 절대 제한: 선형 `0.08 m/s`, 각속도 `0.20 rad/s`

키를 누르는 동안 0.1초 간격으로 입력이 갱신됩니다. 마지막 입력이 0.30초 동안
갱신되지 않으면 ROS deadman이 양쪽에 0속도를 발행합니다. 브라우저 탭을 벗어나거나
창이 focus를 잃어도 정지 입력을 시도합니다.

### 11.1 바퀴를 띄운 첫 기능 확인

아래 순서에서는 각 키를 매우 짧게 누르고 매 단계 `Space`로 정지합니다.

1. `W`: 양쪽이 같은 전진 방향으로 반응하는지 확인
2. `S`: 양쪽이 같은 후진 방향으로 반응하는지 확인
3. `A`, `D`: 횡이동 방향과 wheel 반응 확인
4. `Q`, `E`: Front/Rear가 두 로봇 중점을 기준으로 회전하도록 서로 다른 횡속도를
   받는지 확인
5. 키를 놓았을 때 0.30초 이내에 정지하는지 확인

어느 단계든 방향이 예상과 다르거나 진동·급격한 보정이 보이면 즉시 물리 E-STOP을
누르고 배선, role, wheel 방향, ArUco pose 부호를 다시 확인합니다.

### 11.2 지면에서의 첫 시험

바퀴를 띄운 시험을 모두 통과한 뒤에만 지면 시험을 수행합니다.

1. 주변 사람과 장애물을 제거하고 낮은 마찰 변화가 없는 평면을 사용합니다.
2. 다시 수동 배치하고 `정렬 후보`와 실제 간격을 확인합니다.
3. Arm 후 `W`를 짧게 눌러 수 cm 이내만 움직입니다.
4. `Space`로 정지하고 중심 간격·lateral·yaw 변화를 확인합니다.
5. `S`, `A`, `D`를 각각 짧게 확인합니다.
6. `Q/E`는 마지막에 최소 입력으로 확인합니다.

한 세션에서 어느 로봇이든 누적 wheel-odom 경로가 `30 cm`에 도달하면 `LIMIT`로
정지합니다. 더 움직이려면 실제 위치를 다시 확인하고 제어권을 해제한 뒤 재배치·재Arm
하십시오.

## 12. 자동 안전 정지 조건

주행 중 다음 중 하나라도 발생하면 양쪽 명령을 0으로 만들고 `FAULT`, `LIMIT` 또는
`ESTOP` 상태로 전환합니다.

| 조건 | 기본값 |
| --- | ---: |
| 키 입력 deadman | `0.30 s` |
| ID0 pose/visibility timeout | `0.35 s` |
| wheel odom timeout | `0.50 s` |
| hardware/manual ACK timeout | `0.60 s` |
| 기준 대비 forward 변화 | `3 cm` 초과 시 정지 |
| 기준 대비 lateral 변화 | `3 cm` 초과 시 정지 |
| 기준 대비 yaw 변화 | `5°` 초과 시 정지 |
| 한 세션 누적 이동거리 | `30 cm` 도달 시 정지 |
| odom 한 샘플 위치 점프 | `10 cm` 초과 시 정지 |
| 다른 command 발행자 | 발견 시 Arm 차단 또는 주행 정지 |

`5007` 상태 요청이 0.8초 이상 멈추거나 마지막 정상 상태를 1초 이상 받지 못하면
화면은 이전 `정렬 후보`, ARMED, 측정값을 지우고 `연결 끊김`을 표시합니다. 이 표시는
물리 E-STOP 자체는 아닙니다. **연결 끊김이 보이면 즉시 키를 놓고 물리 E-STOP을
우선 사용**하십시오.

## 13. 화면 상태와 복구 방법

| 상태 | 의미 | 복구 |
| --- | --- | --- |
| `정지 · 제어권 없음` | 정상 IDLE | 배치와 blocker 확인 후 Arm |
| `양쪽 제어권 확인 중` | ARMING | 키를 누르지 말고 blocker 해소 대기 |
| `강체 쌍 제어 준비 완료` | ARMED | 짧은 키 입력 가능 |
| `안전 조건 위반 · 정지 유지` | FAULT | 원인 해결 → 제어권 해제 → 재Arm |
| `세션 거리 제한 · 정지 유지` | LIMIT | 실제 위치 확인 → 제어권 해제 → 재배치·재Arm |
| `비상정지 고정` | ESTOP | 키 사용 금지, 원인 제거 후 STM32 전원 재인가 |

`Space`는 ARMED 상태를 유지한 채 속도만 0으로 만듭니다. 기준 pose까지 버리려면
**정지·제어권 해제** 버튼을 사용합니다.

## 14. 문제 해결

### 14.1 웹 페이지가 열리지 않음

1. Rear launch가 종료되지 않았는지 확인합니다.
2. `robot-1.local` 대신 Rear IP를 사용합니다.
3. 운영 PC와 Rear가 같은 LAN인지 확인합니다.
4. `5005`, `5007` 포트가 방화벽에 차단되지 않았는지 확인합니다.

```bash
hostname -I
curl --max-time 2 http://127.0.0.1:5005/
curl --max-time 2 http://127.0.0.1:5007/
```

### 14.2 `5005` 영상은 보이지만 `마커 찾기`가 계속됨

- ID0 전체가 영상에 들어오는지 확인합니다.
- marker ID가 실제로 `0`인지 확인합니다.
- marker 크기 설정과 실물 크기가 일치하는지 확인합니다.
- `/sync/relative_pose` frame이 `rear_base`인지 확인합니다.
- 두 관련 토픽이 계속 수신되는지 확인합니다.

```bash
ros2 topic echo --once /sync/relative_pose
ros2 topic echo --once /sync/marker_visible
ros2 topic hz /sync/relative_pose
```

`5005`는 화면 표시 timeout이 1초이고 `5007` 제어기는 ID0에 더 엄격한 0.35초를
사용합니다. 따라서 불안정한 저주기 입력에서는 `5005`가 잠깐 정상처럼 보여도
`5007`이 `마커 찾기`를 표시할 수 있습니다.

### 14.3 `안정화 중`에서 바뀌지 않음

- 두 로봇과 카메라를 손으로 잡고 있지 말고 완전히 정지시킵니다.
- ID0가 영상 가장자리나 반사광 위에 있지 않은지 확인합니다.
- 최근 3개 pose의 forward span `1 cm`, lateral span `1 cm`, yaw span `2°` 안에
  들어와야 합니다.

### 14.4 `보정값 없음`

설치된 calibration 파일과 실제 값을 확인합니다.

```bash
ros2 pkg prefix cooperative_parking_robot
sed -n '1,20p' \
  "$(ros2 pkg prefix cooperative_parking_robot)/share/cooperative_parking_robot/config/id0_calibration.yaml"
```

현재 정상값은 `aruco_distance_offset_m: 0.570`입니다. 카메라/마커 장착을 바꾼
경우에는 이전 값을 복사하지 말고 다시 실측합니다.

### 14.5 `hardware_ready가 없거나 오래됨`

- 해당 STM32 serial 경로가 맞는지 확인합니다.
- STM32 전원과 baud rate를 확인합니다.
- 기본 launch는 ultrasonic frame까지 ready 조건에 포함합니다.
- Front와 Rear의 role/hardware profile이 뒤바뀌지 않았는지 확인합니다.

```bash
ros2 topic hz /front/wheel_odom
ros2 topic hz /rear/wheel_odom
ros2 topic echo --once /front/hardware_ready
ros2 topic echo --once /rear/hardware_ready
```

### 14.6 `수동 제어권 확인 안 됨`

Arm 요청 후 두 STM32 bridge가 manual enable에 응답해야 합니다. 다른 수동 제어
프로그램을 종료하고 Front/Rear bridge 로그를 확인합니다.

### 14.7 `다른 주행 발행자 존재`

다음 명령으로 발행자를 확인하고, 이번 시험에 속하지 않는 launch를 종료합니다.

```bash
ros2 topic info /front/cmd_vel --verbose
ros2 topic info /rear/cmd_vel --verbose
ros2 topic info /front/manual_cmd_vel --verbose
ros2 topic info /rear/manual_cmd_vel --verbose
```

자동 상태기계, individual move, rigid sync, 다른 teleop 또는 기존 drive dashboard를
동시에 실행하지 마십시오.

### 14.8 `FAULT`가 발생함

1. 새 키를 누르지 않습니다.
2. 화면의 decision과 blocker를 기록합니다.
3. 물리 상태, ID0 영상, odom, STM32 상태를 확인합니다.
4. 원인을 해결합니다.
5. **정지·제어권 해제**로 IDLE에 돌아갑니다.
6. 처음부터 배치와 Arm을 다시 수행합니다.

### 14.9 `ESTOP`이 발생함

E-STOP은 고정 상태입니다. 웹에서 바로 재Arm하지 마십시오.

1. 모든 키에서 손을 뗍니다.
2. 위험 원인을 제거합니다.
3. 로봇 위치와 배선을 확인합니다.
4. 현장 전원 절차에 따라 STM32 전원을 재인가합니다.
5. launch 로그와 ready 신호를 확인합니다.
6. 바퀴를 띄운 배치 단계부터 다시 시작합니다.

## 15. 정상 종료 절차

1. `Space` 또는 화면의 `■`로 0속도를 발행합니다.
2. **정지·제어권 해제**를 눌러 상태가 `정지 · 제어권 없음`인지 확인합니다.
3. 양쪽 로봇이 실제로 정지했는지 눈으로 확인합니다.
4. Rear launch를 `Ctrl+C`로 종료합니다.
5. Front launch를 `Ctrl+C`로 종료합니다.
6. 현장 절차에 따라 로봇 전원을 차단합니다.

브라우저 창만 닫는 것을 정상 종료 절차로 사용하지 마십시오.

## 16. 카메라 또는 marker 장착 변경 시

카메라, ID0 marker, 차체 중심 위치 중 하나라도 바뀌면 forward offset을 다시
측정해야 합니다.

```text
aruco_distance_offset_m
  = 실제 로봇 중심 간 종방향 거리 - 같은 순간의 ID0 raw forward
```

측정값은 Python 코드에 넣지 않고 다음 YAML만 변경합니다.

```yaml
/**:
  ros__parameters:
    aruco_distance_offset_m: 0.570  # 예시이며 재실측값으로 변경
```

필요하면 launch에서 다른 calibration 파일을 명시할 수 있습니다.

```bash
ros2 launch cooperative_parking_robot cooperative_drive_test_rear.launch.py \
  id0_calibration:=/absolute/path/to/id0_calibration.yaml \
  rigid_pair_separation_m:=0.785 \
  enable_drive_test_dashboard:=false \
  enable_rigid_pair_teleop:=true \
  ...
```

현재 배치 안내는 forward 방향에만 위 offset을 적용합니다. lateral/yaw의 카메라
extrinsic을 별도로 추정하지 않으므로, `정렬 후보` 이후에도 실제 차체의 평행 상태와
간격을 사람이 확인해야 합니다.

## 17. 현장용 빠른 체크리스트

```text
[ ] 바퀴를 띄움 / 물리 E-STOP 준비
[ ] 두 호스트 ROS_DOMAIN_ID와 시간 동기화 확인
[ ] Front launch 실행
[ ] Rear launch 실행 (기존 drive dashboard=false, rigid teleop=true)
[ ] 5005: 영상 정상, ID0 raw badge 초록색
[ ] 5007: 중심 78.5±1.5 cm, lateral 0±1.5 cm, yaw 0±2°
[ ] 정렬 후보 후 실제 간격·평행·간섭 재확인
[ ] 현재 자세 기준 준비 버튼 → 강체 쌍 제어 준비 완료 확인
[ ] W/S/A/D/Q/E를 각각 짧게, 매번 Space 정지
[ ] 이상 시 키를 놓고 물리 E-STOP
[ ] 종료 시 Space → 제어권 해제 → Rear/Front launch 종료 → 전원 차단
```
