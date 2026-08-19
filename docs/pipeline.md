# parkingbot v1.11 실차 배포 pipeline

> 대상 패키지: parkingbot_v1_11  
> ROS 2 원본: Adaptive_Valet_Bot_v1_11_UI_MissionReset_20260805.zip  
> STM32 원본: parking_robot_stm32cubeide_20260724.zip  
> STM32 통합본: parking_robot_stm32cubeide_v111_integrated_20260816

이 문서는 통합 ZIP을 실제 장비에 배포하는 기준 설명서다. ROS 패키지 안의 과거
문서에 CubeMX 프로젝트 미완성 또는 PB12~PB15 초음파 fallback이라는 설명이 남아
있더라도, 이 통합본에서는 stm32/parking_robot 프로젝트와 이 문서를 우선한다.

현재 판정은 **감독하의 저속 단계시험 가능, 무인 차량 인양은 NO-GO**다.
Python 논리·계약 시험 162개와 STM32 HAL/CMSIS GNU11 구문 검사는 통과했지만,
ARM 링크·HEX 생성·보드 플래시·모터 및 하중 시험은 실제 장비에서 해야 한다.

## 1. 패키지 구조와 기준 파일

~~~text
parkingbot_v1_11/
├─ ros2/cooperative_parking_robot/   ROS 2 Humble 패키지
├─ stm32/parking_robot/              플래시 대상 STM32CubeIDE 프로젝트
└─ docs/
   ├─ REAL_WORLD_READINESS.md        실차 준비도와 NO-GO 조건
   └─ pipeline.md                    이 배포 설명서
~~~

- 실제 STM32 빌드는 stm32/parking_robot만 사용한다.
- ros2/.../stm32_firmware는 코드 검토용 동기 사본이다.
- 두 STM32에는 같은 펌웨어를 플래시하고 Front/Rear 역할은 ROS namespace와
  서로 다른 serial port로 구분한다.

## 2. 기준 하드웨어·소프트웨어

| 장비 | 기준 환경 | 역할 |
|---|---|---|
| Jetson Orin Nano | JetPack 6.x, Ubuntu 22.04, ROS 2 Humble | CCTV, YOLO/BEV, Fleet, UI |
| Front Raspberry Pi 4 | Ubuntu 22.04 arm64, ROS 2 Humble | 강체 Master, STM32 bridge, pose fusion |
| Rear Raspberry Pi 4 | Ubuntu 22.04 arm64, ROS 2 Humble | ID0 ArUco, STM32 bridge, pose fusion |
| STM32F401RE ×2 | STM32CubeIDE, HAL | 메카넘 PID, 엔코더, 서보, 초음파, UART |

Jazzy에서 단위시험을 실행한 결과는 참고할 수 있지만 실차 배포 기준은 Humble과
Python 3.10이다. 세 Linux 장비는 같은 LAN, 같은 ROS_DOMAIN_ID, 동기화된 시계를
사용해야 한다.

## 3. 전체 데이터·제어 pipeline

~~~text
천장 CCTV 1대 또는 2대
  → 렌즈 왜곡 보정
  → rectified 영상
  ├→ YOLO vehicle mask → Homography → 차량·슬롯·occupancy map
  └→ 상판 ArUco ID10/ID11 → Front/Rear 전역 pose
          ↓
    단일 CCTV 출력 또는 cctv_merge_node
          ↓
    fleet_manager + UI 승인 게이트
          ↓ A* waypoint
    Front rigid_body_sync Master
          ↓
    /front/cmd_vel, /rear/cmd_vel
          ↓
    각 RPi stm32_bridge_node
          ↓ UART 115200 8N1
    각 STM32 모터 PID·서보·초음파

Rear 전방 카메라 → Front 후면 ID0 → 상대 yaw/거리
엔코더 + CCTV 절대 pose + ID0 상대 pose → PoseEKF
HC-SR04 → STM32 → U,L/R 프레임 → 축 에지·중심 정렬
~~~

정상 임무 순서는 다음과 같다.

~~~text
차량 인식 → UI 입차 승인 → staging → PRE_ALIGN
→ Front/Rear 동시 SCAN_IN → 초음파 축 중심 정렬
→ 양측 파지 완료 → 결합 footprint A*
→ 동기 운반 → 슬롯 축 정렬·직선 진입
→ 하차 → 분리 복귀 → /mission/complete → 다음 임무 reset
~~~

## 4. 절대 생략하지 않는 안전 원칙

1. 물리 ESTOP이 소프트웨어 ESTOP보다 우선한다.
2. 첫 시험은 모터 전원 차단 상태, 다음은 바퀴를 든 잭업 상태에서 한다.
3. hardware_ready가 false이면 모터 전원을 인가하지 않는다.
4. 실측값 대신 기본값을 사용한 채 차량을 들지 않는다.
5. GRIP_DONE은 서보 목표각 도달 신호이며 실제 파지·하중 확인 신호가 아니다.
6. 단계시험 하나가 실패하면 다음 단계로 넘어가지 않는다.
7. HC-SR04 ECHO 5V는 STM32에 직접 넣지 않고 분압 또는 레벨시프터를 쓴다.
8. 모터·서보·연산 전원은 분기하고 공통 GND, 퓨즈, 전원 차단기를 둔다.

## 5. 배포 전에 준비할 실측·외부 파일

| 항목 | 필수 내용 |
|---|---|
| YOLO | vehicle_seg.engine 권장, 또는 검증된 pt 모델과 정확한 model_mode |
| CCTV intrinsic | 카메라마다 calibration npz, 실제 해상도와 동일 |
| Homography | rectified 영상에서 생성한 카메라별 npy |
| 주차장 layout | layout_registered=true, metre 좌표, 슬롯·대기영역·no-go |
| Rear intrinsic | rear_camera_calibration.npz |
| ArUco | 실제 marker_size, yaw_offset, 선택적 distance offset |
| 차체 | wheel_radius, encoder_ppr, lx, ly |
| 차량 | wheelbase, length, width |
| 초음파 | 좌우 sensor_to_gripper_x, threshold, hysteresis |
| 기구 | 좌우 servo open/grip 각도, 방향, 간섭 한계 |

동봉 기본값 wheelbase 0.70m, vehicle length 0.90m, width 0.35m,
wheel radius 0.05m, encoder PPR 2600, lx/ly 0.10m는 실측 전 명목값이다.
ROS launch 값과 STM32 상수는 동일해야 한다.

## 6. STM32 빌드·플래시

### 6-1. 확정 핀 배치

| 기능 | 핀·타이머 | 펌웨어 역할 |
|---|---|---|
| 모터 PWM 1~4 | PA8~PA11, TIM1 CH1~4 | FL, FR, RL, RR |
| 모터 DIR 1~4 | PC0~PC3 | FL, FR, RL, RR |
| FL encoder | PA15/PB3, TIM2 | 32-bit |
| FR encoder | PB4/PB5, TIM3 | 16-bit |
| RL encoder | PB6/PB7, TIM4 | 16-bit |
| RR encoder | PA0/PA1, TIM5 | 32-bit |
| Left servo | PB8, TIM10 CH1 | 파지 왼쪽 |
| Right servo | PB9, TIM11 CH1 | 파지 오른쪽 |
| UART | PA2 TX, PA3 RX, USART2 | ROS 2 bridge |
| Left ultrasonic | PC8 TRIG, PC6 ECHO | ULTRASONIC1 |
| Right ultrasonic | PC5 TRIG, PC7 ECHO | ULTRASONIC2 |
| 초음파 timebase | TIM9, 1MHz, ARR 65535 | pulse width |

실제 배선도 반드시 Motor1/2/3/4 = FL/FR/RL/RR, Ultrasonic1/2 =
Left/Right 순서여야 한다. SWD는 PA13/PA14를 유지한다.

### 6-2. 모터 PWM 스케일

PID 출력 0~999는 TIM1의 현재 ARR 전체 범위로 자동 변환된다.

~~~text
CCR = round(abs(PID_output) / 999 × TIM1_ARR)
~~~

현재 TIM1은 84MHz, PSC 0, ARR 65535이므로 약 1.282kHz다. 대표 환산값은
0→0%, 100→10%, 500→50.1%, 999→약 100%다. ARR를 바꿔도 duty 비율은
유지된다. 모터 드라이버가 20kHz를 요구하면 PSC 0, ARR 4199가 계산상 맞지만
드라이버 데이터시트·소음·발열 시험 없이 바꾸지 않는다.

### 6-3. CubeIDE 절차

1. STM32CubeIDE에서 Existing Projects into Workspace로
   stm32/parking_robot을 import한다.
2. parking_robot.ioc를 열고 타이머·핀 충돌 경고가 없는지 확인한다.
3. Project → Clean 후 Debug 빌드한다.
4. parking_robot_firmware.c가 build console에 포함됐는지 확인한다.
5. warning/error 0건을 확인하고 각 보드에 플래시한다.
6. reset 후 모터 전원 없이 UART 프레임부터 확인한다.

main.c는 주변장치 초기화 뒤 Robot_Init을 한 번 호출하고 while 루프에서
Robot_MainLoop를 계속 호출하도록 통합돼 있다.

### 6-4. UART 물리 연결

- ST-LINK VCP 사용 시 Linux 장치는 보통 /dev/ttyACM*다.
- USB-UART 변환기 사용 시 보통 /dev/ttyUSB*다.
- RPi GPIO 직결은 RPi TX→PA3, RPi RX←PA2, 공통 GND로 교차 연결한다.
- ST-LINK VCP와 외부 TX가 같은 USART2 RX 선을 동시에 구동하지 않게 한다.
- UART와 HC-SR04는 배선 경로를 모터 전원선에서 떨어뜨린다.

## 7. ROS 2 설치

각 Jetson/RPi에서 다음 구조로 배치한다.

~~~bash
mkdir -p ~/parkingbot_ws/src
cp -a <압축해제경로>/parkingbot_v1_11/ros2/cooperative_parking_robot \
  ~/parkingbot_ws/src/
source /opt/ros/humble/setup.bash
cd ~/parkingbot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select cooperative_parking_robot
source install/setup.bash
colcon test --packages-select cooperative_parking_robot
colcon test-result --verbose
~~~

모든 터미널 또는 systemd 서비스에 다음을 동일하게 적용한다.

~~~bash
source /opt/ros/humble/setup.bash
source ~/parkingbot_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
~~~

세 장비에 chrony를 설치하고 chronyc tracking의 offset이 안정적인지 확인한다.
방화벽·AP client isolation·멀티캐스트 차단이 없어야 한다.

## 8. serial port 고정

ttyUSB0/1 또는 ttyACM0/1 번호는 재부팅 시 바뀔 수 있다. 각 RPi에서 다음으로
안정적인 장치 경로를 찾는다.

~~~bash
ls -l /dev/serial/by-id/
sudo usermod -aG dialout $USER
~~~

그룹 변경 후 로그아웃·로그인한다. Front와 Rear에 서로 다른 by-id 경로를
명시한다. full_system.launch.py는 smoke 안전을 위해 enable_serial=false가
기본이므로 실차에서 기본값 그대로 실행하면 STM32와 연결되지 않는다.

## 9. 카메라·BEV 등록

### 9-1. Rear ID0 카메라

실제 운용 해상도에서 체커보드 20장 이상을 중앙·모서리·기울임 자세로 수집한다.

~~~bash
ros2 run cooperative_parking_robot calibrate_camera --ros-args \
  -p image_topic:=/rear/marker_camera/image \
  -p output_path:=~/parkingbot_ws/src/cooperative_parking_robot/config/rear_camera_calibration.npz \
  -p board_cols:=9 -p board_rows:=6 -p square_size_m:=0.025
~~~

RMS가 1px 이상이면 저장하지 말고 다시 촬영한다. 기본 Rear 영상은
1280×720@12fps가 권장된다. 생성 후 다시 colcon build한다.

### 9-2. 단일 CCTV

렌즈 보정된 /cctv/image_rect를 사용해 Homography와 layout을 등록한다.
Raw 영상에서 만든 H와 섞지 않는다.

~~~bash
ros2 launch cooperative_parking_robot bev_layout_calibration.launch.py \
  enable_opencv_camera:=true
~~~

브라우저 포트 5001에서 바닥 기준점, 슬롯 모서리 4점, 통로점, 대기영역,
고정 장애물 no-go를 등록한다. H의 출력 단위가 metre이면
homography_scale_to_m=1.0이며 cm로 만든 과거 파일만 0.01이다.

### 9-3. 듀얼 CCTV

카메라마다 intrinsic npz와 rectified Homography가 하나씩 필요하다. 두 등록에서
같은 물리 바닥점에 같은 map X,Y 값을 입력하고 겹침 영역 공통점 2~3개 이상을
포함한다. 카메라마다 6~12점을 권장하며 RMS가 0.02m 이상이면 다시 등록한다.

1회차 cam0은 layout과 대기영역까지 저장한다. 2회차 cam2는
append_existing_layout=true로 기존 layout에 추가한다. 최종 실행은 다음과 같다.

~~~bash
ros2 launch cooperative_parking_robot cctv_server_dual.launch.py \
  enable_opencv_camera:=true \
  camera0_id:=0 camera2_id:=2 \
  cctv0_camera_calib:=<cam0.npz> \
  cctv2_camera_calib:=<cam2.npz> \
  homography_cam0_file:=<cam0.npy> \
  homography_cam2_file:=<cam2.npy> \
  layout_config:=<parking_layout.yaml> \
  model_path:=<vehicle_seg.engine> \
  enable_operator_ui:=true \
  enable_debug_overlay:=false
~~~

`enable_operator_ui=true`가 Fleet 승인용 kiosk/API를 실행한다. 진단용
YOLO/ArUco/FPS overlay가 필요한 경우에만 `enable_debug_overlay:=true`를 준다.

겹침 영역에 차량을 두고 /cctv/merge_status에서
multi_camera_detections≥1, duplicates_removed≥1인지 확인한다.
/parking/map publisher는 cctv_merge_node 하나만 있어야 한다.

## 10. 실차 preflight

Jetson:

~~~bash
hardware_preflight --role jetson \
  --cctv-camera-calib <실제-cctv.npz> \
  --model-path <vehicle_seg.engine> \
  --model-mode vehicle_seg \
  --homography-file <homography_rectified.npy>
~~~

Rear RPi:

~~~bash
hardware_preflight --role rear \
  --serial-port /dev/serial/by-id/<rear-stm32> \
  --rear-camera-calib <rear_camera_calibration.npz>
~~~

preflight 실패를 launch 인자로 우회하지 않는다. 모델, calibration, Homography,
serial permission을 먼저 고친다.

## 11. 분산 기동 순서

1. 작업구역을 비우고 차량을 지그에 올리며 모터 전원은 차단한다.
2. 두 STM32를 켜고 Jetson 인지·UI를 먼저 실행해 map과 target을 확인한다.
3. Rear RPi를 실행한다.
4. Front RPi Master를 실행한다.
5. 양측 hardware_ready, localization, CCTV merge를 확인한다.
6. 모든 항목이 정상일 때만 모터 전원을 인가한다.
7. UI 입차 버튼은 단계시험 승인 후 누른다.

Rear:

~~~bash
ros2 launch cooperative_parking_robot rear_robot.launch.py \
  serial_port:=/dev/serial/by-id/<rear-stm32> \
  enable_serial:=true require_serial:=true \
  require_hardware_ready:=true require_ultrasonic_for_ready:=true \
  camera_calib:=<rear_camera_calibration.npz> \
  wheelbase:=<실측> wheel_radius:=<실측> encoder_ppr:=<실측> \
  lx:=<실측> ly:=<실측> \
  left_sensor_to_gripper_x_m:=<실측> \
  right_sensor_to_gripper_x_m:=<실측>
~~~

Front:

~~~bash
ros2 launch cooperative_parking_robot front_robot.launch.py \
  serial_port:=/dev/serial/by-id/<front-stm32> \
  enable_serial:=true require_serial:=true \
  require_hardware_ready:=true require_ultrasonic_for_ready:=true \
  wheelbase:=<실측> wheel_radius:=<실측> encoder_ppr:=<실측> \
  lx:=<실측> ly:=<실측> \
  left_sensor_to_gripper_x_m:=<실측> \
  right_sensor_to_gripper_x_m:=<실측> \
  use_aruco_distance:=false
~~~

aruco_distance_offset_m을 실측한 뒤에만 use_aruco_distance=true로 바꾼다.
상대 yaw는 거리 보정을 끈 상태에서도 사용할 수 있다.

## 12. 연결·상태 합격 기준

~~~bash
ros2 topic echo /front/hardware_ready
ros2 topic echo /rear/hardware_ready
ros2 topic hz /front/wheel_odom
ros2 topic hz /rear/wheel_odom
ros2 topic hz /front/ultrasonic_left
ros2 topic hz /front/ultrasonic_right
ros2 topic echo /front/ultrasonic_status
ros2 topic echo /front/localization_status
ros2 topic echo /rear/localization_status
ros2 topic echo /cctv/merge_status
ros2 topic info /parking/map --verbose
~~~

| 항목 | 합격 기준 |
|---|---|
| UART | 10Hz heartbeat ACK, 파싱 오류 0 |
| hardware_ready | 양측 true 유지 |
| encoder | 약 50Hz, 정지 시 count 안정 |
| ultrasonic | 좌우 각각 12~16Hz, 0.5s 이내 갱신 |
| CCTV H | 각 카메라 RMS < 0.02m |
| localization | initialized=true, stale/gate 연속 오류 없음 |
| map | publisher 정확히 1개 |
| UI | 상태 fresh, fault 없음일 때만 입차 활성 |

UART를 분리하면 300ms 내 STM32 PWM이 0이 되고 0.5초 내 hardware_ready=false가
되어야 한다.

## 13. 단계별 실차 시험

| 단계 | 시험 | 다음 단계 조건 |
|---:|---|---|
| 0 | 정적 배선·전압·ESTOP | 단락·5V ECHO 직접입력 없음 |
| 1 | UART만 연결, 모터전원 OFF | ACK/E/U 프레임 안정 |
| 2 | 바퀴 1개 잭업 | 명령·회전·encoder 부호 일치 |
| 3 | 로봇 1대 전체 잭업 | 전후·횡·회전, timeout 정지 |
| 4 | 로봇 1대 바닥 저속 | 직진·횡이동 오차 기록 |
| 5 | 두 로봇 빈손 | 동일방향·회전 동기, 간격 유지 |
| 6 | 초음파 scan-in 모형 | PRE_ALIGN 후 축 중심 20회 재현 |
| 7 | 서보 무부하 | 좌우 방향·간섭·ESTOP hold |
| 8 | 보호지그 저하중 | 실제 파지 확인, 미끄럼 없음 |
| 9 | park→HOME→retrieve→HOME 1사이클 | Registry와 mission reset까지 정상 |

권장 초기 주행 속도는 0.03~0.05m/s다. 펌웨어 허용 상한 0.25m/s는 시험
시작 속도가 아니다. 바퀴 부호가 하나라도 틀리면 kMotorCommandSign과
kEncoderSign을 수정한 후 2단계부터 다시 한다.

## 14. 정상 운용과 mission reset

- 기본 require_ui_confirmation=true에서 차량 검출만으로 움직이지 않는다.
- kiosk 입차는 차량번호, 4~64자 주차 비밀번호와 EMPTY 목적 슬롯을 입력받고,
  target_ready, 빈 슬롯, 양측 IDLE, fresh 상태를 서버에서 다시 검사한 후
  /ui/mission_request를 발행한다.
- 각 로봇은 실제 home 도착 후 HOME ready를 발행한다. 양쪽 HOME ready 뒤 Front가
  HOME commit을 발행하고, 그 이후에만 /mission/complete를 발행한다.
- Fleet는 matching mission ID의 완료만 받아 active mission을 reset한다. park의
  OCCUPIED Registry record는 reset하지 않아 UI에서 출차 슬롯을 선택할 수 있다.
- 출차 UI는 차량번호와 입차 때 등록한 비밀번호만 제출한다. Fleet가 credential을
  검증해 source_slot_id를 자체 결정하고 Registry pose/spec과 고정 waiting_x/y/yaw를
  사용해 기존 접근·인양·A*·운반·하차·복귀 흐름을 재사용한다. source_slot_id만 보낸
  요청은 비밀번호 우회가 되므로 거부한다.
- Fleet Registry는 기본 `~/.ros/adaptive_valet_bot/parking_registry.db`에
  저장된다. `EMPTY/OCCUPIED` 안정 상태는 같은 layout에서 Fleet 재시작 뒤
  복원한다. `RESERVED/EXIT_RESERVED/EXITING`, 손상 DB 또는 layout 불일치는
  startup을 차단하며 자동 재개하지 않는다.

비밀번호 원문은 Registry, /fleet/state와 로그에 남기지 않는다. 다만 현재 HTTP와
ROS String transport 자체는 암호화되지 않으므로 웹 UI는 내부 신뢰망에서만 사용한다.
웹 UI가 죽어도 주행 판단은 Fleet가 소유하지만, 물리 ESTOP 없이 웹 버튼만
안전장치로 사용하면 안 된다.

## 15. ESTOP과 복구

소프트웨어 비상정지:

~~~bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}"
~~~

STM32 ESTOP은 latch된다. UI mission reset이나 ROS 노드 재시작으로 해제되지 않는다.

1. 물리 모터 전원을 차단한다.
2. 원인과 기구 걸림을 제거한다.
3. ROS cmd_vel이 0인지 확인한다.
4. STM32를 전원 재인가한다.
5. hardware_ready와 모든 센서 토픽을 다시 확인한다.
6. 해당 단계시험부터 재개한다.

## 16. 흔한 장애와 조치

| 증상 | 먼저 볼 것 | 조치 |
|---|---|---|
| STM32 연결 실패 | /dev/serial/by-id, dialout | 포트·권한·케이블 확인 |
| hardware_ready=false | ACK와 양쪽 U 프레임 | baud, GND, EXTI, 센서 전원 |
| 초음파 한쪽 TIMEOUT | PC6/PC7 ECHO | Left=1, Right=2 배선과 분압 확인 |
| 모터가 약함 | TIM1 CCR/ARR | 통합 소스 빌드 여부와 duty 확인 |
| 바퀴 방향 반대 | wheel별 sign | 잭업 후 command/encoder sign 수정 |
| PRE_ALIGN 무한대기 | ID0, CCTV pose, yaw | Rear 카메라 calibration과 marker 확인 |
| 축 중심이 일정하게 빗나감 | sensor_to_gripper_x | 좌우 offset 재실측 |
| 겹침 차량이 2개 | merge_status | 같은 기준점 H 재등록 또는 dedup gate |
| A* 시작점 막힘 | robot/target mask | layout no-go와 footprint 실측 확인 |
| 두 번째 임무가 시작 안 됨 | HOME commit, /mission/complete | 양쪽 return_done과 mission ID 확인 |

## 17. 현재 구조적 한계

- GRIP_DONE은 실제 하중을 확인하지 않는다. 무인 인양에는 limit/current/load
  센서와 독립적인 전원 차단이 필요하다.
- A*는 인양 직후 한 번 계획하며 주행 중 동적 재계획이 없다.
- 벽·기둥은 차량 YOLO가 검출하지 않으므로 no-go 영역 등록이 필요하다.
- 듀얼 CCTV 높이가 서로 다르면 단일 camera_height_m parallax 보정이 부정확하다.
- require_full_slot_coverage=false에서는 슬롯 중심 관측만으로 판정할 수 있다.
- Registry는 SQLite로 안정 상태를 복원하지만 transient mission crash recovery와
  Perception 기반 물리 reconciliation은 없다.
- 이번 출차는 이 Fleet 세션이 forward로 주차한 차량만 지원한다.
- park→retrieve 연속 사이클은 자동 테스트됐지만 실제 하중 실기 검증이 필요하다.

## 18. 최종 GO/NO-GO 체크리스트

- [ ] CubeIDE ARM 빌드와 두 보드 플래시 성공
- [ ] Front/Rear stable serial by-id 고정
- [ ] 양측 hardware_ready=true
- [ ] 물리 ESTOP과 모터 전원 차단 시험
- [ ] motor/encoder sign 4개 모두 확정
- [ ] wheel_radius, PPR, lx, ly 실측 및 ROS/STM32 일치
- [ ] Rear intrinsic과 ArUco marker/yaw 보정
- [ ] CCTV별 intrinsic, rectified Homography RMS < 2cm
- [ ] layout_registered=true와 no-go 영역 확인
- [ ] 초음파 12~16Hz, offset 및 중심오차 반복시험
- [ ] 빈손 두 로봇 동기주행 통과
- [ ] 보호지그 저하중 파지·해제 통과
- [ ] 실제 파지/하중 검증 수단 확보

마지막 항목까지 충족하지 못하면 사람 없는 무인 차량 인양은 NO-GO다.
