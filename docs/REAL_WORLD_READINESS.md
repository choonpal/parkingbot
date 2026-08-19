# Adaptive Valet Bot v1.11 실차 적용 준비도 검토

- 검토 대상: `feature/exit-mission-integration` 현재 소스
- 검토일: 2026-08-19
- 목적: Homography 등록 후 실제 협동 주차로봇 운용 가능 여부와 선행 조건 명확화

## 1. 결론

v1.11은 실차 운용을 목표로 한 소프트웨어 구조와 안전 정지 로직을 갖추고 있다. UI 승인, mission reset, CCTV/YOLO/ArUco 인식, PoseEKF, 동시 scan-in, 초음파 차축 정렬, 결합 footprint A*, 슬롯 축 정밀 진입까지 코드에 구현되어 있다.

그러나 **Homography 파일만 넣은 상태로 실제 차량을 들어 올려 무인 주차시키면 안 된다.**
완전한 STM32CubeIDE 프로젝트와 park/retrieve 소프트웨어 흐름은 저장소에
포함됐지만, 실기체별 보정값, ARM build/flash, 전기 안전과 하중 검증은
현장에서 완료해야 한다. 실제 탑재 순서는
`docs/REAL_ROBOT_DEPLOYMENT_RUNBOOK.md`를 우선 따른다.

현재 권장 판정은 다음과 같다.

| 단계 | 판정 |
|---|---|
| 소프트웨어 단위/계약 시험 | PASS — 246 tests |
| 카메라·BEV 정적 인식 시험 | 보정 후 가능 |
| 바퀴 공중시험 | 하드웨어 상수 입력 후 가능 |
| 빈 차체 저속 단독주행 | 단계 시험 후 가능 |
| 두 로봇 저속 동기주행 | 상대 pose 검증 후 가능 |
| 모형차 저하중 인양·주차 | 모든 인터락 검증 후 조건부 가능 |
| 사람 없는 무인 인양·운반 | 현재 NO-GO |

자동 테스트는 ROS 2 Jazzy 환경에서 `246 passed`를 확인했다. 이는 Python
로직과 계약 검증이며, 목표 환경인 Ubuntu 22.04 + ROS 2 Humble의 실제
카메라, UART, DDS, STM32, 모터 및 하중 시험을 대체하지 않는다.

## 2. 구현된 주요 기능

- Front/Rear 상판 ArUco ID10/ID11 기반 CCTV 절대 pose
- Rear 카메라가 보는 Front 후면 ArUco ID0 기반 상대 pose
- 엔코더 예측 + CCTV/ArUco 보정 PoseEKF
- UI 입차 승인 게이트와 `/mission/complete` 기반 다음 임무 reset
- Front/Rear 동시 staging 및 `PRE_ALIGN`
- 두 로봇 모두 사전 정렬 후 동시 `SCAN_IN`
- 초음파 좌우 에지로 차축 중심 및 횡오차 계산
- 횡이탈 시 제한된 재시도와 동료 로봇 동기 후퇴
- 결합 차량 크기를 반영한 footprint A*
- 슬롯 밖 staging → 슬롯 yaw 회전 → 슬롯 축 직선 삽입
- 통신/센서/pose/ArUco timeout과 ESTOP 경로
- 차량번호/비밀번호와 목적 slot을 받는 입차 UI, 인증 기반 출차 UI
- park/retrieve 공통 접근·인양·운반·하차와 양쪽 HOME 완료 barrier
- SQLite Parking Registry의 안정 `EMPTY/OCCUPIED` Fleet restart 복구

## 3. Homography 등록 조건

### 3.1 공통 조건

Homography는 반드시 렌즈 왜곡을 보정한 영상 위에서 생성한다.

```text
/cctv/image_raw
  → cctv_rectify_node
  → /cctv/image_rect
  → homography_*_rectified.npy
```

- 실제 바닥 기준점을 줄자로 측정한다.
- 기준점은 한 직선에 몰리지 않게 주차장 전체에 분산한다.
- 슬롯 모서리, 통로, 차량 대기영역이 모두 같은 map 좌표계를 사용해야 한다.
- metre 출력 Homography는 `homography_scale_to_m=1.0`을 사용한다.
- 과거 centimetre 출력 파일만 `0.01`을 사용한다.
- RMS 재투영 오차가 **2 cm 이상이면 재등록**한다.
- 생성된 runtime layout의 `layout_registered`가 `true`인지 확인한다.

### 3.2 CCTV 한 대

다음 파일이 필요하다.

- 카메라 내부 보정 `.npz`
- `homography_rectified.npy`
- `parking_layout.yaml`

### 3.3 CCTV 두 대

카메라별 파일이 각각 필요하다.

- `camera_calibration_cam0.npz`
- `camera_calibration_cam2.npz`
- `homography_cam0_rectified.npy`
- `homography_cam2_rectified.npy`
- 공통 `parking_layout.yaml`

두 카메라에는 **같은 물리 바닥점에 같은 (X,Y) metre 값**을 입력해야 한다. 겹침 영역에 최소 2~3개의 공통 기준점을 포함한다.

정합 합격 기준:

- 겹침 영역의 같은 물체가 두 장애물로 남지 않는다.
- `/cctv/merge_status`에서 `multi_camera_detections >= 1` 확인
- 같은 조건에서 `duplicates_removed >= 1` 확인
- `/parking/map` publisher가 `cctv_merge_node` 하나뿐인지 확인
- 두 카메라 중 하나가 끊겼을 때 보이지 않는 슬롯을 빈자리로 판정하지 않는지 확인

## 4. 실차 전에 반드시 실측할 값

### 4.1 구동계

| 항목 | 현재 명목/기본값 | 필요한 작업 |
|---|---:|---|
| `wheel_radius` | 0.05 m | 하중 포함 유효 구름 반경 측정 |
| `encoder_ppr` | 2600 | 출력축 1회전 누적 카운트 확인 |
| `lx`, `ly` | 0.10, 0.10 m | 바퀴 접점 기준 실측 |
| motor command sign | 미확정 | 바퀴별 잭업 시험 |
| encoder sign | 미확정 | 양의 명령과 증가 방향 일치 확인 |
| PID/PWM 상한 | 실기 미튜닝 | 무부하 → 저하중 계단 입력 튜닝 |

ROS launch 값과 STM32 펌웨어의 `WHEEL_RADIUS`, `ENCODER_PPR`, `LX`, `LY`는 반드시 동일해야 한다.

### 4.2 차량과 로봇 외곽

현재 설정의 다음 값은 모형 기준 placeholder이다.

- wheelbase: 0.70 m
- vehicle length: 0.90 m
- vehicle width: 0.35 m
- robot length: 0.565 m
- robot width: 0.275 m

실제 외장, 그리퍼 돌출부, 센서 브래킷까지 포함해 측정하고 결합 footprint가 통로와 슬롯 안에 들어가는지 다시 계산한다.

### 4.3 ArUco

- 인쇄된 검은 정사각형의 실제 한 변 길이
- Rear 카메라 내부 파라미터와 왜곡계수
- Rear camera ↔ Front ID0 yaw 부착 오차
- `aruco_distance_offset_m`
- ID10/ID11 마커 중심 ↔ 로봇 base 중심 offset
- ID10/ID11 부착 yaw 오차
- 카메라 높이, 마커 높이, 카메라 광축의 바닥 교점

거리 보정 전에는 `use_aruco_distance=false`로 두고 상대 yaw만 사용한다. 정렬 상태에서 raw solvePnP 거리와 실제 로봇 중심 간 거리를 반복 측정한 후에만 거리 융합을 켠다.

### 4.4 초음파와 그리퍼

- 좌/우 `sensor_to_gripper_x_m`
- 좌/우 lateral sign
- 차축 에지 검출 threshold와 hysteresis
- 센서 timeout 주기
- 그리퍼 실제 파지 위치와 차륜 간섭

현재 `GRIP_DONE`은 서보 목표각 도달 신호일 뿐 실제 파지를 증명하지 않는다. 무인 인양에는 리미트 스위치, 서보 전류, 위치 또는 하중 센서가 필요하다.

## 5. 실차 launch에서 반드시 바꿀 안전 설정

`full_system.launch.py`의 기본값은 smoke/integration 용도이며 실차 운용값이 아니다.

현재 주의할 기본값:

```text
enable_serial=false
require_serial=false
require_hardware_ready=false
require_ultrasonic_for_ready=false
use_aruco_distance=true
aruco_distance_offset_m=0.565
```

실차에서는 최소한 다음 정책을 적용한다.

```text
enable_serial=true
require_serial=true
require_hardware_ready=true
require_ultrasonic_for_ready=true
```

또한 `use_aruco_distance`와 `aruco_distance_offset_m`는 launch 인자로 노출하거나 실측 전 안전값으로 수정해야 한다. 현재 `full_system.launch.py`의 ArUco 거리 사용과 0.565 m offset은 하드코딩돼 있으므로 실측 전 그대로 사용하지 않는다.

UI 승인 기본값은 Fleet Manager에서 `require_ui_confirmation=true`이며, `/ui/mission_request`를 받기 전에는 임무가 시작되지 않아야 한다. UI가 죽어도 임의로 주행을 시작하지 않는지 별도로 시험한다.

## 6. STM32와 전기 안전

`stm32/parking_robot`에 CubeMX `.ioc`, 생성 HAL `main.h`,
startup/linker와 통합 제어 소스가 포함되어 있다. 이 프로젝트를 CubeIDE에서
ARM build/link하고 Front/Rear 두 보드에 ST-LINK flash하는 실기 검증은
아직 별도로 수행해야 한다.

필수 전기 안전:

- 물리 비상정지 스위치
- 모터/서보/연산 전원 분리 분기와 공통 GND
- 퓨즈 또는 적정 전류 보호
- HC-SR04 ECHO 3.3 V 레벨 보호
- 서보 전원을 RPi 5 V 핀에서 직접 공급하지 않음
- 인양 시험 지그 및 낙하 방지 구조

## 7. 단계별 실차 검증 절차

각 단계가 실패하면 다음 단계로 넘어가지 않는다.

### 단계 0 — 정적 사전 점검

- ROS 2 Humble `colcon build/test` 통과
- Jetson/RPi 모두 동일 ROS domain 및 DDS 통신 확인
- NTP/chrony 동기화 확인
- `hardware_preflight` 모든 역할 PASS
- Homography, layout, YOLO/TensorRT, Rear calibration 파일 존재

### 단계 1 — 통신

- 두 STM32 UART 연결
- 10 Hz 이상 heartbeat/ACK 유지
- UART parse/CRC 오류 0건
- 통신 차단 후 300 ms 이내 모터 정지

### 단계 2 — 바퀴 공중시험

- 바퀴별 명령 방향, 실제 회전 방향, encoder sign 일치
- 전진/후진/횡이동/제자리 회전 확인
- ESTOP 시 PWM 0 확인

### 단계 3 — 빈 차체 저속 주행

- 로봇 한 대씩 0.03~0.05 m/s 제한
- odometry 축척과 yaw drift 측정
- CCTV pose 보정 전후 오차 기록
- 센서 timeout 시 정지 확인

### 단계 4 — 두 로봇 빈손 동기주행

- wheelbase 거리 유지
- ArUco raw/offset/fused 거리 비교
- 상대 yaw 오차 제한과 장기 마커 손실 정지 시험

### 단계 5 — scan-in/초음파

- `PRE_ALIGN`에서 종방향 정지 확인
- lateral/yaw 허용오차를 만족한 후에만 `SCAN_IN` 확인
- 좌우 에지와 차축 중심 반복 정밀도 측정
- 한쪽 재시도 시 상대도 동기 후퇴하는지 확인
- 센서 고장, 단일 에지, timeout에서 FAULT 확인

### 단계 6 — 그리퍼 무부하

- 좌우 서보 방향과 각도 확인
- 기구 간섭 없음
- ESTOP에서 갑작스러운 해제 없음
- `GRIP_DONE`/`RELEASE_DONE` 순서 확인

### 단계 7 — 저하중 인양

- 보호 지그와 사람 감독
- 양 로봇 완료 전 `/robot/lifted=false`
- 실제 파지/하중 센서가 없으면 무인 진행 금지
- 정지·전원 차단 시 낙하하지 않는지 확인

### 단계 8 — 전체 park/retrieve 사이클

- 차량 인식 → UI 승인 → 정렬 → 인양
- 결합 footprint 경로계획
- staging에서 슬롯 yaw 정렬
- 슬롯 중심축 직선 삽입
- 하차 → 분리 → 두 로봇 HOME → 입차 완료
- Fleet 재시작 뒤 OCCUPIED/credential 복원
- 같은 차량번호/비밀번호로 출차 → waiting 하차 → 양쪽 HOME
- `/mission/complete` 후 source slot이 EMPTY이고 다음 미션 대기

## 8. 최종 GO/NO-GO 기준

다음 중 하나라도 만족하지 못하면 실제 차량 인양 운행은 NO-GO다.

- [ ] Homography RMS < 0.02 m
- [ ] runtime layout `layout_registered=true`
- [ ] 슬롯·통로·대기영역이 동일 map frame
- [ ] 카메라 2대 사용 시 overlap 중복 제거 검증
- [ ] YOLO 차량 mask 외곽 및 yaw 반복성 검증
- [ ] ID10/ID11 절대 pose offset 실측
- [ ] Rear ID0 calibration 및 거리 offset 실측
- [ ] wheel/encoder/kinematic/PID 실측 완료
- [ ] 초음파–그리퍼 offset과 lateral sign 확인
- [ ] `require_serial=true`
- [ ] `require_hardware_ready=true`
- [ ] `require_ultrasonic_for_ready=true`
- [ ] 물리 ESTOP 동작 확인
- [ ] UART/odom/marker/camera 단절 시험 통과
- [ ] 빈 차체 두 로봇 동기주행 통과
- [ ] 보호 지그에서 저하중 인양 통과
- [ ] 실제 파지 또는 하중 확인 수단 확보

## 9. 알려진 구조적 한계

- A*는 인양 직후 한 번 계산하며 주행 중 동적 재계획이 없다.
- 벽과 기둥은 차량 YOLO 모델이 인식하지 않으므로 no-go 영역으로 등록해야 한다.
- 두 CCTV의 설치 높이가 다르면 현재 단일 `camera_height_m` parallax 보정이 부정확할 수 있다.
- `require_full_slot_coverage=false`에서는 슬롯 중심만 관측 범위에 있으면 판정할 수 있다.
- 출차(retrieve)는 아직 구현되지 않았다.
- 웹 UI에는 별도 인증이 없으므로 신뢰 가능한 내부망에서만 사용한다.

## 10. 권장 운영 판정

현재 v1.11은 **실차 저속 단계시험 준비 가능** 수준이다. Homography와 layout을 등록하고 나머지 실측·하드웨어 인터락을 완료하면 감독하의 모형차 1회 주차 시연까지 진행할 수 있다. 실제 파지/하중 검증 센서와 물리 안전장치가 없으면 사람 없는 무인 인양 시스템으로 판정하지 않는다.
