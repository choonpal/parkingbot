# v1.2 코드 검토·수정 보고서

> **과거 기록 — v1.2 코드 검토 스냅샷.** ROS 2 Humble 배포 변경은 v1.3에서 수행했다.
> 현재 실행 절차는 저장소의 `docs/REAL_ROBOT_DEPLOYMENT_RUNBOOK.md`, 현재 구조는
> `ros2/cooperative_parking_robot/README.md`를 따른다.

## 적용 범위

- YOLO 기반 차량·빈자리 검출과 BEV OccupancyGrid 유지
- 한 종류 모형차, 고정 휠베이스
- 차량 한 대, 한 번의 전체 시연
- 고정 시연 경로이므로 정적 장애물/전체 운반체 footprint 확장은 보류
- 메카넘 특성을 이용해 차량 yaw를 유지한 채 전진·후진·횡이동

## 주요 수정

### 1. 후진하면서 목표를 향해 회전하던 문제

기존 `PurePursuit`는 뒤쪽 목표에서 `vx<0`과 동시에 목표 heading 오차로 `omega`를 냈다. 이를 홀로노믹 body-frame 평행이동 추종기로 바꿨다.

- 뒤쪽 목표: `vx<0`, 기본 `omega=0`
- 옆쪽 목표: `vy≠0`
- 전체 차량 yaw는 `rigid_body_sync_node`의 별도 yaw-hold가 복구
- 필요할 때만 `rotate_to_path=True`를 명시적으로 사용

### 2. 코너를 지난 뒤 이전 waypoint로 후진하던 문제

가장 가까운 waypoint가 아니라 현재 위치를 경로 segment에 투영하고, 진행 index가 뒤로 가지 않는 lookahead를 계산한다.

### 3. FINAL_APPROACH에서 동기 보정이 사라지던 문제

최종 접근 함수가 직접 명령을 발행하지 않고 `(vx, vy, omega)`만 계산한다. 일반 경로와 최종 접근 모두 다음 공통 단계를 거친다.

```text
강체 속도 분배
→ 엔코더/ArUco 상대거리·yaw 융합
→ 거리 PID + 상대 yaw PID
→ marker/odom/오차 fail-safe
→ 속도 상한
→ front/rear cmd_vel 발행
```

### 4. ArUco 거리 기준 불일치

solvePnP의 값은 Rear 카메라–Front 마커 거리이며 로봇 중심 간 거리가 아니다.

- `aruco_distance_offset_m` 추가
- 실측 전 `use_aruco_distance=false`: 거리에는 엔코더 사용, ArUco 상대 yaw만 사용
- 실측 후에만 중심거리 보정 활성화

### 5. 임무 시작 상대필터 초기값

거리 칼만필터를 무조건 고정 휠베이스에서 시작하면 실제 초기 정렬 오차를 놓친다. 경로 수신 후 최신 Front/Rear odom으로 거리와 상대 yaw 상태·raw 기준을 함께 초기화한다.

### 6. 차량 중심 좌표

YOLO 차량 중심과 두 로봇 기하학적 중점이 다를 수 있다. 인양 전 latched `/parking/target_pose`로 초기 차량중심 오프셋을 계산하고 이후 CCTV feedback을 gate+저역통과로 갱신한다.

### 7. 고정 휠베이스 일관성

YOLO/분류는 유지하지만 차종 분류 결과가 휠베이스를 변경하지 못하도록 기본값을 고정 모드로 바꿨다. `individual_move`, `rigid_body_sync`, launch가 같은 값을 쓴다.

### 8. YOLO/맵·A*

- 수신 OccupancyGrid의 실제 resolution을 A* 플래너에 반영
- 잘못된 grid 크기/메타데이터 폐기
- 커스텀 YOLO의 empty_slot 검출이 0개인 경우 빈 PoseArray를 발행; 만차를 빈자리로 만드는 DB fallback 제거

### 9. 상태머신·통신

- ACK가 주행 중 끊기면 `HARDWARE_ACK_TIMEOUT`으로 FAULT/ESTOP
- 동기제어의 odom/marker/거리/yaw fatal 상태를 상태머신에 전파
- UART byte stream에서 newline까지 조립한 완전한 프레임만 파싱
- A* 경로와 최종 슬롯은 transient-local/reliable QoS로 보존해 Front가 늦게 연결돼도 마지막 임무 수신
- retained 경로는 `/robot/lifted=true`이고 Front/Rear 상태가 모두 `DRIVE`일 때만 실행해 노드 재시작 후 stale 임무 재생 차단
- ESTOP latch 뒤에는 속도 프레임과 반복 ESTOP 송신을 멈춰 `ESTOP_LATCHED` 오류 폭주 방지
- 초음파 에지 검출은 ALIGN 상태에서만 활성화하고 ALIGN 진입 시 필터 초기화
- Front/Rear 복귀 좌표 분리

### 10. STM32F401RE·안전

- 존재하지 않는 TIM8 참조 제거
- 모터 PWM: TIM1 CH1~4
- 엔코더: TIM2/3/4/5
- 서보: TIM10/TIM11 CH1
- 16-bit encoder rollover 처리
- 첫 heartbeat/속도 명령 전 startup timeout 오탐 방지
- 모터/엔코더 방향 보정 배열 추가
- ESTOP 시 모터 정지와 동시에 서보 진행을 중단하고 현재 각도 유지

### 11. 하드웨어 상수

BOM의 100 mm 메카넘 휠에 맞춰 명목 `wheel_radius=0.05 m`로 통일했다. Front/Rear launch에서 `wheel_radius`, `encoder_ppr`, `lx`, `ly`를 인자로 노출했다. 실제 유효반경과 PPR은 실측값으로 바꿔야 한다.

### 12. 양쪽 하차 완료 장벽과 DRIVE 이중 확인

- 한 로봇만 `RELEASE_DONE`을 받은 상태에서 먼저 복귀하지 않도록
  `/release/front_done`, `/release/rear_done`을 서로 확인한 뒤에만 RETURN한다.
- retained 경로는 `/robot/lifted=true`뿐 아니라 Front와 Rear 상태가 모두
  `DRIVE`일 때만 실행한다. 한쪽 상태 전이가 늦거나 노드가 재시작해도 다른
  로봇에 먼저 속도 명령이 나가지 않는다.

### 13. CCTV 차량 피드백 임무 상태 게이트

하차 후에는 차량이 슬롯에 남고 로봇만 복귀하므로, YOLO 차량 좌표를 계속
오프셋 보정에 쓰면 내부 차량 중심 오프셋이 오염된다. `/robot/lifted=true`,
Front/Rear 모두 `DRIVE`, 활성 경로 보유 조건에서만 CCTV 차량 피드백을 반영한다.

## 자동 검증

```bash
python3 -m compileall -q cooperative_parking_robot launch test
PYTHONPATH=. pytest -q
gcc -std=gnu11 -fsyntax-only -Itest/firmware_stub \
  ../../stm32/parking_robot/Core/Src/parking_robot_firmware.c
```

결과:

```text
Python compileall       PASS
pytest                  29 passed
STM32 C GNU11 syntax    PASS
```

## 검토 후 남은 사항

### 실차 전 필수

1. 바퀴 유효반경, 출력축 실제 PPR, `lx`, `ly`, 고정 휠베이스 실측
2. 모터/엔코더 부호, PID, PWM 상한, 서보 좌우 각도 튜닝
3. Rear 카메라 intrinsics, 마커 크기/yaw, ArUco 중심거리 offset 실측
4. CCTV homography, 상판 마커 offset/높이/parallax, YOLO 모델 클래스 검증
5. Jetson/RPi 시계 동기화와 카메라/UART 실제 launch 시험
6. 실제 파지 센서 또는 사람 감독·보호 지그

### 의도적으로 미구현

- 두 번째 차량을 위한 latch/state reset
- 복수 천장 카메라 좌표 통합
- 정적 구조물 및 운반체 전체 footprint 기반 재계획
- 다중 차량 ID 추적

현재 사용자 정의 시연 범위에서는 위 항목이 소프트웨어 진행을 막는 blocker는 아니지만, 보고서에서 구현 완료로 과장하면 안 된다.
