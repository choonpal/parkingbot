# V9 scan-in complete merge — 2026-08-03

> **과거 기록 — v1.8.0/2026-08-03 병합 스냅샷.** 현재 패키지 구조와 검증 범위는
> 저장소의 `ros2/cooperative_parking_robot/docs/README.md`와
> `docs/REAL_WORLD_READINESS.md`를 따른다.

## 기준과 범위

- 기준: `v8_front_first_logic_review_v3_merged_20260801` (`1.7.0`)
- 결과: `v9_scanin_complete_20260803` (`1.8.0`)
- 기존 ZIP과 기준 디렉터리는 수정하지 않았다.

## 함께 유지한 V3 검증 항목

- Front/Rear 동시 staging 및 동시 `SCAN_IN` 기본값
- 기존 Front 우선 순차 진입 선택 가능
- 반대 방향 split exit 기본값과 같은 방향 동기 exit 선택 가능
- 리프트 후 `rigid_body_sync_node` 강체 동기 운반
- ArUco ID10/11 절대 pose, Rear 카메라 ID0 상대 pose
- wheel odometry predict + CCTV ArUco correct EKF
- `/front/odom`, `/rear/odom` Reliable QoS
- OpenCV 4.6 legacy ArUco 안전 생성자
- `yaw_sign`, `gray_gain` 카메라 보정 파라미터
- 결합 footprint 기반 A*

## V9에서 추가한 항목

- 모든 진입 모드에서 `SCAN_IN` 전 `PRE_ALIGN` 연속 표본 게이트
- 동시 모드 `PREALIGNED` 출발 장벽과 peer 연동 후퇴
- 좌우 동시 초음파 에코의 중앙값 기반 차량 기준 횡오차
- 횡이탈 연속 검출, scan origin 후퇴, 에지 검출 reset, 제한 재시도
- segmentation mask PCA + EMA 기반 차량 중심축 yaw
- 모든 실기 launch에 관련 조정 파라미터 노출

## 기본 동작

1. 두 로봇이 동시에 rear staging으로 이동한다.
2. peer staging과 ID0 freshness barrier를 통과한다.
3. `PRE_ALIGN`에서 종방향 정지 상태로 중심선/yaw를 맞춘다.
4. 양쪽 `PREALIGNED` 장벽을 확인한 뒤 동시에 `SCAN_IN`한다.
5. 한쪽의 횡이탈 시 양쪽이 각자의 scan origin으로 함께 후퇴·재정렬한다.
6. 초음파로 각 목표 axle 중심을 잡는다.
7. 두 로봇이 리프트한 뒤 강체 동기 운반한다.
8. release 후 기본값은 서로 반대 방향으로 가까운 끝을 통해 이탈한다.

## 현장 설정이 필요한 값

- `lateral_sign`: 좌우 센서 배선/장착 방향에 맞춰 `+1.0` 또는 `-1.0`
- `lateral_deviation_limit_m`: 실제 차체 하부 여유 기준
- `left/right_sensor_to_gripper_x_m`: 센서와 그리퍼 중심 종방향 오프셋
- `yaw_sign`, `yaw_offset_deg`: Rear 카메라 ID0 실측 보정
- `robot_width_m`, `vehicle_width_m`: 타이어 포함 실제 최대 외곽 폭
- `parking_seg` 모델과 Homography: 비영(非零) 차량 yaw를 사용하려면 필수

## 검증 경계

순수 로직·launch·기존 회귀 테스트는 패키지 테스트로 검증한다. 새 초음파 횡오차의
부호와 크기, 차량 mask yaw, 실제 하부 간극은 장착 형상과 카메라 데이터에
의존하므로 실측 calibration 및 저속 dry-run이 추가로 필요하다.
