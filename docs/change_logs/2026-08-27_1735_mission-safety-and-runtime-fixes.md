# 변경 이력: Mission Safety 및 Runtime 전수 보완

- 작업 일시: 2026-08-27 17:35
- 작업 범위: 최신 main의 인지→Lift→강체 주행 state chain 및 runtime safety
- 관련 기능: Perception / Fleet / Rigid-body / Occupancy / Hardware diagnostics
- 주요 대상 파일: `fleet_manager_node.py`, `cctv_merge_node.py`, `relative_sync_filter.py`, `rigid_body_sync_safe_node.py`, `stm32_bridge_node.py`
- 작업 기준 commit: `782145fd2e861351c11d4288e5ceba68cebc4b40`

## 1. 작업 목적

최신 main에서 제기된 heading ambiguity, stale candidate, Lift/reference 계약,
fatal propagation, 분산 clock 진단, occupancy 경계 및 calibration/runbook 불일치를
실제 코드로 재검증하고 fail-closed 성질을 유지하면서 보완한다.

## 2. 기존 문제

- PCA 차량 yaw가 180도 방향성을 잃은 채 target pose로 발행됐다.
- PARK 승인 전 target callback이 mission ID를 생성했고 candidate timeout이 없었다.
- WAIT_LIFT 진입만으로 CCTV target이 영구 보존될 수 있었다.
- pre-lift 허용 자세보다 post-lift reference envelope가 더 좁았다.
- ArUco X를 disable해도 reference X sanity/dispersion에 사용했다.
- reference INVALID/TIMEOUT은 같은 Lift 세션에서 retry되지 않았다.
- lateral/reference fatal이 Robot/Fleet에 모두 전달되지 않았다.
- command stamp reject 진단에 두 host의 시각과 delta가 없었다.
- invalid vehicle dimension default가 mission spec으로 한 번 latch될 수 있었다.
- stale robot odom self-mask, grid truncation, map origin 누락, static no-go 부재가 있었다.

## 3. 수정 내용

### `vision_utils.py`, `yolo_bev_map_node.py`, `cctv_merge_node.py`

수정한 함수/클래스:

- `directed_axis_yaw()`
- `publish_target()` / `_publish_target()`
- `CctvMergeNode`

변경 내용:

- PCA 축의 두 후보 중 `waiting_yaw_deg`에 가까운 방향을 선택했다.
- WAIT_LIFT target 보존을 `/robot/lifted=true`와 결합했다.
- segmentation dimension이 valid해질 때까지 vehicle spec 발행을 보류했다.
- odom freshness, map origin, ceil grid, static obstacle 우선 mask를 추가했다.

### `fleet_manager_node.py`

수정한 함수/클래스:

- `target_cb()`
- `manage_loop()`
- `sync_status_cb()`

변경 내용:

- WAIT_TARGET 입력을 timeout 가능한 candidate로 저장하고 PARK 승인 시 mission으로 승격했다.
- stale candidate와 pending PARK request를 함께 무효화했다.
- 공통 fatal 분류를 사용해 active Fleet mission을 `FAULT`로 정지시켰다.

### `relative_sync_filter.py`, `rigid_body_sync_safe_node.py`

수정한 함수/클래스:

- `MissionReferenceCapture`
- `control_loop()`
- `aruco_cb()`
- `_lock_mission_reference()`

변경 내용:

- optional X capture와 bounded retry/cooldown/final failure를 구현했다.
- `use_aruco_distance=false`이면 y/yaw만 visual sanity를 적용하고 X는 wheel predictor에서 lock한다.
- reference envelope를 pre-lift alignment 계약(60 mm / 30 mm / 4 deg)과 맞췄다.

### `sync_faults.py`, `robot_state_machine_node.py`, `stm32_bridge_node.py`

수정한 함수/클래스:

- `is_fatal_sync_error()`
- `sync_cb()`
- `_report_stamp_rejection()`

변경 내용:

- lateral, relative-X, reference failure를 shared fatal prefix로 분류했다.
- stamp reject에 reason, message stamp, local now, signed age를 기록한다.
- 3회 연속 stale/future reject 시 `ERR,CLOCK_SKEW` hardware telemetry를 발행한다.

## 4. 변경 전 / 변경 후

```text
[변경 전]
PCA axis → undirected target
target 수신 → mission 생성
WAIT_LIFT → target preserve
reference failure → permanent hold
sync fatal → Robot/Fleet 일부 누락

[변경 후]
PCA axis + waiting heading → directed target
target 수신 → expiring candidate → PARK 승인 → mission 생성
WAIT_LIFT + lifted=true → target preserve
reference failure → bounded retry → exhausted fatal
sync fatal → Robot FAULT + Fleet FAULT
```

## 5. 주요 알고리즘 및 제어 로직

- Relative x/y/yaw와 wheel-first DeltaKalman, ID0 우선, CCTV stale fallback은 유지했다.
- Mission reference는 median/std와 nominal envelope를 통과한 뒤 mission 동안 고정된다.
- X visual disable 시 calibration되지 않은 ArUco X를 gate/sanity/update 어디에도 사용하지 않는다.
- Static obstacle은 occupied layer로 먼저 고정하며 robot self-mask가 FREE로 덮지 못한다.
- 분산 command timestamp 검사는 제거하지 않고 반복 reject를 fail-fast diagnostic으로 승격했다.

## 6. 추가/변경된 파라미터

| Parameter | Default | 의미 |
|---|---:|---|
| `target_candidate_timeout_s` | 2.0 | WAIT_TARGET candidate 최대 age |
| `sync_reference_max_retries` | 2 | reference 추가 시도 횟수 |
| `sync_reference_retry_delay_s` | 0.3 | 시도 사이 cooldown |
| `robot_odom_freshness_s` | 0.5 | CCTV self-mask odom freshness |
| `static_obstacle_polygons_json` | `[]` | map-frame static no-go polygons |
| `clock_reject_fault_count` | 3 | CLOCK_SKEW telemetry 발생 연속 횟수 |

변경된 reference sanity는 x 0.060 m, y 0.030 m, yaw 4 deg이며 pre-lift
alignment acceptance와 같은 계약이다. PID와 Kalman Q/R은 변경하지 않았다.

## 7. Topic / Interface 변경

- 기존 `/front|rear/hardware_status`에 `ERR,CLOCK_SKEW:*` 진단 값이 추가됐다.
- 기존 `/fleet/state` JSON에 `sync_fault`가 추가됐다.
- `static_obstacle_polygons_json` layout parameter가 추가됐다.
- 기존 topic 이름과 message type은 변경하지 않았다.

## 8. Safety 영향

- stale command rejection, ESTOP, marker/odom freshness, fail-closed 정책을 유지했다.
- Lift 확인 전 detection 소실은 더 이상 preserved target으로 숨겨지지 않는다.
- reference retry는 제한되며 소진 후 fatal이다.
- STM32 ESTOP latch를 안전하게 해제할 firmware protocol이 없으므로 자동 FAULT reset은 추가하지 않았다.
  Lift 후 Registry 자동 rollback도 수행하지 않는다.

## 9. 테스트

실행:

```bash
python3 -m pytest -q ros2/cooperative_parking_robot/test
python3 -m compileall ros2/cooperative_parking_robot
colcon build --symlink-install --packages-select cooperative_parking_robot
colcon test --packages-select cooperative_parking_robot
colcon test-result --verbose
git diff --check
```

결과:

- pytest: `512 passed in 8.04s`
- compileall: PASS
- colcon build: 1 package PASS
- colcon test: command PASS, 하지만 setup이 테스트를 등록하지 않아 `0 tests`
- YAML parse 3 files: PASS
- 실제 카메라/STM32/RPi: NOT RUN

## 10. 실차 검증 필요 항목

- PCA target heading과 cam0/cam2 preview 방향
- target 소실과 Lift edge의 실제 DDS 순서
- reference 10회 반복성과 retry/fatal telemetry
- ID0 loaded-state FPS 및 0.570 m calibration 재확인
- Jetson/Front/Rear chrony skew 20 ms 이하
- stale self-mask와 static obstacle 실제 OccupancyGrid
- vehicle dimension 안정화까지 PARK 승인 보류 동작

## 11. 남은 문제 / 위험요소

- STM32 ESTOP은 MCU reset까지 latched되며 안전한 remote clear protocol이 없다.
- post-Lift fault는 실제 차량/그리퍼 상태를 알 수 없어 자동 Registry recovery가 금지된다.
- CCTV source-camera ID는 relative fallback pair message에 없으므로 common-source 강제는 불가능하다.
- `0.570m` ID0 offset은 operator 실측 근거가 있으나 하중 상태 반복성 검증이 남았다.
- GitHub CI는 ROS-independent subset만 실행하며 ROS Humble colcon test 등록은 별도 보완이 필요하다.

## 12. 수정 요약

- waiting heading으로 PCA 180도 방향성을 복원했다.
- candidate target과 active mission을 분리했다.
- Lift 확인 전 CCTV target preservation을 차단했다.
- optional ArUco X와 bounded reference retry를 구현했다.
- fatal sync error를 Robot/Fleet에 공통 전파했다.
- clock skew reject를 정량 진단으로 만들었다.
- valid vehicle dimension만 mission spec으로 발행한다.
- occupancy origin/ceil/fresh self-mask/static no-go를 추가했다.
- dual runtime geometry와 ID0 runbook을 최신 코드에 맞췄다.
