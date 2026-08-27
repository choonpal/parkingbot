# 변경 이력: Production 강체 Mission Reference 및 센서 역할 분리

- 작업 일시: 2026-08-27 14:40
- 작업 범위: 실제 차량 Lift 이후 production 강체 내부 상대제어
- 관련 기능: relative x/y/yaw fusion, ID0 우선순위, CCTV fallback, mission reference capture
- 주요 대상 파일: `rigid_body_sync_safe_node.py`, `relative_sync_filter.py`, `sync_params.yaml`, 관련 테스트

## 1. 작업 목적

강체 내부 estimator의 세 상태를 Rear body-frame `relative_x`, `relative_y`, `relative_yaw`로 통일하고, healthy ID0가 있는 동안 CCTV top-marker pair가 내부 filter에 개입하지 않도록 센서 역할을 분리한다. 또한 nominal geometry를 PID target으로 직접 사용하지 않고, 정상 Lift 직후 안정된 ID0 여러 frame으로 mission-specific reference를 한 번 capture하고 mission 동안 고정한다.

키보드 leader/follower, legacy controller, path planning, final approach는 수정하지 않았다. production `rigid_body_sync` entrypoint가 계속 `rigid_body_sync_safe_node:main`을 사용하는 것을 확인했다.

## 2. 기존 문제

### 서로 다른 첫 번째 상태의 혼용

raw wheel predictor와 CCTV pair는 다음 값을 filter에 넣었다.

```text
sqrt(relative_x² + relative_y²)
```

반면 ID0의 보정된 `position.x`는 Rear 기준 Front 중심의 forward X였다. lateral이 0이 아니면 Euclidean norm과 forward X는 다른 물리량이므로 같은 `DeltaKalman1D`에서 융합하면 안 된다.

### ID0가 healthy해도 CCTV가 개입

현재 control cycle에서 소비할 새 ID0 timestamp가 없으면 바로 CCTV pair를 시도했다. 따라서 ID0 stream age가 정상이어도 그 사이 cycle에 homography/dual-camera registration bias가 강체 내부 correction으로 들어올 수 있었다.

### PID target이 nominal geometry

DRIVE 중 x/y/yaw error는 wheelbase, configured lateral 0, yaw 0을 기준으로 계산했다. 실제 Lift 접촉점, 타이어 변형, marker/camera mounting bias가 정상 범위 안에서 존재해도 이를 계속 제거하려는 힘이 발생할 수 있었다.

## 3. 수정 내용

### `cooperative_parking_robot/relative_sync_filter.py`

수정한 클래스/함수:

- `MissionReference`, `MissionReferenceCapture`
- `stream_is_healthy()`
- `cctv_fallback_allowed()`
- `CctvPairStampGate`
- `reference_blocks_drive()`

변경 내용:

- unique ID0 x/y/yaw sample을 수집하고 median, 표준편차, nominal sanity를 확인해 reference를 lock한다.
- yaw sample은 첫 sample 주변으로 unwrap한 뒤 median을 구하고 wrap-aware RMS dispersion을 계산한다.
- ID0 age 기반 health와 CCTV fallback 허용 정책을 제어 loop와 독립적으로 검증 가능하게 했다.
- CCTV pair timestamp slop과 pair당 한 번 소비를 별도 gate로 명시했다.
- Lift/두 로봇 DRIVE 상태에서 reference가 준비되지 않으면 command를 차단하는 조건을 분리했다.

### `cooperative_parking_robot/rigid_body_sync_safe_node.py`

수정한 클래스/함수:

- `RigidBodySyncNode.__init__()`
- `vehicle_lifted_cb()`, `vehicle_spec_cb()`, `path_cb()`
- `control_loop()`, `_reference_telemetry()`
- `aruco_cb()`, `_lock_mission_reference()`
- `_raw_wheel_relative()`, `_relative_predictor()`
- `_new_cctv_pair()`, `_consume_visual_measurement()`
- `apply_sync_and_publish()`

변경 내용:

- production 첫 filter를 명확한 `relative_x_kalman`으로 정리했다.
- raw wheel, fused odom fallback, CCTV pair가 Euclidean norm 대신 Rear-frame longitudinal X를 반환한다.
- ID0 age가 `aruco_timeout_s` 안이면 새 frame이 없는 cycle에서도 CCTV pair를 처리하지 않는다.
- CCTV는 ID0 actual stale 이후에만 freshness, synchronized pair, one-pair-one-use, 축별 innovation gate를 거쳐 correction한다.
- CCTV fallback 중에도 ID0 loss가 길어지면 감속 후 recoverable hold하도록 제한했다.
- Lift rising edge에서 reference capture를 시작하고 settle 기간 이후 ID0 sample만 수집한다.
- reference가 `READY`가 아니면 production DRIVE control loop가 zero command를 유지한다.
- invalid/timeout capture는 nominal fallback 없이 recoverable hold한다.
- reference lock 시 filter를 captured x/y/yaw와 현재 raw predictor anchor로 일치시킨다.
- PID error를 fused state와 locked mission reference의 차이로 변경했다.
- path/mission reset 및 Lift 해제 시 reference를 reset한다.

### `config/sync_params.yaml`

Mission reference sample 수, timeout, settle, nominal sanity, sample stability 파라미터를 추가했다. 기존 wheelbase, `sync_target_lateral_m`, 새 `sync_target_yaw_deg`는 DRIVE PID target이 아니라 capture sanity용 nominal geometry다.

### 테스트

`test_relative_sync_filter.py`, `test_motion_control.py`를 보강하고 `test_safe_relative_sensor_priority.py`를 추가했다.

## 4. 변경 전 / 변경 후

```text
[변경 전]

wheel/CCTV: hypot(x,y) ─┐
                        ├─ distance filter ─ nominal wheelbase error
ID0: forward x ─────────┘

새 ID0 없는 한 cycle → CCTV correction 가능


[변경 후]

raw wheel → rear-frame x/y/yaw delta predict
                         ↑
healthy ID0 → authoritative x/y/yaw correction
                         ↓
Lift 후 stable samples → median/sanity/stability → REFERENCE LOCK
                         ↓
ex = fused_x - locked_x_ref
ey = fused_y - locked_y_ref
eyaw = wrap(fused_yaw - locked_yaw_ref)
                         ↓
x/y/yaw PID → symmetric correction → common pair saturation

ID0 actual stale only → qualified CCTV fallback → bounded slowdown/hold
```

## 5. 주요 알고리즘 및 제어 로직

### 상대상태 좌표계

- `relative_x`: Rear 중심에서 Front 중심까지의 world displacement를 Rear yaw로 회전한 forward 성분
- `relative_y`: 같은 displacement의 Rear-left positive lateral 성분
- `relative_yaw`: `wrap(front_yaw - rear_yaw)`

raw wheel, calibrated ID0, CCTV top-marker pair 모두 동일한 세 physical state를 사용한다.

### Sensor fusion 우선순위

1. raw wheel odom으로 고주기 delta predict
2. ID0가 fresh하면 timestamp당 한 번 authoritative correction
3. ID0 receipt age가 `aruco_timeout_s`를 넘은 경우에만 CCTV pair fallback
4. raw wheel이 stale일 때만 fused odom predictor fallback

CCTV source camera ID는 현재 PoseStamped interface에 없어 same-camera 검증은 할 수 없다. 가능한 조건인 두 marker local freshness, stamp sync slop, monotonic pair consumption, 축별 innovation/reacquire는 적용했다.

### Reference capture

Lift rising edge에서 state를 `REFERENCE_CAPTURE`로 전환한다. settle time 이후 calibration 적용된 unique ID0 sample을 설정 개수만큼 모은다. x/y는 median과 population standard deviation, yaw는 wrap-aware median과 RMS dispersion을 구한다.

median이 nominal x/y/yaw sanity envelope 안이고 세 축 dispersion도 허용값 이하여야 `REFERENCE_READY`가 된다. 한 번 lock된 object는 추가 sample을 받지 않으며 새 path 또는 Lift 해제 시에만 reset한다.

### PID, feed-forward, saturation

```text
ex = fused_relative_x - mission_relative_x_ref
ey = fused_relative_y - mission_relative_y_ref
eyaw = wrap(fused_relative_yaw - mission_relative_yaw_ref)
```

기존 deadband/PID, Front/Rear symmetric correction, `RigidBodyKinematics.split()` feed-forward, `limit_twist_pair()` common scale을 유지한다.

### Stale와 safety

- reference capture 중: zero command, `REFERENCE_CAPTURE`
- sanity/dispersion 실패: `REFERENCE_INVALID` recoverable hold
- ID0 sample 부족: `REFERENCE_TIMEOUT` recoverable hold
- actual visual loss: 기존 slowdown → `MARKER_HOLD`
- CCTV fallback이 ID0 loss를 가리더라도 ID0 loss duration으로 감속 → `ID0_LOSS_HOLD`
- correction 축 stale: 기존 `*_CORRECTION_STALE/HOLD`

## 6. 추가/변경된 파라미터

| Parameter | Default | 의미 |
|---|---:|---|
| `sync_target_yaw_deg` | 0.0 | reference sanity용 nominal relative yaw |
| `sync_reference_capture_samples` | 20 | lock에 필요한 unique ID0 sample 수 |
| `sync_reference_capture_timeout_s` | 5.0 | capture 제한 시간 |
| `sync_reference_settle_time_s` | 0.5 | Lift 직후 sample 제외 시간 |
| `sync_reference_max_x_error_m` | 0.06 | captured X와 nominal wheelbase 차이 한계 |
| `sync_reference_max_lateral_error_m` | 0.04 | captured Y와 configured target 차이 한계 |
| `sync_reference_max_yaw_error_deg` | 5.0 | captured yaw와 nominal yaw 차이 한계 |
| `sync_reference_max_sample_std_x_m` | 0.01 | X sample 표준편차 한계 |
| `sync_reference_max_sample_std_y_m` | 0.01 | Y sample 표준편차 한계 |
| `sync_reference_max_sample_std_yaw_deg` | 2.0 | yaw sample dispersion 한계 |

기존 PID와 Kalman Q/R 기본값은 변경하지 않았다.

## 7. Topic / Interface 변경

ROS topic, service, message type, launch argument 변경 없음. `sync_params.yaml` parameter와 기존 error-state JSON telemetry만 확장했다.

추가 telemetry:

- raw/fused/reference/error `relative_x`, `relative_y`, `relative_yaw`
- `reference_state`, `reference_sample_count`
- capture x/y/yaw dispersion
- `id0_stream_age`, `id0_stream_healthy`
- `relative_correction_source`: `ID0_WHEEL`, `CCTV_FALLBACK`, `WHEEL_ONLY`

기존 telemetry field는 호환성을 위해 유지했다.

## 8. Safety 영향

- reference 없는 DRIVE를 명시적으로 차단한다.
- 잘못된 Lift pose나 흔들리는 ID0를 reference로 받아들이지 않는다.
- nominal fallback으로 조용히 진행하지 않는다.
- ID0 healthy 상태에서 CCTV registration bias가 내부 강체 correction으로 들어오는 경로를 차단한다.
- CCTV fallback은 ID0 장기 손실을 무기한 숨기지 못한다.
- 기존 `OncePerStamp`, 축별 gate/reacquire, marker loss, correction stale, fatal distance/yaw, ESTOP 구분은 유지했다.

새 failure mode는 ID0 frame rate가 낮거나 capture 기준이 현장 noise보다 엄격하면 DRIVE가 시작되지 않는 것이다. 이는 안전 쪽 실패이며 실차 로그로 threshold를 조정해야 한다.

## 9. 테스트

실행 명령:

```bash
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR=/tmp/parkingbot-ros-logs
python3 -m pytest -q test/test_relative_sync_filter.py \
  test/test_motion_control.py test/test_safe_relative_sensor_priority.py
PYTHONNOUSERSITE=1 python3 -m pytest -q \
  -k 'not test_package_and_setup_versions_match'
python3 -m compileall -q cooperative_parking_robot launch test
ament_flake8 cooperative_parking_robot/relative_sync_filter.py \
  cooperative_parking_robot/rigid_body_sync_safe_node.py \
  test/test_relative_sync_filter.py test/test_motion_control.py \
  test/test_safe_relative_sensor_priority.py
git diff --check
```

결과:

```text
관련 테스트: 47 passed
전체 ROS 회귀: 426 passed, 1 deselected
compileall: 통과
변경 파일 flake8: No problems found
git diff --check: 통과
production node 생성: 성공
```

제외된 테스트는 기존 package/setup `1.11.1`과 test 기대값 `1.11.0` 불일치로 이번 범위 밖이다. production node 생성 시 sandbox의 DDS UDP socket 경고가 있었지만 node와 reference capture 초기화는 성공했다.

## 10. 실차 검증 필요 항목

- ID0가 발행하는 calibrated x/y/yaw와 Rear body-frame 정의 일치
- Lift 완료 후 실제 진동이 0.5초 settle 및 dispersion 기준 안에 들어오는지
- 20 samples를 5초 안에 안정적으로 수집하는지
- captured reference와 nominal wheelbase/lateral/yaw 편차 분포
- reference lock 전 두 로봇 command가 실제 STM32에서 zero인지
- ID0 unplug/loss 시 CCTV fallback, 감속, hold 시간 순서
- cam0/cam2가 서로 다른 marker를 볼 때 registration bias
- 실제 차량 하중에서 locked reference 유지와 횡방향 기구 하중

## 11. 남은 문제 / 위험요소

- CCTV PoseStamped에는 source camera ID가 없어 common-source나 camera transition을 판정할 수 없다.
- reference threshold와 settle/sample/timeout 기본값은 실차 로그로 확정하지 않았다.
- reference capture는 command stop과 sample dispersion으로 안정성을 판단하며 wheel twist/속도를 직접 구독해 정지 판정하지 않는다.
- ID0 static calibration bias는 mission reference에 포함되지만 calibration 자체를 대신하지 않는다.
- CCTV fallback은 제한적으로만 허용되며 장기 ID0 장애 시 운행 지속 기능이 아니다.
- package/setup version test 불일치가 별도 잔여 문제로 남아 있다.

## 12. 수정 요약

- Euclidean distance와 ID0 forward X 혼용을 제거했다.
- production 상태를 Rear-frame relative x/y/yaw로 통일했다.
- ID0 age가 healthy하면 CCTV relative correction을 차단한다.
- CCTV fallback에 freshness, stamp sync, one-use, innovation 조건을 적용했다.
- Lift 후 stable ID0 median으로 mission reference를 한 번 lock한다.
- nominal geometry는 sanity check, locked reference는 DRIVE PID target으로 분리했다.
- reference 준비 전 DRIVE를 zero-command로 보류한다.
- invalid/timeout capture는 recoverable hold한다.
- reference/source/health telemetry와 회귀 테스트를 추가했다.
- 실차 capture threshold와 CCTV registration bias 검증은 남아 있다.
