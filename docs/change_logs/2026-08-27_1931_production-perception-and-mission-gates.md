# Production perception 및 mission gate 보완

## 발견된 문제와 root cause

- `waiting_yaw_deg`가 Fleet 전용 YAML block에만 있어 dual-CCTV의 YOLO/merge는
  각자 `0 deg` 기본값을 사용했다. PCA의 180도 ambiguity가 production target
  pose를 반대로 향하게 할 수 있었다.
- homography tool output은 runtime canonical layout과 달리 map origin이 없고
  4.40 x 3.83 m 크기를 유지했다. copy script는 calibration 갱신 때 이 stale
  layout도 무조건 runtime에 덮어썼다.
- X-disabled reference sample은 `None`인데 telemetry가 그대로 `pstdev`를
  호출했다. 또한 공통 ArUco callback이 X 사용 여부와 무관하게 X envelope로
  observation 전체를 폐기했다.
- UI/Fleet PARK 승인 조건에는 segmentation vehicle dimension의 validity와
  freshness가 없었다. Fleet의 default footprint fallback이 production에서도
  mission을 진행시킬 수 있었다.

## 수정 파일과 방법

- `config/parking_layout.yaml`, `bev_layout_core.py`, homography output:
  `waiting_yaw_deg`를 wildcard 공용 parameter로 승격하고 canonical negative
  origin/size를 generated layout에도 보존했다.
- `copy_results_to_project.sh`: calibration-only 기본 실행은 runtime layout을
  보존한다. `--include-layout` 명시 시에만 layout을 backup 후 교체한다.
- `relative_sync_filter.py`, `rigid_body_sync_node.py`,
  `rigid_body_sync_safe_node.py`: disabled X를 unavailable로 유지하고 X gate만
  조건부 적용했다. Y/yaw validation과 correction은 그대로 유지한다.
- `fleet_manager_node.py`: production-selectable `require_valid_vehicle_spec`과
  timestamp freshness gate를 PARK request 및 자동 transition 양쪽에 적용했다.
- `cctv_merge_node.py`: 차량이 계속 유효하게 관측되는 동안 spec timestamp를
  1초 주기로 갱신해 Fleet freshness가 단발 publish 때문에 만료되지 않게 했다.
- `jetson_vision_web_node.py`: fresh `dimension_valid=true` vehicle spec을 PARK
  button 조건에 추가하고 `WAITING_VEHICLE_DIMENSION` reason/banner를 노출했다.
- production dual/smoke launch에서 Fleet의 strict spec gate를 활성화했다.

최신 main의 detection hold/flicker 완화와 duplicate merge 로직은 변경하지
않았으며 해당 회귀 테스트를 함께 실행했다.

## 추가한 regression test

- PCA 0/180 axis와 -180/180 wrap-around의 expected waiting direction 선택
- wildcard waiting yaw가 Fleet/YOLO/merge에 공통 적용되는 layout 구조
- canonical/generated map origin/size 일치 및 negative origin 보존
- calibration copy 기본 경로가 기존 runtime layout을 보존
- X sample 전체가 `None`이어도 Y/yaw reference가 lock되고 X std는 `None`
- X-disabled callback이 X finite/envelope를 필수 gate로 사용하지 않는 wiring
- production Fleet request/transition 양쪽의 valid/fresh dimension gate
- UI의 dimension wait reason 및 PARK disable 조건

## 실행한 테스트와 결과

- ROS-independent 관련 pytest 묶음: `131 passed`
- 보강 후 핵심 pytest 묶음: 최종 결과는 아래 재검증 결과에 따름
- `copy_results_to_project.sh <temp-dir>`: sentinel runtime layout 보존 확인
- Python `compileall`: 성공
- `git diff --check`: 성공

## 실행하지 못한 테스트 및 이유

- `test_operator_ui_requests.py`, `test_v9_scanin_complete.py`를 포함한 ROS node
  import 테스트: 현재 shell에 ROS 2 Python package `rclpy`가 없어 collection
  단계에서 `ModuleNotFoundError`가 발생했다. 통과로 간주하지 않는다.
- 실제 ROS 2 launch/통합 테스트: 동일하게 ROS 2 runtime 미설치로 미실행.

## 남아 있는 실차 검증

1. Jetson에서 runtime YAML 로드 후 YOLO cam0/cam2, merge, Fleet parameter가 모두
   `waiting_yaw_deg=180`인지 `ros2 param get`으로 확인한다.
2. 실제 차량을 waiting zone에 두고 target yaw 및 approach 시작 geometry를
   rosbag/overlay로 확인한다.
3. segmentation dimension stability/freshness 경계에서 PARK button과 direct
   mission request가 동일하게 차단되는지 확인한다.
4. RPi에서 `use_aruco_distance=false`로 absurd/missing X와 정상 Y/yaw를 주입해
   reference/제어 지속을 확인하고, true에서는 동일 X가 reject되는지 확인한다.
5. `--include-layout`은 현장 geometry 검토와 backup 확인 후에만 사용한다.
