# 변경 이력: 강체제어 P0 Production Guards

## 적용 범위

`Lift → Path → DRIVE` 실제 상태 순서에서 mission reference가 지워지는 문제와,
실차 lateral 하중·분산 wheel timestamp로 인한 허위 상대오차를 함께 보완했다.

## 핵심 변경

- production `rigid_body_sync` entrypoint를 `rigid_body_sync_production_node`로 전환
- path 및 재계획 수신 시 Lift-session mission reference 유지
- Lift 해제에서만 lateral P0 상태와 synchronized wheel cache 초기화
- Front/Rear wheel source timestamp 차이가 50 ms 이내인 pair만 새 predict에 사용
- 비동기 구간에는 마지막 synchronized relative pose를 hold
- lateral 20 mm 초과 감속, 40 mm 즉시 정지, 1 s 지속 시 정지
- mission reference nominal sanity를 x 25 mm, y 20 mm, yaw 3 deg로 축소
- lateral PID를 저속 P-only(Kp 0.4, correction cap 0.015 m/s)로 변경
- deadband가 0 error를 전달하면 PID integral/previous state를 reset
- 위 동작을 검증하는 ROS-independent 회귀 테스트 추가

## 실차 적용 전 확인

1. `humble_build_check.sh` 전체 통과
2. Lift 후 `/sync/error_state`에서 `REFERENCE_CAPTURE → REFERENCE_READY` 확인
3. 그 뒤 waypoint가 들어와도 `relative_*_ref` 값이 유지되는지 확인
4. `wheel_pair_skew_s`가 일반 주행에서 50 ms 이하인지 rosbag으로 확인
5. lateral PID는 바퀴 공중시험, 무부하 바닥시험 후 하중시험 순으로 진행
