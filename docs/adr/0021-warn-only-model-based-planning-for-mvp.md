---
status: accepted
amends: 0012-preflight-retrieve-robot-approach-corridors, 0016-reserve-park-slot-after-successful-planning
---

# MVP에서는 모델 기반 계획 사전검사를 경고로 운영한다

현재 등록 슬롯은 입구가 열린 물리 구조지만 기존 slot-fit은
Front+차량+Rear의 loaded footprint 전체가 닫힌 슬롯 직사각형 안에 들어가야
한다고 가정했다. P2의 차량 자체는 들어가지만 loaded footprint는 슬롯보다
길어 `SLOT_TOO_SHORT`가 되었고, Fleet는 요청을 이미 승인한 뒤 `PLAN_PATH`에서
경로를 발행하지 않은 채 반복 재계획했다. UI에는 원인이 나타나지 않았다.

Fleet에 `planning_validation_mode=ENFORCE|WARN_ONLY` 계약을 둔다. 이번 MVP의
실차와 Gazebo launch 기본값은 `WARN_ONLY`다. 다음 모델 기반 검사는 계속
계산하고 안정적인 warning code와 ROS warning을 남기되, 기존 알고리즘이
실행 가능한 경로를 만들 수 있으면 후보를 버리지 않는다.

- loaded-footprint slot fit
- retrieve 로봇 접근 corridor와 예측 inter-robot clearance
- park staging 회전 공간과 insertion corridor
- retrieve extraction corridor
- waiting staging 회전 공간과 insertion corridor

`WARN_ONLY`는 검사를 삭제하거나 장애물을 free로 다시 쓰지 않는다. A*,
기존 APPROACH/ALIGN/LIFT, Pure Pursuit와 post-release egress 알고리즘도
변경하지 않는다.

다음 조건은 planning warning이 아니므로 계속 차단한다.

- A*가 waypoint를 전혀 만들지 못함
- map, fresh odometry, vehicle record 또는 mission command가 없음
- Registry lifecycle, mission ID, 차량 인증 또는 coordination correlation 불일치
- ROS 명령 publish 실패
- 사용자 E-stop, stale/NaN command, STM32 watchdog
- 인양 중 실제 Front/Rear 거리·yaw 한계와 하드웨어 fault

Fleet는 기존 `/fleet/state` optional field에
`planning_validation_mode`, `validation_warnings`,
`planning_blocker`를 제공한다. UI는 경고 운행과 실제 경로 생성 불가를
즉시 구분해 표시한다. 새 ROS topic은 추가하지 않는다.

향후 현장 치수와 장애물 모델이 검증되면 launch override로 `ENFORCE`를
선택할 수 있다.
