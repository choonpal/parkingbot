---
status: accepted
---

# 출차 시작 전에 양쪽 로봇의 직선 접근 corridor를 검증한다

Fleet Manager는 retrieve mission의 `WAIT_LIFT`를 활성화하기 전에 Front/Rear의 fresh한 현재 odometry와 선택 차량의 Registry final pose 및 vehicle spec으로 기존 rear-side staging 두 점을 계산한다. Front current pose에서 Front staging, Rear current pose에서 Rear staging까지의 직선 접근만 허용한다.

각 route는 map boundary, OccupancyGrid obstacle, 다른 parked vehicle과 oriented robot footprint를 기준으로 검사한다. 막힌 route에는 A* 우회를 만들지 않고 retrieve 요청을 시작하지 않는다. 개별 navigation topic이나 retrieve 전용 planner를 추가하지 않으며 `IndividualMoveNode`의 실제 직선 접근 동작도 변경하지 않는다. 이 preflight는 retrieve 시작 조건에만 추가하고 기존 park 접근은 변경하지 않는다.

`simultaneous_entry=true`에서는 두 선분이 기하학적으로 교차한다는 이유만으로 거부하지 않는다. 기존 접근 속도로 두 로봇의 진행을 시간 매개화하고 각 시점의 oriented robot footprint 또는 center/body clearance가 실제 `minimum_inter_robot_gap_m`을 위반할 때만 동시 접근 불가로 판단한다.

`simultaneous_entry=false`에서는 기존 coordination 순서 그대로 Front 이동 중 Rear를 HOME에 고정하고, Front가 staging에 도착한 뒤 Front를 고정한 상태에서 Rear route를 시간 샘플링한다. 두 phase 모두 같은 oriented footprint와 최소 간격을 검사한다.

구현은 TDD로 다음 경계를 고정한다.

- route가 교차하지만 통과 시점이 달라 충분한 clearance가 있으면 허용한다.
- 같은 시점의 body clearance가 최소 간격을 위반하면 거부한다.
- wall, map boundary, parked vehicle에 막힌 개별 route를 거부한다.
- 현재 예시 HOME/slot geometry의 P1~P4를 동시/순차 정책 모두 회귀 테스트한다.

검증 결과 P1~P4 모두 동시 정책에서는 clearance를 위반하고 기존 Front-first 순차 정책에서는 통과했다. 이에 따라 ADR 0017에서 실차 기본값을 `false`로 변경했다.
