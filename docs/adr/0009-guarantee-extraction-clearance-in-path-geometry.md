---
status: accepted
---

# 출차 직선 clearance를 경로 geometry로 보장한다

출차 경로는 다음 순서로 구성한다.

```text
Registry final vehicle pose
  -> source staging pose
  -> extraction clear pose
  -> clear pose에서 시작하는 A* path
  -> waiting staging
  -> waiting pose
```

source staging은 기존 `make_approach_candidates()`와 loaded footprint를 사용하여 slot open boundary 밖으로 footprint 전체와 `slot_staging_gap_m`이 빠져나온 위치다. extraction clear pose는 이 staging에서 슬롯 바깥 방향, 즉 등록된 slot entry axis의 반대 방향으로 최소 `lookahead + extraction_safety_margin_m`만큼 더 연장한 점이다.

`final vehicle pose -> source staging -> extraction clear pose`는 같은 슬롯 축과 차량 yaw를 유지하는 하나의 직선 corridor다. Pure Pursuit가 clear pose보다 lookahead 거리 앞에서 다음 A* segment를 보기 시작하더라도 그 시점에는 loaded footprint가 slot open boundary 밖에 있어야 한다.

Fleet Manager는 기존 `_insertion_corridor_free`의 oriented-footprint 검사 또는 같은 기하 검사를 역방향 extraction corridor 전체에 적용한다. final vehicle pose부터 clear pose까지 각 swept footprint가 장애물과 map boundary를 침범하면 retrieve 계획을 승인하지 않는다. A*는 extraction clear pose를 시작점으로 계획하고, 최종 waypoint를 합칠 때 epsilon 이내로 같은 clear/A* 첫 점은 하나만 남겨 불연속과 zero-length segment를 만들지 않는다.

clearance 계산에 쓰는 lookahead는 고정 literal을 복제하지 않는다. 하나의 공유 launch/config 값이 Fleet의 extraction planning parameter와 `RigidBodySyncNode.lookahead`에 함께 전달되어야 하며, `slot_staging_gap_m`, 현재 loaded footprint와 별도 safety margin을 같이 사용한다.

이번 범위에서는 `RigidBodySyncNode`, `PurePursuit`와 lookahead 계산을 변경하지 않고 retrieve 전용 `EXTRACT_TO_CLEAR` FSM 또는 내부 subphase도 추가하지 않는다.
