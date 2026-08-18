---
status: accepted
---

# Registry 요약을 기존 fleet state로 제공한다

Parking Registry 요약은 새 topic을 만들지 않고 기존 `/fleet/state` JSON의 optional `parking_slots` field로 제공한다. 기존 `empty_count`는 하위 호환을 위해 유지하고 Robot FSM이 사용하지 않는 optional field 추가로 기존 park 동작이 바뀌지 않도록 테스트한다.

각 slot summary는 UI에 필요한 `slot_id`, `lifecycle`, `retrievable`만 포함한다. vehicle pose, vehicle spec과 `parking_direction`은 노출하지 않는다. lifecycle의 authoritative owner는 계속 Fleet 내부 Parking Registry다.

`retrievable=true`는 최소한 record가 `OCCUPIED`이고 이번 통합이 지원하는 주차 방향이며 유효한 final vehicle pose, vehicle spec, 차량번호와 비밀번호 검증값을 모두 가진 경우에만 가능하다. 이는 Registry 기반의 가벼운 UI 사전 자격이며 차량번호나 검증값 자체는 summary에 노출하지 않는다. active mission 유무와 현재 Fleet state는 UI의 전체 출차 동작 gate에 별도로 사용한다.

실제 retrieve 요청에서는 Fleet가 active mission, mission ID, 현재 slot lifecycle과 source-staging 접근 corridor 등 실행 시점 조건을 다시 검증한다. odometry와 map에 따라 달라지는 무거운 corridor 계산은 주기적인 `/fleet/state` 생성에서 하지 않고 실제 승인 경로에서만 수행한다.

Fleet는 lifecycle, 요청 승인·거부처럼 UI에 즉시 보여야 할 상태가 바뀌면 기존 1 Hz timer를 기다리지 않고 `/fleet/state`를 즉시 한 번 발행한다.
