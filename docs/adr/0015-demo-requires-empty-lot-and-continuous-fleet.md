---
status: accepted
---

# 실증은 빈 주차장과 연속된 Fleet 프로세스를 요구한다

이번 실증은 모든 등록 parking slot이 물리적으로 `EMPTY`인 상태에서 시작한다. Fleet Manager와 세션 한정 Parking Registry는 시작 시 모든 등록 slot을 `EMPTY`로 초기화한다. 이후 `OCCUPIED`가 되는 차량은 반드시 같은 Fleet 프로세스가 수행한 park mission으로 주차된 차량뿐이며 사람이 차량을 움직이지 않는다.

park mission의 matching RETURN commit에서 저장한 `slot_id`, final vehicle pose, parking direction, vehicle spec과 `OCCUPIED` lifecycle은 active mission reset 및 양쪽 HOME 복귀 뒤에도 유지한다. Web UI가 재시작되면 새 `client_id`로 요청 sequence를 다시 시작하고 `/fleet/state`의 Registry summary를 받아 슬롯 표시를 복원한다.

Registry startup 복구를 위한 `/parking/slot_observations` topic, Perception 관측 기반 자동 재구성과 디스크 persistence는 이번 범위에서 구현하지 않는다. Web UI 재시작은 지원하지만 차량이 주차된 상태의 Fleet Manager 단독 재시작은 지원하지 않는다.

> Fleet restart after a vehicle has been parked is not supported in this demo configuration.

Fleet 재시작이 필요하면 실증을 중단하고 실제 주차장의 모든 차량을 제거한 뒤 전체 시스템을 빈 초기 상태로 다시 시작한다. 실제 운영 단계의 process restart recovery는 물리 관측 기반 reconciliation 또는 Registry persistence 기능으로 별도 설계한다.
