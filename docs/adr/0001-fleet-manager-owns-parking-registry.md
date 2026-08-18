---
status: accepted
---

# Fleet Manager가 Parking Registry를 소유한다

현재의 단일 활성 미션 구조에서는 Fleet Manager 내부에 세션 한정 Parking Registry를 두고, 슬롯 운영 상태와 차량-슬롯 기록의 단일 writer로 사용한다. Perception/CCTV는 `observed`, `occupied`, fresh live pose, 차량 detection 같은 물리 관측만 제공하며 운영 상태를 변경하지 않는다. UI는 Fleet Manager가 제공하는 Parking Registry 상태를 표시하고 출차 의도만 전달한다.

슬롯 운영 상태는 최소한 `EMPTY -> RESERVED -> OCCUPIED -> EXIT_RESERVED -> EXITING -> EMPTY` 전이를 지원한다. 활성 미션 reset은 Parking Registry의 `OCCUPIED` 기록을 삭제하지 않는다.

별도 Registry node와 디스크 영속화는 현재 범위에서 제외한다. 이번 실증에서는 모든 슬롯을 물리적으로 비운 뒤 Fleet를 시작해야 한다. 차량이 주차된 뒤의 Fleet 단독 재시작은 지원하지 않으며, 재시작이 필요하면 실증을 중단하고 모든 차량을 제거한 뒤 전체 시스템을 `EMPTY`로 다시 시작한다.
