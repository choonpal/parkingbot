---
status: accepted
---

# 첫 출차 통합에서는 신규 차량을 전진 주차한다

첫 출차 통합 범위에서는 모든 신규 입차 차량의 기본 운용 `parking_direction`을 `forward`로 고정한다. 로직 내부에 방향을 하드코딩하지 않고 기존 parameter 구조를 유지한 채 launch 및 운용 설정의 기본값으로 강제한다. 이렇게 하면 차량 뒤쪽이 통로를 향하므로 기존 `vehicle_entry`의 후방 진입 알고리즘을 출차 접근에 재사용할 수 있다.

Parking Registry는 각 차량-슬롯 기록에 `FORWARD`, `REVERSE`, `UNKNOWN` 주차 방향을 저장한다. 현재 세션에서 완료된 신규 입차는 `FORWARD`로 기록한다. 방향을 신뢰할 수 없는 기존 점유 차량은 `UNKNOWN`으로 기록하고 Fleet Manager가 자동 출차를 허용하지 않는다. 후진 주차 차량을 위한 새로운 접근 알고리즘은 이번 범위에서 만들지 않는다.

UI는 주차 방향이나 내부 불가 사유를 노출하지 않는다. 다만 Fleet Manager가 지원 방향과 필수 Registry record 존재 여부로 계산한 `retrievable` boolean을 사용해 슬롯의 출차 동작 활성 여부를 표시한다. 최종 허용 여부는 실제 요청 시 Fleet가 다시 검증한다.
