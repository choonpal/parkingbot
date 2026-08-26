---
status: accepted
---

# OccupancyGrid 원점이 등록 지도 범위를 정의한다

차량·슬롯 pose를 옮겨 A* 경계 문제를 피하지 않고, 실제 자유공간을 포함한 `OccupancyGrid.info.origin`과 width/height를 지도 범위의 권위값으로 사용한다. A*, oriented footprint, corridor 및 source mask는 모두 같은 원점을 해석하며 맵 밖은 계속 점유로 처리한다. 현재 Gazebo 실증 layout은 확인된 waiting/home 쪽 자유공간을 나타내기 위해 기존 차량 `(0.6, 0.4)`과 슬롯 좌표를 유지한 채 `x=-0.4..4.4`, `y=-0.8..3.83`을 사용하고, 실차 값은 현장 측정·Homography 등록 결과로 확정해야 한다.
