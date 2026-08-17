# 결합 직사각형 footprint A* 수정 기록 (2026-07-25)

## 확정된 기하 전제

- 로봇 1대 외곽: 차량 앞뒤 방향 `0.565m`, 차량 좌우 방향 `0.275m`
- Front/Rear 로봇의 그리퍼 중심과 회전 중심은 각각 차량 앞축·뒤축 중심과 일치
- 운반 중 `base_virtual` yaw 고정
- Front/Rear 중심 간 목표거리는 목표 차량 휠베이스

## Mission footprint

```text
length = max(vehicle_length, wheelbase + robot_length)
         + 2 × safety_margin

width  = max(vehicle_width, robot_width)
         + 2 × safety_margin
```

안전 기본 휠베이스 `0.70m`, 임시 차량 외곽 `0.90m × 0.35m`, 여유
`0.06m`를 적용하면 `1.385m × 0.47m`다. 5cm 격자에서 반치수는
앞뒤 14칸, 좌우 5칸이다. 휠베이스 0.70m는 로봇 몸체 사이 0.135m를
남기지만, 차량 길이·폭은 실측값으로 교체해야 한다.

## A* 변경

- 고정 4칸 원형 팽창을 mission별 축 정렬 직사각형 팽창으로 교체
- OccupancyGrid의 `-1` 미확인 셀을 점유로 처리
- footprint 일부가 맵 밖으로 나가는 중심 셀 차단
- 대각선 이동 시 양옆 직교 셀이 모두 자유공간이어야 통과
- A* 시작점은 고정 대기좌표가 아닌 Front/Rear 최신 odometry 중점 사용
- 시작점 또는 목표점이 팽창맵에서 막히면 경로 생성 거부

## 남은 실기 작업

- `config/parking_layout.yaml`의 차량 길이·폭 placeholder와 실측 휠베이스 확인
- 실제 벽·기둥·검출 차량 외곽을 OccupancyGrid에 반영
- 조립 후 전체 외곽과 안전여유 실측
- Pure Pursuit의 코너 추종 편차가 팽창 여유 안에 드는지 저속시험

## 검증

- 직사각형 팽창
- 맵 경계 차단
- 미확인 셀 차단
- 대각선 corner cutting 차단
- 6m × 4m 빈 맵에서 기본 대기 위치→첫 슬롯 경로 생성
- 전체 Python 컴파일 및 최신 `pytest` 결과는
  `FRONT_FIRST_ENTRY_ID0_ULTRASONIC_20260725.md`를 따른다.
