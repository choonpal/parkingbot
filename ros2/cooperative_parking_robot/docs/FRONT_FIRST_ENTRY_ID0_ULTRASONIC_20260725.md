# Front-first 차량 하부 진입·정렬 구현 기준

최종 갱신: 2026-07-25

이 문서는 같은 차량 뒤쪽에서 두 로봇이 순차 진입하는 현재 구현의 기준이다.
과거의 “Rear 선정렬” 또는 “Front/Rear가 반대쪽에서 동시 진입” 설명보다 이
문서를 우선한다.

## 확정 구성

- 로봇 1대 외곽: 앞뒤 `0.565m`, 좌우 `0.275m`
- 기본 목표 차량 휠베이스: `0.70m`
- 정렬 후 로봇 몸체 사이 이론 간격: `0.70 - 0.565 = 0.135m`
- 코드가 허용하는 최소 몸체 간격: `0.10m`
- Front 상판 ID10: 천장 CCTV 절대 pose
- Rear 상판 ID11: 천장 CCTV 절대 pose
- Front 후면 ID0: Rear 전면 카메라 상대 pose

`0.70m`는 현재 로봇 길이를 만족하는 안전 기본값이다. 목표 모형차의 실제
앞·뒤 차축 중심 간 거리를 재서 `0.70m`와 다르면 모든 노드에 vehicle spec으로
같은 값을 전달해야 한다. `0.665m(0.565+0.10)` 미만은 현재 외곽과 최소 여유를
만족하지 않아 코드가 거부한다.

## 구현된 동작 순서

1. CCTV가 ID10/ID11을 보며 두 로봇의 절대 pose와 목표 차량 pose를 제공한다.
2. Front가 차량 뒤 종축 standoff에 선 뒤, Rear는 그보다 한 휠베이스 뒤의
   외부 ID0 관측 queue로 이동한다. 측면 staging은 사용하지 않는다.
3. Rear camera에서 ID0가 실제로 보일 때 Front가 차량 뒤쪽 중심선으로 먼저 진입한다.
4. Front 초음파가 첫 번째 wheel pair(rear axle)를 검출하면 통과 이벤트만
   기록하고 정지하지 않는다.
5. 약 한 휠베이스 뒤의 두 번째 wheel pair(front axle)를 검출해야 최종
   `wheel_center_s`를 발행한다. 다른 간격의 두 번째 물체는 축으로 인정하지 않는다.
6. Front 그리퍼 중심은 초음파가 계산한 front axle 중심에 정렬된다.
7. Front 정렬 완료 토픽을 받은 뒤에만 Rear가 같은 차량 뒤쪽으로 진입한다.
8. Rear는 첫 번째 wheel pair(rear axle)를 최종 축으로 선택한다.
9. Rear 그리퍼 중심도 초음파 중심에 정렬한 뒤 ID0 상대거리와 상대 yaw가
   허용범위인지 검증한다.
10. 두 로봇의 상태머신 배리어가 모두 완료된 뒤에만 동시에 인양한다.

## 센서 제어권

| 구간 | 종방향(축 중심) | 횡방향·yaw | 안전 검증 |
| --- | --- | --- | --- |
| 차량 외부 | CCTV pose/odom | CCTV pose/odom | 경로 envelope |
| Front 하부 진입 | 초음파 축 순서 | ID0 우선 보조, odom 유지 | ID0·상판 pose 유실시간 |
| Rear 하부 진입 | 초음파 첫 축 | ID0 상대 pose 보조 | ID0 거리 guard |
| 최종 축 정렬 | **초음파만 최종 종방향 제어** | 차량축 + ID0 yaw/lateral | ID0 거리/yaw 일치 검사 |
| 인양 후 주행 | 강체 경로제어 | ID0 거리/yaw 융합 | ID0 → ID10+ID11 → encoder 순 fallback |

ID0는 `CENTER_AXLE`에서 목표 종방향 위치를 명령하지 않는다. Rear의 coarse
scan을 휠베이스 부근에서 감속하지만, 초음파가 바퀴의 뒤 에지까지 지나 중심을
계산할 수 있도록 계속 전진한다. 초음파 중심과 ID0 예상거리의 결과가 다르면
둘을 섞어 중간 위치로 이동하지 않고 정지·오류 처리한다.

## ID0 또는 상판 마커가 가려질 때

- ID0가 신선하면 ID0 상대 pose를 사용한다.
- 주행 중 ID0가 없지만 ID10과 ID11이 둘 다 신선하면 두 절대 pose의 상대값을
  사용한다.
- 차량 하부에서 사용할 수 있는 영상 pose가 모두 사라지면 짧게 encoder를
  유지하고, `0.75s` 후 `35%`로 감속하며, `1.50s` 후 정지한다.
- Rear 최종 정렬 때 ID0가 없거나 거리/yaw가 계속 불일치하면 인양하지 않는다.

## 2026-07-25 재검토 결론

핵심 동작 순서는 논리적으로 연결된다. 즉, `같은 차량 뒤쪽 종축 대기 → Front
첫 축 통과/둘째 축 정렬 → Rear 첫 축 정렬 → ID0 최종 검사 → 양쪽 인양 배리어
→ 결합 footprint A* → 강체 주행`에는 순서 역전이나 초음파·ID0의 종방향
제어권 충돌이 없다.

다만 현재 상태를 **“오류 없음”으로 판정할 수는 없다.** 정적 단위시험으로 잡히지
않는 아래 세 가지 코드 안전 공백이 있고, 전체 프로젝트 설명과 현재 1카메라·1대
시연 코드 사이에도 범위 차이가 있다.

| 검토 항목 | 판정 | 근거 |
| --- | --- | --- |
| Front 먼저 종방향 진입 | 일치 | Front는 두 번째 축, Rear는 첫 번째 축만 최종 선택 |
| 최종 종방향 제어권 | 일치 | `wheel_center_s`만 목표 위치를 만들고 ID0 거리는 감속·검증에만 사용 |
| 인양 순서 | 일치 | 양쪽 `LIFT READY → COMMIT` 뒤에만 서보 명령 |
| 적재 A* | 조건부 일치 | 원점 `(0,0)`, yaw `0`, 정적 장애물, 고정 적재 yaw 범위에서는 일치 |
| 인양 후 절대 yaw | 보강 필요 | ID0는 상대 yaw만 보며, 현 차량 CCTV 피드백은 절대 yaw를 제공하지 않음 |
| 천장 CCTV 수량 | 설명 불일치 | 인수인계는 2대지만 현재 launch·보정·융합 코드는 1스트림 기준 |

## P1 — 실물 인양 전 코드에서 반드시 보강할 항목

### P1-A. 상판 마커와 ID0 데이터의 신선도 계약

`individual_move_node`는 상판 마커 Bool의 마지막 값이 `True`이면 수신이 끊긴
뒤에도 계속 상판 pose가 사용 가능한 것으로 판단한다. 이 경우 의도한
`ID0 → 제한된 encoder → 감속/정지` 전환이 시작되지 않는다. 또한 이 노드의
`/sync/relative_pose` 콜백은 `header.stamp`와 `frame_id`를 검사하지 않아 지연된
ID0 pose를 새 측정처럼 받을 수 있다. `rigid_body_sync_node`는 ID0 stamp는
검사하지만 `rear_base` frame 계약은 확인하지 않는다.

필수 수정 기준은 다음과 같다.

- 상판 marker Bool 또는 실제 CCTV pose의 **마지막 수신 시각**을 저장하고
  `cctv_marker_timeout_s`를 넘으면 즉시 unusable로 바꾼다.
- ID0 pose는 `frame_id == rear_base`, 유효한 quaternion, source stamp의
  stale/future/duplicate 검사를 통과해야 한다.
- source stamp 검사는 시계 동기화된 ROS 시간으로 하고, 통신 단절 timeout은
  각 장비의 monotonic 수신시각으로 별도 판단한다.
- 회귀시험에서 마지막 `True` 뒤 발행 노드를 중단했을 때 감속 후 정지해야 한다.

### P1-B. 최종 ID0 검사에 상대 횡오차 추가

현재 `final_relative_check()`는 `상대거리≈wheelbase`와 `상대 yaw`만 확인하고
ID0의 `relative_y`는 확인하지 않는다. 각 로봇의 차량 중심선 오차를 따로
검사하더라도 최종 인양 직전에는 두 로봇 사이의 직접 횡오차를 한 번 더 막는
편이 맞다.

- `abs(relative_y) <= relative_lateral_tolerance_m`을 최종 조건에 추가한다.
- 허용값은 임의로 확정하지 말고 그리퍼 좌우 유격과 ID0 반복 측정 표준편차로
  정한다. 초깃값 후보는 `0.02~0.03m`지만 실측값이 우선이다.
- 거리·yaw·횡오차 중 하나라도 timeout 동안 벗어나면 인양하지 않는다.

### P1-C. 초음파 축 후보의 절대 위치 plausibility gate

현재 Front는 첫 번째 좌우 동시 물체를 rear axle로 저장한 뒤, 그 지점에서 약
한 휠베이스 떨어진 두 번째 물체만 검사한다. Rear는 첫 번째 좌우 동시 물체를
바로 rear axle로 인정한다. 따라서 차체 구조물이나 다른 대칭 반사체가 먼저
잡히면 잘못된 축 순서를 만들 수 있다.

- latched vehicle frame에서 첫 축은 `s≈-wheelbase/2`, 둘째 축은
  `s≈+wheelbase/2`인 예상 창 안에 있을 때만 axle 후보로 인정한다.
- YOLO 차량 중심 오차가 있으므로 창 폭은 CCTV 반복 오차를 포함해 실측하고,
  간격 검사도 함께 유지한다.
- 가짜 좌우 반사체 → 실제 rear axle → 실제 front axle 순서의 시험을 추가한다.

## P1 — 코드 수정 또는 시연 범위 제한 중 하나를 반드시 선택할 항목

### 인양 후 공통 절대 yaw

ID0는 Front와 Rear가 서로 얼마나 틀어졌는지는 알지만, 두 로봇이 같이 같은
방향으로 틀어진 공통 yaw 오차는 알 수 없다. ID10/ID11이 차량에 가려진 동안
현재 `/parking/vehicle_pose_feedback`은 위치 `x,y`만 주며 yaw는 항상 0으로
발행되고, 강체 제어기도 그 차량 yaw를 보정에 사용하지 않는다.

다음 중 하나를 선택해야 한다.

1. 차량 segmentation/OBB 또는 외부에서 계속 보이는 마커로 운반 차량의 절대
   yaw를 구해 `base_virtual` yaw에 융합한다.
2. 짧은 고정-yaw 저속 경로에서 공통 yaw 드리프트가 허용치 이내임을 실측하고,
   그 범위만 시연·보고서의 현재 한계로 명시한다.

### 2대 천장 CCTV와 현재 1스트림 코드

현재 `cctv_server.launch.py`, `yolo_bev_map_node`,
`cctv_robot_marker_node`는 카메라 하나의 rectified 영상과 homography 하나만
사용한다. 최종 구성에서 천장 CCTV 2대를 유지한다면 카메라마다 calibration과
`H1/H2`를 두고 공통 world 좌표에서 검출·마커 관측을 중복 제거/융합해야 한다.
이번 시연을 1대로 제한할 경우에는 BOM·보고서·구성도도 1대로 맞춰야 한다.

### 동적 장애물과 재계획 주장

현재 A*는 인양 직후 한 번 계획하고 `fleet_manager_node`의 `NAVIGATING` 상태는
재계획을 수행하지 않는다. 따라서 사람이나 물체가 경로에 새로 들어오는 환경을
지원한다고 쓰면 코드와 불일치한다. 현 시연은 통제된 정적 구역으로 제한하고
물리 E-stop을 둔다. 동적 환경을 주장하려면 맵 변경·경로 이탈 감지와 정지 후
재계획을 구현해야 한다.

## 현재 시연에서는 허용되지만 범위를 넓히면 수정할 항목

| 현재 구현 | 현재 허용 범위 | 확장 시 필요한 수정 |
| --- | --- | --- |
| 고정 wheelbase `0.70m` | 검증된 모형차 한 종류 | mission 시작 때 제원을 latch하고 차종 DB 검증 |
| `cars_in_slots[0]`을 운반 차량 피드백으로 선택 | 영상에 차량 한 대만 있는 시연 | 지속 ID/IoU/Kalman 데이터 연관 및 운반 차량 self-mask |
| `WAIT_TARGET → ... → NAVIGATING` 1회 | 한 사이클 실행 | fleet·YOLO target latch·sequence 전체 reset |
| 접근/정렬 완료가 Bool 토픽 | 한 사이클, 노드 재시작 없음 | approach/align도 mission ID·sequence 이벤트로 변경 |
| A*가 OccupancyGrid origin을 사용하지 않음 | origin `(0,0)`, yaw `0`인 현재 map | non-zero origin 좌표변환 구현 |
| 축 정렬 직사각형 footprint | 차량·슬롯 yaw `0`, 운반 yaw 고정 | 회전 footprint/SE(2) 계획 또는 방향별 보수 팽창 |
| `GRIP_DONE`이 서보 명령각 도달만 의미 | 감독·보호지그가 있는 모형 시연 | 전류·리미트·하중 센서로 실제 파지 확인 |

현재 launch는 분류 결과와 관계없이 `use_fixed_wheelbase=true`로 `0.70m`를
사용한다. 따라서 현 보고서에는 “차종 분류로 여러 휠베이스를 자동 적용했다”고
쓰지 않고, **고정 차종 시연**이라고 적어야 한다.

## 실물시험 전에 필수로 해야 하는 일

1. 목표 모형차의 실제 휠베이스를 차축 중심끼리 측정한다.
2. 차량 전체 앞뒤·좌우 외곽을 측정해 현재 임시값 `0.90m × 0.35m`를 교체한다.
3. 로봇 `0.565m × 0.275m`가 프레임만이 아니라 카메라·마커·그리퍼·배선
   돌출부까지 포함한 최대 외곽인지 확인한다. 현재 몸체 사이 이론 여유는
   `0.135m`뿐이다.
4. ID0와 Rear 카메라를 실제 위치에 장착한 뒤, 두 로봇이 정확히 축 중심에 있을
   때 `중심거리 - raw camera-to-marker 거리`를 재서
   `aruco_distance_offset_m`을 보정한다. 현재 `0.565m`는 두 센서가 로봇
   끝면에 정확히 있다는 가정값이다.
5. Rear 카메라 intrinsic/distortion과 ID0 실제 검은 정사각형 크기를 보정한다.
6. Front/Rear 초음파 센서 중심과 그리퍼 중심의 X offset을 각각 실측한다.
7. ID0 광학 경로를 실제 하부에서 확인한다. 현재 기하 가정이면 raw 광학 거리는
   초기 queue·최종 정렬에서 약 `0.135m`, Front만 front axle까지 들어간 순간
   약 `1.335m`이므로 이 전 구간에서 초점·화각·조명·가림이 모두 허용돼야 한다.
8. ID10/ID11의 yaw offset, 마커 높이, base 중심 offset과 천장 parallax를
   각각 보정한다.
9. 차량이 map `+x` 종축과 평행하게 들어오도록 기계 가이드 또는 yaw 검출을
   둔다. 현재 YOLO target pose와 슬롯 yaw는 0으로 고정돼 있다.
10. Homography 출력, 대기영역, 슬롯, A* map이 모두 metre 단위의 같은
    `(origin=0, yaw=0)` 좌표계인지 확인한다.
11. 실제 CubeMX `.ioc`를 생성해 TIM1 PWM, TIM2~5 encoder, TIM9 1MHz,
    TIM10/11 servo, USART2, HC-SR04 EXTI 핀을 펌웨어 기대값과 맞춘다. 현재
    ZIP에는 `.ioc`와 HAL 생성 프로젝트가 없으므로 C 소스만 바로 플래시할 수 없다.
12. encoder PPR·부호, 유효 wheel radius, `lx/ly`, PID/PWM, servo 각도와
    HC-SR04 5V ECHO 레벨을 잭업 상태에서 확인한다.
13. Jetson/RPi 시계를 NTP/Chrony로 동기화하고, 실차 분산 실행에서는
    `cctv_server.launch.py`, `front_robot.launch.py`, `rear_robot.launch.py`를
    사용한다. `full_system.launch.py` 기본값은 영상·상판 마커·Rear ID0가 꺼진
    한-PC smoke 모드다.
14. E-stop을 누를 수 있는 저속 실물 시험에서 Front 1축 통과 → 2축 정렬 →
    Rear 1축 정렬 → ID0 최종 검사 순서를 rosbag으로 확인한다.

## 선택 조정 사항

- `axle_spacing_tolerance_m`, 새 axle 위치 창, 초음파 threshold는 타이어 폭·차체
  반사·센서 노이즈 결과에 따라 조정한다.
- `rear_aruco_slowdown_window_m`, scan 속도와 marker loss 시간은 저속 시험 후
  보수적으로 조정한다.
- ID0 가림이 반복되면 마커 크기·조명·카메라 위치를 바꾸거나 보조 마커를
  추가하되, ID 충돌 없이 별도 관측으로 처리한다.
- 두 로봇의 서보 시작 편차가 실제 차체 기울임을 만든다면 `COMMIT`에 미래 실행
  시각을 넣어 동시 시작을 예약한다.

## 맵·A* 결과

기본 적재 footprint는
`max(0.90, 0.70+0.565)+2×0.06 = 1.385m` ×
`max(0.35, 0.275)+2×0.06 = 0.47m`다. 기존 `6m × 4m` 맵 자체는 사용할 수
있다. Rear queue까지 맵 안에 두기 위해 기본 차량 대기 중심은 `(2.3, 0.6)`,
Front/Rear 시작점은 각각 `(1.15, 0.6)`, `(0.45, 0.6)`으로 종축 배치했다.
실제 장애물·통로·슬롯 폭을 측정한 뒤 A* 경로 존재 여부를 다시 시험해야
한다.

## 테스트

WSL/Ubuntu에서 다음 명령으로 실행한다.

```bash
python3 -m pytest -q
```

회귀 테스트는 같은 쪽 진입, Front 2번째/Rear 1번째 축 선택, `0.70m` 간격,
초음파 종방향 제어권, ID0/ID10/ID11 fallback, 직사각형 footprint A*를 포함한다.

최종 정적 검증 결과: Python `compileall` 통과, `pytest` **89개 전체 통과**.
이 통과는 현재 구현된 계약의 회귀 결과이며, 위 P1-A~C와 공통 절대 yaw가
이미 해결됐다는 뜻은 아니다. 카메라·초음파·모터가 연결된 HIL/실물 주행
검증은 별도 필수 작업이다.
