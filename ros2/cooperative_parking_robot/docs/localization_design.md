# 실기체 localization 설계 — encoder_odometry + Kalman (PoseEKF)

> **Humble 이식 주의:** 이 문서 안의 Ubuntu 24.04/OpenCV 4.6 표현은 이전 정적 검증 환경의 기록이다. 현재 배포 대상은 Ubuntu 22.04 + ROS 2 Humble이며, 신형/legacy ArUco API 호환 코드는 유지했다. 이전 기록이 Humble 실기 통합 검증을 의미하지는 않는다.

> **최신 결정(v2.2):** Front 상판 ID10과 Rear 상판 ID11은 천장 절대 pose,
> Front 후면 ID0은 Rear 전면 카메라의 상대 pose에 사용한다. 차량 하부에서
> 상판 마커가 가려지면 ID0로 상대 yaw/거리를 유지하고, 둘 다 가려지면 제한된
> encoder fallback 뒤 정지한다.
> v1.9 — 2026-07-20 재검증에서 yaw 좌표계, OpenCV 4.6 호환성,
> 토픽 순서 경쟁, launch 누락, 이상치 강제 재획득 안전장치를 보완했다.
> 코드 경로와 비ROS 단위 테스트는 통과했지만 homography·카메라 calibration,
> 마커 부착각·높이 등 실측값은 아직 없다 — **실측 전엔 정확도 보장 안 됨**.
> 관련 코드: `encoder_odometry.py`, `kalman_filter.py`(PoseEKF), `pose_fusion_node.py`,
> `cctv_robot_marker_node.py`, `aruco_utils.py`(신규), `stm32_bridge_node.py`(수정),
> `aruco_tracker_node.py`(수정, yaw축 버그),
> `rigid_body_sync_node.py`(수정, 초기화 레이스), `stm32_firmware/.../parking_robot_firmware.c`(수정, 엔코더 rollover),
> `front_robot.launch.py` / `rear_robot.launch.py` / `full_system.launch.py` / `cctv_server.launch.py`(수정),
> `test/test_localization_math.py`, `docs/localization_validation_report.md`
> 이 문서는 `system_spec_v15_reviewed.md`를 대체하지 않는다 — 그 문서의 몇 가지
> 서술을 이 설계가 바꾸므로, 뒷부분 "system_spec 대비 변경점"에 반영 필요 항목을 정리했다.

---

## 1. 왜 이 작업이 필요했나 — 기존 코드에서 찾은 공백

`rigid_body_sync_node.py`를 읽어보면 절대 위치/자세 보정이 실제로 어디에
적용되는지가 생각보다 좁다.

```
가상 강체 중심 heading:
  theta = atan2(front.y - rear.y, front.x - rear.x)   ← rigid_body_kinematics.virtual_pose()
  이 theta로 Pure Pursuit·FINAL_APPROACH의 world↔body 변환을 전부 수행한다.

front.x, front.y, front.theta / rear.x, rear.y, rear.theta
  ← /front/odom, /rear/odom을 그대로 읽음 (stm32_bridge → EncoderOdometry, 순수 dead-reckoning)

보정이 적용되는 곳:
  ① dist_kalman, yaw_kalman (ScalarKalman) — Front-Rear "상대" 거리/yaw 차이만 보정.
     입력은 enc_dist = hypot(front.x-rear.x, ...)와 front.theta-rear.theta.
     → theta(가상중심 heading) 자체에는 반영되지 않는다.
  ② cctv_offset — vehicle_pose_feedback으로 가상중심 (cx,cy)에 오프셋만 더함.
     → theta에는 영향 없음. x,y도 "가상중심 좌표"에만 적용되고
       front.x/y, rear.x/y 개별값은 그대로 드리프트된 상태로 kinematics.split()에
       계속 사용된다.
```

결론: **개별 로봇의 절대 x, y, yaw 중 어느 것도 보정되는 경로가 없다.**
enc_dist·enc_yaw(상대차)만 ArUco로 보정되고, 가상중심 위치만 CCTV로 스무딩될 뿐,
Pure Pursuit이 실제로 세상 좌표계에서 어디를 보고 있는지(theta)는 순수
엔코더 드리프트에 그대로 노출돼 있다. 시뮬에서는 `/front/odom`, `/rear/odom`이
Gazebo 물리엔진의 참값이라 이 공백이 드러나지 않았다.

최신 결정은 Front 상판의 천장 관측용 ArUco로 Front의 절대 yaw를 얻고,
Rear는 Rear 전면 카메라가 관측한 Front 후면 ID0 상대 yaw를 결합해 Front와
yaw축을 맞추는 것이다. Rear 절대 위치는 천장 객체 추적을 사용할 수 있지만,
Rear 자체의 천장용 ArUco pose는 사용하지 않는다.

---

## 2. 센서 구성 (확정된 결정)

| 마커 | 부착 위치 | 관측 카메라 | 제공 정보 |
|------|-----------|-------------|-----------|
| Front 상판 마커 (신규) | Front 로봇 상단 평면(상방), **앞쪽(+x) 끝** | Jetson 천장 CCTV | Front 로봇의 절대 (x, y, yaw) |
| Front 후면 마커 ID0 (기존) | Front 로봇 후면, 측면 부착 | Rear 로봇 전방 카메라 | Front-Rear **상대** 거리/yaw (단방향) |

**Front 상판 마커 배치:** 차량 밑 삽입 후에도 천장에서 보이도록 Front의 앞쪽(+x)
노출부에 둔다. 실제 차체와 로봇 치수로 가시 구간을 확인해야 한다.

> ⚠️ **마커 중심 ≠ 로봇 회전중심.** 바깥끝에 붙였으므로 마커 중심은 base_link에서 진행축
> (+x)으로 `marker_offset_x`만큼 떨어져 있다. CCTV가 읽는 것은 마커 pose이므로,
> `cctv_robot_marker_node`가 이 오프셋을 로봇 yaw로 회전시켜 빼서 **base_link pose로
> 환산한 뒤** `/{role}/cctv_pose`를 발행한다 (`aruco_utils.marker_center_to_base_link`).
> 이 환산을 빠뜨리면 `rigid_body_sync_node`의 가상중심·self-mask·강체 간격이 전부
> `marker_offset_x`만큼 틀어진다. `front_marker_offset_x_m`의 기본값 0.0은
> 마커가 중심에 있는 기존 동작과 동일하며, 실제 장착 후 실측값을 넣는다.

차량(들어올리는 대상)에는 마커를 붙이지 않는다 — 실제 배포 시 임의의
고객 차량에는 마커 부착이 불가능하므로, 차량 인지는 계속 YOLO 세그멘테이션
경로를 쓴다(이 결정은 바뀌지 않음).

기존 Front 후면 ID0 마커(Rear 카메라용)는 **제거하지 않고 그대로 유지**한다.
이유:
- CCTV는 프레임레이트가 낮고(Jetson 처리 부하 포함 10~30Hz 추정) WiFi 전송 지연도 있다.
  그립 직전 정밀 정렬(FINAL_APPROACH, 허용오차 2cm/3°) 구간에서는 Rear 카메라의
  근거리 고정밀 측정이 CCTV보다 유리할 가능성이 높다 — 실측 전엔 단정 불가.
- 두 소스가 서로 다른 오차 특성(CCTV: homography 왜곡, ArUco 측면: 근접 시 정확)을
  가지므로 교차검증 용도로도 유지 가치가 있다.
→ 즉 기존 `rigid_body_sync_node`의 상대오차 보정 레이어는 **그대로 두고**,
  그 앞단에 로봇별 절대 pose 보정 레이어를 새로 추가하는 구조로 설계했다.

---

## 3. 데이터 흐름 (변경 후)

```
STM32 "E,fl,fr,rl,rr"
        │
        ▼
stm32_bridge_node ── EncoderOdometry.update()
        │                 dead-reckoning 누적치(x,y,theta) → 진단용 pose 필드
        │                 이번 주기 body-frame 델타(dx_body,dy_body,dtheta) → twist 필드
        ▼
  /{role}/wheel_odom (Odometry)              ← v1.6에서 이름 변경 (구 /{role}/odom)
        │
        ▼
pose_fusion_node ── PoseEKF
   predict(dx_body, dy_body, dtheta, dt)  ← wheel_odom 도착마다
   correct(x, y, yaw, R)                  ← /{role}/cctv_pose 도착 시 (신선하고 게이트 통과 시만)
        │
        ▼
  /{role}/odom (Odometry)                    ← rigid_body_sync_node가 구독 (기존과 동일 토픽명/타입)
        │
        ▼
rigid_body_sync_node
   virtual_pose(front, rear) → 이제 절대보정된 front/rear로 theta 계산
   dist_kalman / yaw_kalman  → 기존처럼 ArUco 측면 상대오차 추가 보정
   cctv_offset               → 기존처럼 가상중심에 추가 스무딩 (중복 보정이지만 저주파라 안정성 위주로 유지)
```

`kalman_filter.ScalarKalman`의 역할은 유지했다. 다만 재검증에서
`rigid_body_sync_node.py`에는 측정 timestamp·frame 신선도 검사를,
`aruco_tracker_node.py`에는 yaw 좌표계 수정과 calibration fail-closed,
OpenCV 버전 호환 처리를 추가했다. `/{role}/odom`의 최종 발행자는
stm32_bridge가 아니라 pose_fusion_node라는 인터페이스는 유지된다.

### cctv_robot_marker_node (v1.8에서 구현 완료)

`/front/cctv_pose`(PoseStamped, frame_id='map'), `/front/cctv_marker_visible`(Bool)을
발행하는 Jetson 측 노드. `yolo_bev_map_node`와 같은 왜곡 보정 `/cctv/image_rect` 피드를 구독하고,
**같은 `homography_rectified.npy`**를 재사용해 위치·자세를 일관되게 얻는다.

- Front 상판 마커 ID(기본 front=10 — 기존 ID0과 분리, `aruco_tracker_node`와
  파이프라인이 섞이지 않게)를 `cv2.aruco`로 검출
- 마커 4코너 픽셀 → homography로 world 좌표 변환 (`yolo_bev_map_node.pixel_to_world`와
  동일 관례)
- yaw는 solvePnP 없이 **코너 두 점(top-left→top-right)의 world 벡터 방향**으로 직접
  계산 — ArUco 코너 순서가 마커 자신의 로컬 축 기준이라 마커가 이미지에서 어떻게
  돌아가 있어도 항상 마커의 +x변을 가리킨다. 카메라가 거의 나달이므로 (aruco_tracker_node가
  쓰는) 카메라 intrinsics 기반 3D pose 추정이 필요 없다 — homography 하나로 위치·자세
  전부 처리.
- 출력 pose의 `header.stamp`는 **입력 이미지의 촬영시각을 그대로 전파**한다. 여기서
  `now()`로 새로 찍으면 §10-3에서 고친 신선도 검사(수신시각 대신 촬영시각 사용)가
  무의미해진다 — 처리지연이 실제로 얼마나 나는지 숨겨버리기 때문.

**아직 실측/검증 안 된 것 (동작은 하지만 정확도 미보장, §7·§10-1 확장):**
1. **부착각(yaw_offset)** — 마커의 top-left→top-right 변이 로봇 진행축(+x)과 정확히
   일치하게 부착해야 한다. `front_yaw_offset_deg` 파라미터로
   상쇄하는데, 지금은 0.0 placeholder라 실측 전엔 고정 yaw 바이어스가 남는다.
2. **Parallax(시차)** — homography는 "바닥" 평면 기준 캘리브레이션인데, 로봇 상판
   마커는 그보다 높은 곳(섀시 높이만큼)에 있다. 카메라 광축에서 먼 위치일수록
   실제보다 광축 반대방향으로 밀려 보이는 오차가 생긴다. 대략
   `오차 ≈ (마커 높이/카메라 설치높이) × (광축-마커 수평거리)` — 예: 카메라 2.5m,
   마커 높이 0.12m, 광축에서 2m면 약 9.6cm. 카메라·마커 높이 실측 후 이 근사가
   허용 범위인지 확인 필요. 나달 카메라 근사 보정 로직은 구현했으며
   `camera_ground_x/y_m`, `camera_height_m`, `front_marker_height_m`
   실측값을 넣어야 활성화된다. 높이가 0이면 보정은 비활성이다. yaw는 코너
   두 점이 서로 가까워 이 오차가 대부분 상쇄되므로 위치보다 영향이 작다.
3. **렌즈 왜곡 무보정** — 순수 투영변환(homography)이라 광각 왜곡을 별도 안 푼다.
   `yolo_bev_map_node`와 동일한 기존 한계를 그대로 물려받음(새 문제 아님).

`pose_fusion_node`는 이 계약(토픽명/타입)만 맞으면 바로 동작하도록 이미 만들어 뒀다.

---

## 4. EncoderOdometry 변경 (v1.6)

`EncoderOdometry.update(counts)`가 기존 누적 pose(`x,y,theta`)에 더해
**이번 호출分 body-frame 델타** `dx_body, dy_body, dtheta`를 함께 반환하도록 확장했다.

핵심 설계 판단: 이 델타를 world 프레임으로 미리 회전시키지 않고 **body 프레임 그대로**
반환한다. world 회전은 `PoseEKF.predict()`가 **자기 자신의(=CCTV로 보정되고 있는)
yaw 추정치**로 직접 수행한다. 만약 여기서 `EncoderOdometry` 내부의 드리프트 중인
`self.theta`로 미리 회전시켜 버리면, 회전 오차가 델타 자체에 섞여 들어가 나중에
PoseEKF가 아무리 yaw를 잘 보정해도 이미 지나간 위치 적분 오차는 못 고친다.

내부 진단용 누적치(`x,y,theta`)는 그대로 유지 — RViz 확인, 로그용으로 계속 쓸 수 있다.
단 **어떤 Kalman predict 입력으로도 이 누적치를 쓰면 안 된다** (CLAUDE.md §4 "칼만
predict 덮어쓰기" 재발 방지 — 과거 이 프로젝트에서 실제로 겪은 버그).

---

## 5. PoseEKF (kalman_filter.py 신규 클래스)

3-state EKF, 상태 `[x, y, yaw]`. 기존 `ScalarKalman`은 그대로 두고 별도 클래스로 추가했다
(용도가 다름 — 위 §2 참조).

### Predict (encoder 델타, 50Hz 내외 — wheel_odom 도착 시마다)

```
yaw_mid = yaw + dtheta/2                          # 회전 중 호(arc) 근사 개선
dx_w = dx_body·cos(yaw_mid) - dy_body·sin(yaw_mid)
dy_w = dx_body·sin(yaw_mid) + dy_body·cos(yaw_mid)
x += dx_w ; y += dy_w ; yaw = norm(yaw + dtheta)

F = [[1,0,-dy_w],[0,1,dx_w],[0,0,1]]               # 자코비안 (소각 근사)
Q = diag(q_pos, q_pos, q_yaw)
  q_pos = (k_pos · |Δ이동거리|)² + pos_floor·dt      # 슬립 비례항 + dt 비례 바닥잡음
  q_yaw = (k_yaw · |dtheta|)²   + yaw_floor·dt
P = F·P·Fᵀ + Q
```

`k_pos, k_yaw, pos_floor, yaw_floor`는 전부 **placeholder**다 (기본값
`k_pos=k_yaw=0.05`, `pos_floor=5e-4`, `yaw_floor=1e-3`). §7 캘리브레이션 절차로
실측 확정 필요 — 지금 값으로 배선/통신 검증은 되지만 정량적 정확도는 보장 못 한다.

### Correct (유효한 CCTV+ArUco 절대측정 도착 시)

`frame_id='map'`, 촬영시각 기준 0.5초 이내, 유효 quaternion/유한 좌표인
pose만 보정에 사용한다. pose 도착 자체가 해당 촬영 프레임의 마커 검출
증거이므로 별도 `marker_visible` Bool의 교차토픽 도착 순서를 게이트로
사용하지 않는다. Bool은 상태 진단과 연속 기각 끊기에 사용한다.

```
S = P + R                      # H=I (직접 관측)이므로 단순화
d² = eᵀS⁻¹e                    # Mahalanobis distance, e = 측정 - 예측
if d² > 11.34 (3자유도 카이제곱 99%): 이번 측정 기각 (오탐/occlusion 글리치로 간주)
else:
  K = P·S⁻¹
  x,y,yaw += K·e (yaw는 정규화)
  P = (I-K)(P)(I-K)ᵀ + K·R·Kᵀ   # Joseph form — 수치 안정성 확보
```

`R`(측정 노이즈)도 placeholder: `σ_xy=2cm, σ_yaw=3°`. 실제로는 homography
reprojection 오차 + ArUco pose 추정 지터를 정적 테스트로 측정해서 넣어야 한다(§7).

### 연속 기각(reject streak) 강제 재수렴

카이제곱 게이트를 통과 못 하는 측정이 **5회 연속** 들어오고 잔차가
재획득 상한(기본 위치 0.5m, yaw 45°) 안이면 필터가 실제로 틀어진 것으로
보고 강제로 받아들인다. 상한 밖의 측정은 반복돼도 받지 않아 100m 오탐처럼
물리적으로 불가능한 좌표로 점프하지 않는다. 마커 손실·잘못된 frame·stale
측정은 연속 기각 횟수를 끊는다. 두 경로 모두 자동 테스트로 고정했다(§8).

---

## 6. pose_fusion_node.py — 신규 노드

역할 하나: `/{role}/wheel_odom` + `/{role}/cctv_pose` → `/{role}/odom`.

- 구독: `/{role}/wheel_odom`(Odometry, twist=델타), `/{role}/cctv_pose`(PoseStamped),
  `/{role}/cctv_marker_visible`(Bool)
- 발행: `/{role}/odom`(Odometry, pose.covariance에 P의 x,y,yaw 대각성분 채움),
  `/{role}/localization_status`(String, JSON — 5Hz 진단: 보정 소스, 위치/yaw
  표준편차, 마지막 Mahalanobis 거리, 강제재수렴 여부)
- predict는 wheel_odom 콜백에서 매번 즉시 실행 (dt는 메시지 `header.stamp`
  차이, 첫 측정은 기본 0.02s, 0.2s 상한; 역행·중복 stamp는 폐기). correct는
  **cctv_pose_cb 안에서
  메시지 1개당 정확히 1번만** 실행 — 이전엔 wheel_odom 콜백에서도 캐시된
  cctv_pose를 재사용해 같은 CCTV 프레임을 최대 수십 번 중복 반영하던 버그가
  있었다 (§10-2에서 수정). 신선도는 수신시각이 아니라 `msg.header.stamp`
  (CCTV 촬영시각) 기준으로 판정한다 (§10-3).
- **정지/감속 판단은 하지 않는다.** CCTV 마커를 오래 놓쳐도 이 노드는 계속
  dead-reckoning만으로 odom을 낸다 (공분산은 자연히 커짐). "느려져야 하나/
  멈춰야 하나"는 여전히 `rigid_body_sync_node`의 기존 워치독(odom 0.5s 끊김→정지)과
  ArUco 마커손실 1s/2s 정책이 담당한다 — 책임을 이원화하지 않기 위한 의도적 설계.

---

## 7. 실측/캘리브레이션 체크리스트 (하드웨어 조립 후)

| 항목 | 방법 | 반영 위치 |
|------|------|-----------|
| `wheel_radius`, `encoder_ppr` | 바퀴 1회전 시 카운트 실측 | stm32_bridge_node 파라미터 |
| `lx, ly` (로봇 축간/윤거 절반) | 섀시 실측 | EncoderOdometry 생성 파라미터 |
| `k_pos, k_yaw` (프로세스 노이즈) | 직선/제자리 회전 반복 주행 후 encoder-only 추정치와 실제 이동량 오차의 표준편차 | PoseEKF 생성 파라미터 |
| `meas_sigma_xy_m, meas_sigma_yaw_deg` | 마커를 여러 알려진 지점에 고정해두고 CCTV 추정치 분산 측정 (정적 테스트) | pose_fusion_node 파라미터 |
| `init_x/y/yaw` (로봇별) | 전원 인가 시 도킹 홈 위치 실측 | front/rear_robot.launch.py |
| `front_yaw_offset_deg` (마커 부착각) | 마커 top-left→top-right 변과 Front 진행축 정렬 오차 실측 | cctv_robot_marker_node 파라미터 |
| `front_marker_offset_x_m` (마커 바깥끝 오프셋) | Front 마커 중심 ↔ 회전중심(base_link) 거리를 진행축(+x)으로 실측. 검증: 로봇 제자리 회전 시 발행 base_link 위치가 안 움직여야 정상 | cctv_robot_marker_node 파라미터 |
| Parallax 보정 파라미터 | 카메라 설치 높이·광축 바닥 교점·Front 마커 높이를 실측하고 유효범위 최악 오차 확인 | cctv_robot_marker_node의 `camera_ground_x/y_m`, `camera_height_m`, `front_marker_height_m`; 높이 0은 비활성 |
| `cctv_camera_calibration.npz` | 천장 카메라 intrinsic/distortion — 패키지 포함 | cctv_rectify_node |
| `homography_rectified.npy` | `/cctv/image_rect` 바닥 기준점으로 생성 — 결과 파일은 별도 필요 | yolo_bev_map_node와 cctv_robot_marker_node 공통 |
| `rear_camera_calibration.npz` | Rear marker 카메라 전용 intrinsic/distortion | aruco_tracker_node; 천장 파일과 별개 |
| CCTV 프레임레이트/지연 | Jetson 실측 (YOLO+ArUco 동시 처리 부하 포함, cctv_robot_marker_node도 같은 프레임을 매번 detectMarkers 처리하므로 부하 가산됨) | `cctv_timeout` 파라미터, 필요시 지연보상 로직 추가 검토 |

낙관적으로 말하지 않기: 지금 이 문서의 모든 게인·노이즈 값은 **동작 검증용
placeholder**이지 정확도를 보장하는 값이 아니다. 특히 `k_pos/k_yaw`처럼
"엔코더 슬립 비율"에 해당하는 값은 로봇마다, 바닥 마찰계수(검정 종이 바닥
기준)마다 달라질 수 있어 반드시 실측이 필요하다.

---

## 8. 검증한 것 / 안 한 것

**했음:**
- `test/test_localization_math.py`의 자동 테스트 6개 통과: Rear yaw
  회전행렬 0°/±20°와 실제 solvePnP 20°, 엔코더 reset 폐기, 반복 100m
  오탐 차단, 상한 안의 5회 연속 측정 재획득, OpenCV 4.6 legacy ArUco API.
- Ubuntu 24.04의 실제 OpenCV 4.6.0에서 호환 어댑터 초기화 성공.
- localization Python 모듈·launch 전체 `compileall`, `git diff --check` 통과.
- 앞선 수치 스니펫에서 2% 엔코더 바이어스가 있는 500스텝 시뮬레이션은
  10Hz·2cm 노이즈 CCTV 보정 시 참값 0.5m 대비 4mm 이내로 수렴했다.

**안 했음 (하드웨어 없이는 불가능):**
- 실제 STM32 UART "E,..." 타이밍/주기 — dt 계산이 가정하는 수신 간격이 맞는지
- 실제 CCTV+ArUco 파이프라인 지연 (Jetson 처리시간 포함) — `cctv_timeout=0.5s`가
  적절한지
- ROS2 노드 3대(Jetson/Front RPi/Rear RPi) 간 실제 rclpy 통합 실행 (import/토픽
  연결 문법 오류는 `py_compile`로만 확인했고, 실제 spin 테스트는 안 함)
- Gazebo 시뮬레이션 반영 — `parking_gz_sim_formation_fixed`는 현재 Gazebo
  ground-truth odom을 직접 쓰는 구조라 이 변경과 별개(§ CLAUDE.md 논의 참조).
  이 설계를 sim에서 검증하려면 encoder-odom 노이즈 주입 + 가짜 CCTV/ArUco 토픽을
  발행하는 테스트 하네스가 별도로 필요함 (미작성).

---

## 9. system_spec_v15_reviewed.md 대비 변경/충돌 사항 (반영 필요)

1. **"CCTV는 위치만 보정하며 yaw는 제공하지 않음" (라인 185, 191)** —
   이 문장은 "CCTV가 채도/색상 블롭으로 yaw를 추정하면 안 된다"는 규칙(정당함,
   CLAUDE.md §4 "CCTV yaw 과장" 항목)이지, "CCTV가 ArUco 마커 pose를 읽어서
   yaw를 얻는 것"까지 막는 규칙은 아니어야 한다. 이번 설계로 CCTV는 로봇 상판
   마커를 통해 **정당한 방법으로** yaw를 제공하게 된다. 문서 문구를 "CCTV의
   색상/블롭 기반 추정은 yaw를 제공하지 않는다 — yaw는 ArUco 마커(측면 상대
   또는 CCTV 상판 마커)에서만 얻는다"로 수정 권장.
2. **7-6 rigid_body_sync_node 입력 목록** — `/front/odom`, `/rear/odom`의 출처가
   "stm32_bridge_node 직접 발행"에서 "pose_fusion_node가 융합 후 발행"으로 바뀜.
   입력 목록 자체(토픽명)는 안 바뀌었으니 인터페이스 문서 수정은 선택사항이지만,
   §7-8 stm32_bridge_node의 출력 항목(`/{role}/odom`)은 `/{role}/wheel_odom`으로
   정정 필요.
3. **12. 파일 구조** — `pose_fusion_node.py`, `cctv_robot_marker_node.py` 신규 파일 추가 필요.

---

## 10. 외부 리뷰(v1.7)로 발견된 버그 — 수정 완료 6건 (+ v1.8 후속: 미해결 항목도 구현)

최초 구현(§1~9) 이후 별도 리뷰를 거쳐 실제 동작을 무너뜨릴 수 있는 버그
6건을 찾아 모두 고쳤다. 전부 코드로 재현 후 수정했다(아래 각 항목의 재현
결과 참조). 리뷰가 지적한 문제 중 1건(§10-1)은 당시 의도적으로 범위 밖에
남겨뒀는데, v1.8에서 `cctv_robot_marker_node`로 구현했다 — 자세한 내용은
§3 "cctv_robot_marker_node" 절 참조.

### 10-1. (v1.7 당시 미해결 → v1.8에서 구현) cctv_robot_marker_node

리뷰 시점(v1.7)에는 **이게 설계 전체에서 유일하게 남은 "진짜" 차단
요인**이었다. `/{role}/cctv_pose`, `/{role}/cctv_marker_visible`을 내는
노드가 없으면 `pose_fusion_node`의 `correct()`가 한 번도 호출되지 않고,
`PoseEKF`는 `predict()`만 영원히 반복하는 상태 — `/front/odom`,
`/rear/odom`이 이름만 "융합된 odom"이지 실질적으로는 순수 엔코더
dead-reckoning이었다.

v1.8에서 `cctv_robot_marker_node.py`로 구현 완료 — 상세 설계는 §3
"cctv_robot_marker_node" 절 참조. **단, 배선은 끝났지만 정확도가 검증된
건 아니다** — homography 파일, 부착각 오프셋, 카메라/마커 높이(parallax)
전부 실측 전 placeholder다. §1에서 지적한 "개별 로봇 절대 yaw 무보정"
공백이 **경로상으로는** 메워졌지만, 수치가 맞는지는 하드웨어 실측 전까지
알 수 없다는 점은 그대로다.

### 10-2. (수정 완료) CCTV 측정값 중복 반영

`pose_fusion_node`가 `wheel_odom` 콜백(엔코더 도착률, 최대 수십Hz)마다
캐시된 `cctv_pose`로 `correct()`를 매번 재호출했다. CCTV가 더 낮은
주기로 갱신되므로, `cctv_timeout`(0.5s) 동안 같은 프레임 하나가 독립적인
새 측정인 것처럼 최대 수십 번 반영됨.
- 재현: 동일한 정상 측정을 25회 적용 → 위치 표준편차가 실제로는 그대로여야
  하는데 약 20mm→4mm로 허위 축소.
- 부작용: 이상치 게이트(§5 연속기각)의 "5회 연속"이 진짜 독립된 5번의 나쁜
  측정이 아니라 한 프레임을 5번 재사용한 것만으로 채워져서, 이상치 1개가
  들어와도 거의 즉시 강제 재수렴이 발동 — 게이트가 사실상 무력화.
- 수정: `correct()` 호출을 `wheel_odom_cb`에서 제거하고 `cctv_pose_cb`
  안으로 옮김 — 메시지 1개당 정확히 1번만 반영 (`pose_fusion_node.py`).

### 10-3. (수정 완료) CCTV 신선도를 수신 시각으로 판정

`msg.header.stamp`(촬영/검출 시각)를 무시하고 콜백이 불린 시각을 저장해서
신선도를 계산했다. 네트워크·Jetson 처리 지연으로 오래된 pose도 "방금 온
새 측정"으로 오인할 수 있었다. `Time.from_msg(msg.header.stamp)` 기준으로
변경 (`pose_fusion_node.py`). 단, 지연 자체를 과거 시점으로 되돌려
재적분하는 delay compensation은 여전히 안 함 — 이 로봇 속도 영역(≤0.08m/s)에서는
영향이 작다고 보고 미루기로 함(§8 "안 했음"과 동일 성격의 근사).

### 10-4. (수정 완료) Rear ArUco yaw가 잘못된 회전축에서 계산됨

`aruco_tracker_node.py`의 최초 공식
`atan2(rot[1][0], rot[0][0])`은 카메라 Z축(광축) 회전만 읽어 좌우
heading을 놓쳤다. v1.8에서 Y축 성분
`atan2(rot[0][2], rot[2][2])`로 바꿨지만, solvePnP의 마커 +Z 법선이
카메라 쪽을 향하는 이 object-point 관례에서는 정면이 180°가 되는 부호
오류가 남아 있었다. v1.9에서 로봇 진행축인 음의 마커 법선을 사용해
`atan2(-rot[0][2], -rot[2][2])`로 확정했다. 합성 회전행렬의 0°/±20°
회귀 테스트로 검증했으며 부착 오차는 `yaw_offset_deg`로 보정한다.

### 10-5. (수정 완료) 엔코더 카운터 리셋/rollover 무방비

RPi 측(`encoder_odometry.py`)은 `counts[i]-prev[i]`를 그대로 믿어서,
STM32 재부팅으로 누적 카운트가 예: 10000→0으로 리셋되면 그 델타(-10000)를
실제 이동으로 해석 — 재현 결과 한 주기에 약 -0.725m 위치 점프 발생. 한
주기에 물리적으로 불가능한 틱 변화(기본 임계값 1300틱, 최대속도 대비 50배
이상 여유)가 감지되면 그 주기 모션을 버리고 카운터 기준점만 재동기화하도록
가드 추가.

STM32 측(`parking_robot_firmware.c`)도 같은 계열 문제가 있었다. F401RE에는
TIM8이 없으므로 네 엔코더를 TIM2/TIM3/TIM4/TIM5로 재배치했다. 이 중
TIM2/TIM5는 32비트, TIM3/TIM4는 16비트다. 16비트 타이머를 단순
`int32_t` 차분으로 계산하면 하드웨어 카운터가 0↔65535로 순환할 때 거대한
반대방향 델타가 생긴다. TIM3/TIM4만 `int16_t` 캐스팅 차분으로 처리해
wraparound를 복원한다. 실제 CubeMX `.ioc`와 핀 배선 대조는 여전히 필수다.

### 10-6. (수정 완료) lx, ly 실측 파라미터가 배선 안 됨

`EncoderOdometry` 생성자엔 `lx, ly` 인자가 있었지만 `stm32_bridge_node`가
`wheel_radius, encoder_ppr`만 넘기고 있어서, launch 파일에 `lx/ly`를 아무리
설정해도 항상 기본값(0.10/0.10)으로 고정되는 상태였다. `stm32_bridge_node`에
파라미터 선언 추가하고 전달하도록 수정.

### 10-7. (수정 완료, 사전 존재 버그) rigid_body_sync 초기화 레이스

이번 세션에서 새로 만든 코드는 아니고 원래 있던 `rigid_body_sync_node.py`의
문제. `front_ready`/`rear_ready`가 되기 전(front/rear가 placeholder (0,0,0))에
`/parking/vehicle_pose_feedback`이 먼저 도착하면 `cctv_offset`이
"CCTV절대위치 - (0,0)"으로 계산되어 스무딩(α=0.3)에 섞여 들어간다 — 이후
실제 odom이 들어와도 이 초기 오염이 지수평활 특성상 몇 사이클 잔류하며
일시적 이중보정을 일으킬 수 있다. `front_ready and rear_ready` 가드 추가.

### 10-8. (v1.9 재검증) 배포 경로·호환성·안전장치 보완

- Ubuntu 24.04의 OpenCV 4.6에는 `cv2.aruco.ArucoDetector`가 없음을 실제
  환경에서 확인했다. `aruco_utils.ArucoDetectorCompat`가 신/구 API를
  모두 지원하도록 두 ArUco 노드에 공통 적용했다.
- 같은 OpenCV 4.6에서 `SOLVEPNP_IPPE_SQUARE`가 재투영 오차 약 28px인
  잘못된 반대면 해를 반환함을 재현했다. 정확히 20°를 복원한
  `SOLVEPNP_ITERATIVE`로 고정하고 실제 solvePnP 회귀 테스트를 추가했다.
- `full_system.launch.py`에 빠져 있던 `cctv_robot_marker_node`를 추가했다.
- visible Bool과 pose가 서로 다른 DDS 토픽이라 전달 순서를 보장할 수 없는
  경쟁을 제거했다. pose 자체를 현재 프레임 검출 증거로 사용한다.
- homography와 Rear 카메라 calibration이 없거나 비정상이면 잘못된 pose를
  계속 내지 않고 기본적으로 노드 시작을 중단한다.
- 5회 연속 게이트 기각 뒤 강제 재획득에는 위치/yaw 잔차 상한을 추가해
  반복되는 극단적 오탐으로의 점프를 막았다.
- 카메라 Image 구독은 sensor-data QoS로 바꾸고, CCTV/Rear ArUco/차량
  feedback은 입력 이미지의 촬영시각을 보존하도록 했다.
- 세 노드가 쓰는 NumPy/OpenCV/pyserial 실행 의존성을 `package.xml`에
  선언하고 ROS 비의존 회귀 테스트 5개를 추가했다.

### 검증 방법에 대한 솔직한 한계

ROS 비의존 자동 테스트 6개, Python 전체 컴파일, git diff 형식 검사,
Ubuntu 24.04/OpenCV 4.6 호환 어댑터 초기화는 통과했다. 다만 이 환경에는
ROS2/rclpy와 실기체가 없어 노드를 실제로 `spin()`시킨 3대 통합,
카메라 영상·UART·DDS 지연, 캘리브레이션 정확도는 확인하지 못했다.
상세 판정과 재현 명령은 `localization_validation_report.md`에 기록한다.


## v1.2 후속 검토

후진/횡이동 yaw-hold, FINAL_APPROACH 공통 동기제어, ArUco 중심거리 offset,
STM32F401RE 타이머 재배치와 최신 24개 회귀 테스트는
`REVIEW_FIXES_V1_2.md`를 기준으로 한다.
