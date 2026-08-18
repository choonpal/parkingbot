# Jetson YOLO + ArUco 통합 검토

## 결론

전달된 단독 스크립트는 카메라 인식과 Flask 화면 확인에는 유용하지만, 그 자체로는 ROS 2 맵·로봇 pose·경로 토픽을 발행하지 않는다. v1.6에서는 해당 기능을 ROS 파이프라인에 나눠 통합했다.

| 기능 | 담당 노드 |
|---|---|
| 카메라 단독 점유 및 `/cctv/image_raw` 발행 | `opencv_camera_node` |
| 렌즈 왜곡 보정 | `cctv_rectify_node` |
| 차량/빈자리/OccupancyGrid | `yolo_bev_map_node` |
| Front ID10·Rear ID11 천장 절대 pose | `cctv_robot_marker_node` |
| YOLO+ArUco 웹 화면 | `jetson_vision_web_node` |

## 원본을 그대로 임무 코드로 쓰면 생기는 문제

1. `cv2.VideoCapture(0)`를 직접 열어 기존 ROS 카메라 드라이버와 충돌할 수 있다.
2. Flask 요청마다 `generate_frames()`가 실행되므로 여러 접속자가 같은 카메라를 각각 열 수 있다.
3. `yolov8n.pt`는 COCO 모델이며 `empty_slot` 클래스가 없다.
4. 모든 COCO 객체를 그리는 결과는 화면 확인용이지 OccupancyGrid 장애물 정의가 아니다.
5. 검출 결과가 ROS 메시지로 발행되지 않는다.
6. `tvec[2]`는 광축 방향 깊이이며 일반적인 카메라-마커 직선거리는 `||tvec||`이다.
7. Raw 프레임에서 PnP를 하고 별도로 rectified Homography를 사용하면 픽셀 좌표계가 섞일 수 있다.

## v1.6 안전장치

- `model_mode`를 `coco`, `vehicle_seg`(권장), `parking_seg` 중 하나로 명시
- `vehicle_seg`는 차량 mask와 고정 등록 슬롯의 겹침률로 점유를 판정
- COCO 모드에서는 차량 class ID만 허용
- 카메라 ownership을 publisher 한 곳으로 제한
- Mission 노드와 웹 진단 노드 분리
- 동일 `/cctv/image_rect` 사용
- YOLO/ArUco가 같은 `homography_scale_to_m` 사용
- ArUco는 지정 ID와 최소 면적을 모두 통과한 가장 큰 후보만 사용
- 웹 PnP는 Euclidean 거리 표시

## 실행 가능성

### 정적 실행 준비가 된 부분

- ROS 2 Humble console entry point와 launch 연결
- 카메라 publisher → rectify → YOLO/ArUco 토픽 연결
- COCO/커스텀 모델 모드 분리
- 선택적 Flask 서버
- calibration 파일 로더
- Homography 유효성 검사

### 현장 파일 없이 검증할 수 없는 부분

- 실제 `homography_rectified.npy`의 좌표 정확도
- top-down 모형차에 대한 COCO YOLO recall
- 18cm 마커가 실제 영상에서 1000px 이상인지
- Jetson의 실시간 FPS와 열/전력 제한
- 카메라 드라이버가 실제로 1280×720을 수용하는지
- TensorRT engine과 현재 JetPack/TensorRT의 호환성

### 운용 권장

Mission 실행 시 웹 YOLO는 끄고 ArUco/영상 모니터만 사용한다.

```text
enable_operator_ui=false
enable_debug_overlay=true
debug_enable_yolo=false
debug_enable_aruco=true
```

모델 성능을 튜닝할 때만 `debug_enable_yolo=true`로 켠다.

## 통과 기준

1. `/cctv/image_raw`가 1280×720로 안정 발행된다.
2. `/cctv/image_rect`에 직선 왜곡이 줄고 프레임 크기가 예상과 같다.
3. 알려진 바닥 4점 이상에서 Homography 오차를 측정한다.
4. `bev_layout_calibration.launch.py`의 브라우저에서 대기구역과 슬롯을
   등록한다. 생성물은 기본 mission launch가 같은
   `~/.ros/adaptive_valet_bot/` 경로에서 자동으로 읽는다.
5. 차량이 대기구역에서 연속 2초 검출되어 `/parking/target_ready=true`가 된다.
6. 슬롯별 점유 결과가 실제와 일치한다.
7. Front ID10과 Rear ID11 pose/yaw가 정지 상태에서 튀지 않는다.
8. 웹을 켜고 꺼도 Mission 토픽 값이 변하지 않는다.
