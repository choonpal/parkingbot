# 천장 카메라 보정 적용

## 전달받은 파일

`calibration_data.npz`는 다음 키를 포함한다.

```text
mtx  : 3x3 camera matrix
dist : 1x5 distortion coefficients
```

수치:

```text
fx = 708.48633456
fy = 707.63853756
cx = 664.39994909
cy = 358.75645269

dist = [0.03678515, 0.05353067, -0.00274540, 0.00510393, -0.07254668]
```

패키지에는 `config/cctv_camera_calibration.npz`로 정규화해 넣었으며 원본 키와 표준 키를 모두 보존했다.

## 듀얼 CCTV 임시 640×480 보정값

2026-08-14 전달본에서 다음 카메라별 파일을 가져왔다.

```text
config/cctv0_camera_calibration.npz
config/cctv2_camera_calibration.npz
```

두 파일은 서로 다른 intrinsic이며 `mtx/dist` 키를 사용한다. principal point가
각각 약 `(340,238)`, `(327,256)`이므로 이번 단계에서는 **640×480 보정값**으로
취급한다. `cctv_server_dual.launch.py`의 영상 및 calibration 기본 해상도도
640×480으로 맞췄다.

이 파일은 파이프라인 연결과 초기 실증을 위한 provisional asset이다. 실제 장착한
cam0/cam2가 이 보정을 생성한 카메라라는 보장은 없으므로 최종 정밀 주행 전에는
각 실제 카메라를 다시 보정해야 한다. 재보정 결과는 다음 runtime 경로에 둔다.

```text
~/.ros/adaptive_valet_bot/cctv0_camera_calibration.npz
~/.ros/adaptive_valet_bot/cctv2_camera_calibration.npz
```

영상 해상도 또는 NPZ를 바꾸면 rectified pixel 좌표가 바뀐다. 따라서 기존
Homography를 재사용하지 말고 두 카메라 모두 `/cctv*/image_rect`에서
Homography와 layout을 다시 등록한다.

전달 ZIP에는 실측 Homography가 없으며 저장소도 합성 fallback을 제공하지
않는다. 실차 기동 전 각 카메라의 rectified 영상에서 같은 map 기준점을 찍어
`homography_cam0_rectified.npy`와 `homography_cam2_rectified.npy`를 각각
생성해야 한다. 파일이 없거나 검증되지 않았으면 운행하지 않는다.

## 처리 순서

```text
천장 카메라 Raw /cctv/image_raw
  -> cctv_rectify_node
  -> 보정 영상 /cctv/image_rect
       -> YOLO 차량/빈자리 검출
       -> 상판 ArUco 검출
       -> homography_rectified.npy로 world 좌표 변환
```

YOLO와 상판 ArUco는 반드시 같은 `/cctv/image_rect`를 사용한다.

## Homography 재생성 필수

렌즈 왜곡 보정 전후에는 픽셀 좌표가 달라진다. 따라서 기존 Raw 영상에서 만든 `homography_matrix.npy`는 보정 영상에 적용하면 안 된다.

1. `cctv_rectify_node`를 실행한다.
2. `/cctv/image_rect`를 RViz 또는 이미지 뷰어로 연다.
3. 바닥 기준점을 해당 화면에서 선택한다.
4. 그 좌표로 `homography_rectified.npy`를 생성한다.
5. YOLO와 상판 마커 노드 모두 같은 파일을 사용한다.

## 해상도 확인

이 NPZ에는 캘리브레이션 영상 크기가 없다. `(cx≈664, cy≈359)` 때문에 1280×720 가능성이 높아 보이지만 확정할 수 없다.

- 캘리브레이션과 실시간 영상 크기가 같으면 `calibration_width_px:=0`, `calibration_height_px:=0`을 유지한다.
- 캘리브레이션 크기가 확인됐고 실시간 영상을 같은 종횡비로 리사이즈했다면 원본 크기를 launch에 넣는다.
- 16:9로 캘리브레이션하고 4:3으로 실행하는 등 종횡비가 달라지면 단순 스케일 보정으로 처리하지 않고 다시 캘리브레이션한다.

## Rear 카메라와 분리

이 파일은 천장 카메라 전용이다. Rear 로봇이 Front 후면 ID0을 보는 전방 카메라에는 별도의 `rear_camera_calibration.npz`가 필요하다.

## 두 대의 천장 카메라를 쓸 경우

현재 패키지는 카메라별 raw → rectify → YOLO 결과를 `cctv_merge_node`에서
공통 map frame으로 합친다. 카메라마다 아래 파일이 따로 필요하다.

```text
cctv0_camera_calibration.npz + homography_cam0_rectified.npy
cctv2_camera_calibration.npz + homography_cam2_rectified.npy
```

두 Homography는 같은 바닥 기준점에 같은 map 좌표를 입력해 하나의 world
좌표계로 맞춘다. 한 카메라의 calibration을 다른 카메라에 복사해 쓰면 안 된다.
