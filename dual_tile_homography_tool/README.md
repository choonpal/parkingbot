# Dual CCTV 40cm Tile Homography Tool

Jetson Orin Nano에서 CAM0/CAM2를 한 프로그램으로 등록하는 도구입니다.

## 핵심 방식

```text
CAM0 raw -> cctv0_camera_calibration.npz -> rectified cam0 -> H0
CAM2 raw -> cctv2_camera_calibration.npz -> rectified cam2 -> H2

공통 Tile(i,j)
X = origin_x + i * tile_pitch
Y = origin_y + j * tile_pitch
```

바닥 타일이 40 cm라면 기본 `tile_pitch = 0.400 m`입니다.

두 카메라에서 같은 물리 타일 꼭짓점에는 반드시 같은 `(i,j)`를 입력합니다.
따라서 겹치는 영역에서도 metre 좌표를 따로 계산할 필요가 없습니다.

## 포함 파일

- `dual_tile_homography_gui.py`
- `cctv0_camera_calibration.npz`
- `cctv2_camera_calibration.npz`
- `run_dual.sh`
- `copy_results_to_project.sh`
- `output/`

현재 포함된 두 calibration 자산은 GitHub `choonpal/parkingbot`의
`ros2/cooperative_parking_robot/config/`에서 가져온 640x480 자산입니다.
실제 설치 카메라와 보정 당시 카메라가 다르면 최종 시운전 전 intrinsic calibration을 다시 해야 합니다.

## 설치

```bash
sudo apt update
sudo apt install -y \
  python3-opencv \
  python3-numpy \
  python3-flask \
  python3-werkzeug
```

## 실행

기본 장치가 `/dev/video0`, `/dev/video2`에 해당할 때:

```bash
cd dual_tile_homography_tool
chmod +x run_dual.sh
./run_dual.sh
```

카메라 번호가 다르면:

```bash
./run_dual.sh --cam0 0 --cam2 4
```

가능하면 실제 설치에서는 `/dev/v4l/by-id/...` 사용을 권장합니다.

```bash
./run_dual.sh \
  --cam0 /dev/v4l/by-id/<CAM0-ID> \
  --cam2 /dev/v4l/by-id/<CAM2-ID>
```

## GUI

Jetson 자체:

```text
http://127.0.0.1:5001
```

Windows PC에서는 Jetson IP를 확인한 뒤:

```bash
hostname -I
```

```text
http://JETSON_IP:5001
```

## 현장 좌표계

바닥의 한 타일 꼭짓점을 공통 원점으로 정합니다.
현재 parkingbot 현장 맵의 실측 크기는 가로 4.40m × 세로 3.83m이며,
이 값이 도구의 기본 BEV 크기로 설정되어 있습니다.

```text
Tile(0,0) = (0.0, 0.0)m
Tile(1,0) = (0.4, 0.0)m
Tile(4,3) = (1.6, 1.2)m
Tile(8,5) = (3.2, 2.0)m
```

GUI에서는 X,Y를 직접 입력하지 않고 `Tile i`, `Tile j`만 입력합니다.

## 사용 순서

1. 타일 여러 칸을 줄자로 재서 실제 pitch가 0.400m인지 확인
2. 바닥 공통 원점을 Tile(0,0)으로 결정
3. `두 카메라 영상 정지`
4. CAM0 선택
5. 화면 전체에 퍼진 타일 꼭짓점 8~12개 클릭
6. 각 점에 Tile i,j 입력 후 등록
7. `현재 CAM H 계산`
8. CAM2 선택
9. CAM2에서도 같은 공통 Tile 좌표계를 사용해 8~12개 등록
10. CAM2 H 계산
11. `겹침 검증` 모드 선택
12. 겹치는 영역의 같은 Tile(i,j)를 CAM0에서 클릭
13. CAM2로 바꿔 같은 물리 꼭짓점을 클릭
14. CAM0↔CAM2 오차 확인
15. `H0 + H2 저장`

## CAM2만 다시 맞추기 (CAM0 잠금)

CAM0 정합이 정상이고 CAM2만 틀어진 경우에는 다음 모드로 실행합니다.

```bash
./run_dual.sh --cam2-only
```

이 모드는 `output/homography_cam0_rectified.npy`를 시작할 때 불러오며,
CAM0 H 계산과 전체 저장 API를 백엔드에서도 차단합니다.

1. `두 영상 정지(H 보존)`을 누릅니다.
2. CAM2 화면에 새 기준점 8~12개를 등록합니다.
3. `현재 CAM H 계산`을 누릅니다.
4. `CAM0 + CAM2 합성`에서 청록(CAM0)과 빨강(CAM2) 구조가 회색으로
   포개지는지 확인합니다.
5. RMS/최대 오차와 겹침 검증점 차이를 확인합니다.
6. `CAM2만 저장`을 누릅니다.

저장 전에 이전 CAM2 H·스냅샷·summary가 `output/backups/`에 복사됩니다.
저장 과정에서는 CAM0 `.npy`의 SHA-256을 전후 비교하여 파일이 바뀌지
않았는지도 검증합니다.

## 기준점 추천

카메라 한 대당 8~12점:

- 영상 네 모서리 쪽
- 중앙
- WAITING ZONE 쪽
- P1~P4 쪽
- 겹침 구역

두 카메라 겹침 구역에서는 동일 타일 꼭짓점 3~5개 이상을 공통으로 쓰는 것을 권장합니다.

## RANSAC

기준점이 5개 이상이면 RANSAC을 사용합니다.
기본 임계값은 `0.03m = 3cm`입니다.

## 목표 오차

초기 목표:

```text
각 카메라 inlier RMS          < 0.02 m
각 카메라 inlier MAX          < 0.05 m
CAM0-CAM2 겹침 검증점 차이   < 0.03~0.05 m
```

Homography에 사용하지 않은 타일 꼭짓점도 겹침 검증에 포함하면 좋습니다.

## 결과 파일

```text
output/
├── homography_cam0_rectified.npy   # parkingbot 런타임용
├── homography_cam2_rectified.npy   # parkingbot 런타임용
├── homography_cam0_rectified.npz   # 보관용
├── homography_cam2_rectified.npz   # 보관용
├── dual_homography_summary.json
├── parking_layout.yaml
├── cam0_rectified_snapshot.jpg
└── cam2_rectified_snapshot.jpg
```

현재 parkingbot에 넣을 핵심 결과는 `.npz`가 아니라 다음 `.npy` 2개입니다.

```text
homography_cam0_rectified.npy
homography_cam2_rectified.npy
```

## 결과 복사

기본 런타임 폴더로 복사:

```bash
chmod +x copy_results_to_project.sh
./copy_results_to_project.sh
```

CAM2만 다시 등록했다면 CAM0와 배치 파일을 건드리지 않고 적용합니다.

```bash
./copy_results_to_project.sh --cam2-only
```

다른 주차로봇 폴더로 복사:

```bash
./copy_results_to_project.sh /원하는/주차로봇/폴더
```

## 다시 Homography 해야 하는 경우

- CCTV 위치 변경
- CCTV 각도 변경
- 해상도 변경
- 렌즈 변경
- intrinsic calibration 변경
