#!/usr/bin/env bash
# 카메라가 안 열릴 때 원인을 찾는 진단 스크립트
echo "=== 1. 비디오 장치 목록 ==="
ls -l /dev/video* 2>/dev/null || echo "  /dev/video* 없음 — USB 연결 확인"
echo
echo "=== 2. 장치별 이름 (어느 게 진짜 카메라인지) ==="
v4l2-ctl --list-devices 2>/dev/null || echo "  v4l-utils 없음: sudo apt install v4l-utils"
echo
echo "=== 3. 지금 장치를 잡고 있는 프로세스 ==="
sudo fuser -v /dev/video* 2>&1 | grep -v "^$" || echo "  없음 (정상)"
echo
echo "=== 4. 내 계정이 video 그룹에 있는가 ==="
id | tr ',' '\n' | grep -q video && echo "  OK" || echo "  없음 -> sudo usermod -aG video $USER 후 재로그인"
echo
echo "=== 5. 살아있는 ROS 카메라 관련 프로세스 ==="
pgrep -af "opencv_camera|cctv_server|camera_preview" || echo "  없음 (정상)"
echo
echo "=== 6. OpenCV로 직접 열어보기 ==="
for i in 0 1 2 3 4; do
  [ -e /dev/video$i ] || continue
  python3 - "$i" <<'PY'
import sys, cv2
i=int(sys.argv[1])
cap=cv2.VideoCapture(i, cv2.CAP_V4L2)
if not cap.isOpened():
    print(f"  /dev/video{i}: 열기 실패"); sys.exit()
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
ok, frame = cap.read()
if ok:
    print(f"  /dev/video{i}: OK  {frame.shape[1]}x{frame.shape[0]}")
else:
    print(f"  /dev/video{i}: 열렸지만 프레임 읽기 실패 (캡처 장치가 아닐 수 있음)")
cap.release()
PY
done
echo
echo "=== 7. 지원 해상도 (video0) ==="
v4l2-ctl -d /dev/video0 --list-formats-ext 2>/dev/null | grep -E "MJPG|YUYV|Size" | head -20
