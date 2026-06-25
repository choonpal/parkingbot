#!/usr/bin/env python3
"""
==================================================
calibrate.py
==================================================
CCTV 호모그래피 캘리브레이션 도구

90도 수직 천장 카메라라도 픽셀 좌표를 실제 거리(cm)로
변환하기 위한 호모그래피 행렬(H)을 생성합니다.

사용 순서:
  1) 바닥에 기준점 4개 표시 (테이프, ArUco 등)
  2) 각 점 사이의 실제 거리를 줄자로 측정
  3) 이 스크립트 실행
  4) 카메라 화면에서 기준점 4개를 순서대로 클릭
  5) 각 점의 실제 좌표(cm) 입력
  6) homography_matrix.npy 자동 생성

cctv_fusion_node.py와 같은 폴더에서 실행하세요.

실행: python3 calibrate.py
"""

import cv2
import numpy as np
import sys
import os
import argparse


class HomographyCalibrator:
    def __init__(self, camera_id=0, width=1280, height=720,
                 output='homography_matrix.npy', num_points=4):
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.output = output
        self.num_points = num_points

        self.pixel_points = []
        self.real_points = []
        self.img = None
        self.img_display = None

    # ================================================
    # 마우스 클릭 콜백
    # ================================================
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.pixel_points) < self.num_points:
                self.pixel_points.append([x, y])
                idx = len(self.pixel_points)

                # 클릭 위치에 점 + 번호
                cv2.circle(self.img_display, (x, y), 8, (0, 0, 255), -1)
                cv2.circle(self.img_display, (x, y), 12, (0, 255, 0), 2)
                cv2.putText(self.img_display, str(idx),
                            (x + 15, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 255, 0), 2)
                cv2.imshow("Calibration", self.img_display)

                print(f"  점 {idx}: 픽셀 ({x}, {y})")

                if len(self.pixel_points) == self.num_points:
                    print(f"\n  {self.num_points}개 점 선택 완료!")
                    print("  아무 키나 누르면 좌표 입력 단계로 넘어갑니다.")

    # ================================================
    # 카메라에서 캡처
    # ================================================
    def capture_from_camera(self):
        print(f"\n[카메라 연결] id={self.camera_id}, "
              f"{self.width}x{self.height}")
        cap = cv2.VideoCapture(self.camera_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        if not cap.isOpened():
            print("[오류] 카메라를 열 수 없습니다!")
            print("  - 카메라 연결 확인")
            print("  - camera_id 확인 (--camera 옵션)")
            sys.exit(1)

        print("\n[안내] 카메라 미리보기")
        print("  'c' 키: 현재 프레임 캡처")
        print("  'q' 키: 종료")

        while True:
            ret, frame = cap.read()
            if not ret:
                print("[오류] 프레임 읽기 실패!")
                break

            preview = frame.copy()
            cv2.putText(preview, "Press 'c' to capture, 'q' to quit",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 255), 2)
            cv2.imshow("Camera Preview", preview)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('c'):
                self.img = frame.copy()
                self.img_display = frame.copy()
                cv2.imwrite("calibration_capture.jpg", frame)
                print("\n[캡처 완료] calibration_capture.jpg 저장됨")
                break
            elif key == ord('q'):
                cap.release()
                cv2.destroyAllWindows()
                sys.exit(0)

        cap.release()
        cv2.destroyAllWindows()

    # ================================================
    # 저장된 이미지에서 로드
    # ================================================
    def load_from_image(self, path):
        self.img = cv2.imread(path)
        if self.img is None:
            print(f"[오류] 이미지 로드 실패: {path}")
            sys.exit(1)
        self.img_display = self.img.copy()
        print(f"\n[이미지 로드] {path} "
              f"({self.img.shape[1]}x{self.img.shape[0]})")

    # ================================================
    # 기준점 클릭
    # ================================================
    def pick_points(self):
        print(f"\n{'='*50}")
        print(f"  기준점 {self.num_points}개를 순서대로 클릭하세요")
        print(f"  권장 순서: 좌상 → 우상 → 우하 → 좌하")
        print(f"{'='*50}\n")

        cv2.imshow("Calibration", self.img_display)
        cv2.setMouseCallback("Calibration", self.mouse_callback)

        while len(self.pixel_points) < self.num_points:
            if cv2.waitKey(100) & 0xFF == ord('q'):
                print("\n취소됨")
                sys.exit(0)

        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # ================================================
    # 실제 좌표 입력
    # ================================================
    def input_real_coords(self):
        print(f"\n{'='*50}")
        print(f"  각 점의 실제 좌표를 입력하세요 (cm 단위)")
        print(f"  ※ 한 점을 원점(0,0)으로 잡으면 편합니다")
        print(f"{'='*50}\n")

        for i in range(self.num_points):
            px, py = self.pixel_points[i]
            print(f"  점 {i+1} (픽셀: {px}, {py})")
            while True:
                try:
                    x = float(input(f"    실제 X (cm): "))
                    y = float(input(f"    실제 Y (cm): "))
                    break
                except ValueError:
                    print("    숫자를 입력하세요!")
            self.real_points.append([x, y])
            print()

    # ================================================
    # 호모그래피 계산 + 저장
    # ================================================
    def compute_and_save(self):
        pts_pixel = np.float32(self.pixel_points)
        pts_real = np.float32(self.real_points)

        H, status = cv2.findHomography(pts_pixel, pts_real)

        if H is None:
            print("[오류] 호모그래피 계산 실패!")
            print("  기준점이 일직선상에 있지 않은지 확인하세요.")
            sys.exit(1)

        np.save(self.output, H)
        print(f"\n[저장 완료] {self.output}")
        print(f"\n  H 행렬:")
        print(f"  {H}")

        # 검증: 각 점 변환 오차 확인
        print(f"\n[검증] 변환 오차:")
        max_error = 0.0
        for i in range(self.num_points):
            px, py = self.pixel_points[i]
            pt = np.array([px, py, 1.0])
            r = H @ pt
            rx, ry = r[0]/r[2], r[1]/r[2]
            ex, ey = self.real_points[i]
            err = np.sqrt((rx-ex)**2 + (ry-ey)**2)
            max_error = max(max_error, err)
            print(f"  점 {i+1}: 변환({rx:6.1f}, {ry:6.1f}) "
                  f"vs 실제({ex:6.1f}, {ey:6.1f}) → 오차 {err:.2f}cm")

        if max_error < 1.0:
            print(f"\n  최대 오차 {max_error:.2f}cm — 우수")
        elif max_error < 3.0:
            print(f"\n  최대 오차 {max_error:.2f}cm — 양호")
        else:
            print(f"\n  [주의] 최대 오차 {max_error:.2f}cm — "
                  f"기준점을 다시 측정하는 것을 권장")

        return H

    # ================================================
    # 변환 결과 시각화
    # ================================================
    def visualize(self, H):
        result = self.img.copy()
        h, w = result.shape[:2]

        # 격자점을 변환해서 표시
        for px in range(0, w, 80):
            for py in range(0, h, 80):
                pt = np.array([px, py, 1.0])
                r = H @ pt
                if abs(r[2]) < 1e-10:
                    continue
                rx, ry = r[0]/r[2], r[1]/r[2]
                cv2.circle(result, (px, py), 2, (255, 0, 0), -1)
                cv2.putText(result, f"({rx:.0f},{ry:.0f})",
                            (px+3, py-3), cv2.FONT_HERSHEY_SIMPLEX,
                            0.3, (0, 255, 255), 1)

        cv2.imwrite("calibration_verify.jpg", result)
        print("\n[시각화] calibration_verify.jpg 저장됨")
        print("  각 격자점의 변환된 실제 좌표(cm)를 확인하세요.")

        cv2.imshow("Verification (press any key)", result)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        description='CCTV 호모그래피 캘리브레이션')
    parser.add_argument('--camera', type=int, default=0,
                        help='카메라 ID (기본 0)')
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--image', type=str, default=None,
                        help='저장된 이미지 사용 (카메라 대신)')
    parser.add_argument('--output', type=str,
                        default='homography_matrix.npy')
    parser.add_argument('--points', type=int, default=4,
                        help='기준점 개수 (기본 4, 많을수록 정확)')
    args = parser.parse_args()

    print("=" * 50)
    print("  CCTV 호모그래피 캘리브레이션")
    print("=" * 50)

    cal = HomographyCalibrator(
        camera_id=args.camera,
        width=args.width,
        height=args.height,
        output=args.output,
        num_points=args.points)

    # 이미지 소스
    if args.image:
        cal.load_from_image(args.image)
    else:
        cal.capture_from_camera()

    # 기준점 클릭
    cal.pick_points()

    # 실제 좌표 입력
    cal.input_real_coords()

    # 계산 + 저장
    H = cal.compute_and_save()

    # 시각화
    cal.visualize(H)

    print("\n[완료] 다음: cctv_fusion_node.py 실행")
    print(f"  ros2 run your_package cctv_fusion_node "
          f"--ros-args -p homography_file:={args.output}")


if __name__ == '__main__':
    main()