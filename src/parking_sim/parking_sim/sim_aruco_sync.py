#!/usr/bin/env python3
"""
==================================================
sim_aruco_sync.py
==================================================
ArUco 융합 동기화 시뮬레이션 (하드웨어 없이)

시뮬레이션 내용:
  - 두 로봇이 가상 강체로 묶여 목표로 이동
  - 엔코더에 드리프트(미끄러짐) 주입
  - 마스터가 슬레이브 ArUco 마커를 "본다"고 가정하여 거리 측정
  - 칼만 필터로 엔코더 + ArUco 융합
  - 마커가 가끔 안 보이는 상황도 시뮬레이션

OpenCV 2D 시각화로 다음을 표시:
  - 두 로봇 + 가상 강체 링크
  - 엔코더 추정 거리 vs ArUco 측정 vs 융합 거리
  - 마커 가시성 상태

실행: python3 sim_aruco_sync.py
ROS2 불필요 — 순수 시뮬레이션
"""

import numpy as np
import math
import time
import random

try:
    import cv2
    CV2_OK = True
except ImportError:
    CV2_OK = False
    print("[경고] opencv 없음 — 콘솔 모드로만 실행")


# ==================================================
# PID
# ==================================================
class PID:
    def __init__(self, Kp, Ki, Kd, limit=0.05):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.integral = 0.0
        self.prev = 0.0
        self.limit = limit

    def compute(self, error, dt):
        p = self.Kp * error
        self.integral += error * dt
        self.integral = max(-1, min(1, self.integral))
        i = self.Ki * self.integral
        d = self.Kd * (error - self.prev) / dt if dt > 0 else 0
        self.prev = error
        return max(-self.limit, min(self.limit, p + i + d))


# ==================================================
# 거리 칼만 필터
# ==================================================
class DistanceKalman:
    def __init__(self, init=0.25):
        self.dist = init
        self.P = 0.1
        self.Q = 0.0001
        self.R = 0.0004

    def predict(self, enc_dist):
        self.dist = enc_dist
        self.P += self.Q

    def update(self, aruco_dist):
        K = self.P / (self.P + self.R)
        self.dist += K * (aruco_dist - self.dist)
        self.P = (1 - K) * self.P
        return K


# ==================================================
# 시뮬레이션 로봇
# ==================================================
class SimRobot:
    """노이즈 + 드리프트를 포함한 가상 로봇"""
    def __init__(self, x, y, theta, drift_rate=0.015):
        # 실제 위치 (ground truth)
        self.true_x = x
        self.true_y = y
        self.true_theta = theta
        # 엔코더 추정 위치 (드리프트 포함)
        self.enc_x = x
        self.enc_y = y
        self.enc_theta = theta
        self.drift_rate = drift_rate

    def move(self, vx, vy, omega, dt):
        # 실제 이동
        ct, st = math.cos(self.true_theta), math.sin(self.true_theta)
        gvx = vx * ct - vy * st
        gvy = vx * st + vy * ct
        self.true_x += gvx * dt
        self.true_y += gvy * dt
        self.true_theta += omega * dt

        # 엔코더 추정 (드리프트 + 노이즈 주입)
        drift = self.drift_rate * abs(vx) * dt
        noise = random.gauss(0, 0.0003)
        self.enc_x += gvx * dt + drift + noise
        self.enc_y += gvy * dt + noise
        self.enc_theta += omega * dt


# ==================================================
# 메인 시뮬레이션
# ==================================================
class ArucoSyncSim:
    def __init__(self):
        self.wheelbase = 0.25
        self.half_L = 0.125
        self.max_speed = 0.08
        self.max_omega = 0.3

        # 두 로봇 (실제 시작 위치)
        self.front = SimRobot(0.70, 0.60, math.radians(90), drift_rate=0.015)
        self.rear = SimRobot(0.70, 0.85, math.radians(90), drift_rate=0.020)

        self.target = (0.40, 0.15, math.radians(90))
        self.state = 'MOVING'

        self.dist_kalman = DistanceKalman(self.wheelbase)
        self.rear_pid = PID(1.2, 0.1, 0.05, limit=self.max_speed)

        self.dt = 0.02
        self.step = 0

        # 통계 기록
        self.history = {
            'encoder': [], 'aruco': [], 'fused': [], 'true': []
        }

    def get_true_distance(self):
        dx = self.front.true_x - self.rear.true_x
        dy = self.front.true_y - self.rear.true_y
        return math.sqrt(dx*dx + dy*dy)

    def get_encoder_distance(self):
        dx = self.front.enc_x - self.rear.enc_x
        dy = self.front.enc_y - self.rear.enc_y
        return math.sqrt(dx*dx + dy*dy)

    def simulate_aruco(self):
        """
        마스터 카메라가 슬레이브 마커를 본다고 가정.
        실제 거리 + 작은 측정 노이즈 반환.
        가끔(10%) 마커가 안 보임.
        """
        # 10% 확률로 마커 안 보임
        if random.random() < 0.1:
            return None
        # 실제 거리에 ArUco 측정 노이즈 (2cm 수준)
        true_dist = self.get_true_distance()
        return true_dist + random.gauss(0, 0.005)

    def get_virtual_pose(self):
        cx = (self.front.enc_x + self.rear.enc_x) / 2
        cy = (self.front.enc_y + self.rear.enc_y) / 2
        dx = self.front.enc_x - self.rear.enc_x
        dy = self.front.enc_y - self.rear.enc_y
        return cx, cy, math.atan2(dy, dx)

    def control_step(self):
        # === Step 1: 거리 융합 ===
        enc_dist = self.get_encoder_distance()
        self.dist_kalman.predict(enc_dist)

        aruco_dist = self.simulate_aruco()
        aruco_visible = aruco_dist is not None
        if aruco_visible:
            self.dist_kalman.update(aruco_dist)
        fused_dist = self.dist_kalman.dist

        # 기록
        self.history['encoder'].append(enc_dist)
        self.history['aruco'].append(aruco_dist if aruco_visible else None)
        self.history['fused'].append(fused_dist)
        self.history['true'].append(self.get_true_distance())

        # === Step 2: 가상 강체 목표 속도 ===
        cx, cy, ct = self.get_virtual_pose()
        tx, ty, tt = self.target
        dist_to_target = math.sqrt((tx-cx)**2 + (ty-cy)**2)

        if dist_to_target < 0.03:
            self.state = 'ARRIVED'
            return False

        cc, ss = math.cos(ct), math.sin(ct)
        ldx = (tx-cx)*cc + (ty-cy)*ss
        ldy = -(tx-cx)*ss + (ty-cy)*cc
        dth = math.atan2(math.sin(tt-ct), math.cos(tt-ct))

        vx = max(-self.max_speed, min(self.max_speed, 0.8*ldx))
        vy = max(-self.max_speed, min(self.max_speed, 0.8*ldy))
        om = max(-self.max_omega, min(self.max_omega, 1.0*dth))
        if dist_to_target < 0.08:
            vx *= dist_to_target / 0.08
            vy *= dist_to_target / 0.08

        # === Step 3: 강체 기구학 분배 ===
        front_vel = (vx, vy + om*self.half_L, om)
        rear_vel = (vx, vy - om*self.half_L, om)

        # === Step 4: 거리 오차 PID 보정 ===
        dist_error = fused_dist - self.wheelbase
        rear_corr = self.rear_pid.compute(dist_error, self.dt)

        # === Step 5: 로봇 이동 ===
        self.front.move(*front_vel, self.dt)
        self.rear.move(rear_vel[0] + rear_corr, rear_vel[1],
                       rear_vel[2], self.dt)

        self.step += 1
        self._last = {
            'enc': enc_dist*100,
            'aruco': aruco_dist*100 if aruco_visible else None,
            'fused': fused_dist*100,
            'true': self.get_true_distance()*100,
            'visible': aruco_visible,
            'target_dist': dist_to_target*100,
        }
        return True

    # ===== OpenCV 시각화 =====
    def render(self):
        if not CV2_OK:
            return True

        SC = 400  # 1m = 400px
        W, H = int(1.2*SC), int(1.0*SC)
        img = np.full((H, W, 3), (30, 30, 40), dtype=np.uint8)

        def to_px(x, y):
            return int(x*SC), int(H - y*SC)  # y 뒤집기

        # 목표
        tx, ty, _ = self.target
        tp = to_px(tx, ty)
        cv2.drawMarker(img, tp, (0, 200, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(img, "TARGET", (tp[0]+12, tp[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1)

        # 실제 위치 (ground truth) - 흐리게
        ftp = to_px(self.front.true_x, self.front.true_y)
        rtp = to_px(self.rear.true_x, self.rear.true_y)
        cv2.line(img, ftp, rtp, (80, 80, 80), 1)
        cv2.circle(img, ftp, 4, (100, 100, 100), -1)
        cv2.circle(img, rtp, 4, (100, 100, 100), -1)

        # 엔코더 추정 위치 - 진하게
        fep = to_px(self.front.enc_x, self.front.enc_y)
        rep = to_px(self.rear.enc_x, self.rear.enc_y)
        cv2.line(img, fep, rep, (200, 150, 100), 2)

        # 앞 로봇 (주황)
        cv2.circle(img, fep, 14, (240, 160, 50), -1)
        cv2.circle(img, fep, 14, (255, 255, 255), 1)
        cv2.putText(img, "F", (fep[0]-5, fep[1]+5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # 뒤 로봇 (초록)
        cv2.circle(img, rep, 14, (50, 200, 50), -1)
        cv2.circle(img, rep, 14, (255, 255, 255), 1)
        cv2.putText(img, "R", (rep[0]-5, rep[1]+5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # 마커 시야 표시 (앞→뒤 점선이면 보임)
        info = getattr(self, '_last', {})
        if info.get('visible'):
            cv2.line(img, fep, rep, (0, 255, 255), 1, cv2.LINE_AA)

        # 상태 텍스트
        y0 = 25
        lines = [
            f"Step {self.step} | {self.state}",
            f"True dist:    {info.get('true', 0):.1f} cm (target: 25.0)",
            f"Encoder:      {info.get('enc', 0):.1f} cm (drifting)",
            f"ArUco:        {info.get('aruco') if info.get('aruco') else 'NOT VISIBLE'}" +
                (" cm" if info.get('aruco') else ""),
            f"Fused(Kalman):{info.get('fused', 0):.1f} cm",
            f"Marker: {'VISIBLE' if info.get('visible') else 'HIDDEN (encoder only)'}",
        ]
        colors = [(255,255,255), (150,150,150), (200,150,100),
                  (0,255,255) if info.get('visible') else (0,100,200),
                  (100,255,150), (0,255,0) if info.get('visible') else (0,100,200)]
        for i, (line, col) in enumerate(zip(lines, colors)):
            cv2.putText(img, line, (10, y0 + i*22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)

        # 범례
        cv2.putText(img, "gray=true  orange/green=encoder est.",
                    (10, H-15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150,150,150), 1)

        cv2.imshow("ArUco Sync Simulation", img)
        key = cv2.waitKey(20) & 0xFF
        return key != ord('q')

    def run(self):
        print("=" * 55)
        print("  ArUco 융합 동기화 시뮬레이션")
        print("  q 키로 종료")
        print("=" * 55)
        print("  엔코더는 드리프트로 점점 틀어지지만,")
        print("  ArUco 보정으로 융합 거리는 실제값(25cm)에 가깝게 유지\n")

        running = True
        while running and self.state == 'MOVING':
            ok = self.control_step()
            if not ok:
                break
            running = self.render()

            # 콘솔 출력 (1초마다)
            if self.step % 50 == 0:
                info = self._last
                print(f"  Step {self.step:3d} | "
                      f"실제:{info['true']:.1f} "
                      f"엔코더:{info['enc']:.1f} "
                      f"융합:{info['fused']:.1f}cm | "
                      f"{'마커O' if info['visible'] else '마커X'}")

        # 최종 통계
        self.print_summary()
        if CV2_OK:
            cv2.waitKey(1500)
            cv2.destroyAllWindows()

    def print_summary(self):
        print("\n" + "=" * 55)
        print("  시뮬레이션 결과")
        print("=" * 55)

        # 엔코더만 썼을 때 vs 융합했을 때 오차 비교
        enc_errors = [abs(e - t) for e, t in
                      zip(self.history['encoder'], self.history['true'])]
        fused_errors = [abs(f - t) for f, t in
                        zip(self.history['fused'], self.history['true'])]

        print(f"  최종 상태: {self.state} ({self.step} 스텝)")
        print(f"\n  거리 추정 오차 (실제값 대비):")
        print(f"    엔코더만:  평균 {np.mean(enc_errors)*1000:.1f}mm, "
              f"최대 {np.max(enc_errors)*1000:.1f}mm")
        print(f"    칼만 융합: 평균 {np.mean(fused_errors)*1000:.1f}mm, "
              f"최대 {np.max(fused_errors)*1000:.1f}mm")

        improvement = (1 - np.mean(fused_errors)/np.mean(enc_errors)) * 100
        print(f"\n  → 융합으로 오차 {improvement:.0f}% 감소")

        visible_count = sum(1 for a in self.history['aruco'] if a is not None)
        print(f"  → 마커 가시율: "
              f"{visible_count/len(self.history['aruco'])*100:.0f}% "
              f"(안 보일 때도 엔코더로 정상 주행)")


def main():
    random.seed(42)  # 재현 가능한 결과
    sim = ArucoSyncSim()
    sim.run()


if __name__ == '__main__':
    main()