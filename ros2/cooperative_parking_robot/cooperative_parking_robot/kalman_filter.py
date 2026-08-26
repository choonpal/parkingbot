#!/usr/bin/env python3
"""칼만 필터 모듈 — 두 종류를 목적에 따라 구분해서 쓴다.

  ScalarKalman — Front/Rear "상대" 거리·yaw 동기 오차 보정 (기존, v1.5).
                 rigid_body_sync_node의 sync 레이어에서 그대로 사용.
  PoseEKF      — 로봇 1대의 "절대" world-frame pose(x,y,yaw) 추정 (v1.6 신규).
                 encoder delta(predict) + CCTV·ArUco 절대측정(correct)을 융합.
                 pose_fusion_node가 로봇별로 1개씩 인스턴스화해서 쓴다.

v1.5 수정: predict를 델타(변화량) 기반으로 변경.
  기존: predict(절대값) → 상태를 덮어써서 ArUco 보정이 매 사이클 소멸 (버그)
  수정: predict(델타)   → 상태를 전파하여 ArUco 보정이 유지됨
  → PoseEKF도 동일 원칙: predict()는 반드시 encoder DELTA만 받는다.
"""

import math


class ScalarKalman:
    """1D 칼만 필터 — 거리 또는 yaw 추정"""
    def __init__(self, init=0.0, Q=0.0001, R=0.0004):
        self.x = init
        self.P = 0.1
        self.Q = Q      # 프로세스 노이즈 (엔코더 델타 신뢰도)
        self.R = R      # 측정 노이즈 (ArUco)
        self._prev_raw = None

    def predict_delta(self, delta):
        """엔코더 '변화량'으로 상태 전파 (ArUco 보정 유지)"""
        self.x += delta
        self.P += self.Q

    def predict_from_raw(self, raw_value):
        """엔코더 절대값을 받아 내부적으로 델타 전파.
        (호출부 편의용 — 첫 호출은 초기화만)"""
        if self._prev_raw is None:
            self._prev_raw = raw_value
            return
        self.predict_delta(raw_value - self._prev_raw)
        self._prev_raw = raw_value

    # 하위 호환 (기존 호출부): 절대값 → 델타 전파로 동작
    def predict(self, raw_value):
        self.predict_from_raw(raw_value)

    def reset(self, value=0.0, raw_value=None, covariance=0.1):
        """새 임무 시작 시 실제 현재값으로 상태와 raw 기준을 함께 초기화한다."""
        self.x = float(value)
        self.P = float(covariance)
        self._prev_raw = float(value if raw_value is None else raw_value)

    def update(self, measured):
        """ArUco 측정 보정. 마커 안 보이면 호출 안 함"""
        K = self.P / (self.P + self.R)
        self.x += K * (measured - self.x)
        self.P = (1 - K) * self.P
        return K


# ===== 3x3 행렬 유틸 (numpy 미사용 — RPi에서 의존성 최소화) =====

def _mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def _mat_transpose(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def _mat_add(a, b):
    return [[a[i][j] + b[i][j] for j in range(3)] for i in range(3)]


def _mat_sub(a, b):
    return [[a[i][j] - b[i][j] for j in range(3)] for i in range(3)]


def _mat_inv3(m):
    """3x3 역행렬 (여인수 전개). S=P+R은 대칭 양정치라 항상 가역."""
    a, b, c = m[0]
    d, e, f = m[1]
    g, h, i = m[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-12:
        det = 1e-12 if det >= 0 else -1e-12
    inv_det = 1.0 / det
    return [
        [(e * i - f * h) * inv_det, (c * h - b * i) * inv_det, (b * f - c * e) * inv_det],
        [(f * g - d * i) * inv_det, (a * i - c * g) * inv_det, (c * d - a * f) * inv_det],
        [(d * h - e * g) * inv_det, (b * g - a * h) * inv_det, (a * e - b * d) * inv_det],
    ]


def _ang_norm(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class PoseEKF:
    """로봇 1대의 world-frame pose(x,y,yaw) EKF.

    predict(): encoder BODY-FRAME 델타만 받는다 (EncoderOdometry.update()의
               dx_body/dy_body/dtheta). 여기서 자기 자신의(=보정된) yaw로
               world 프레임으로 회전시키므로, 엔코더 자체의 yaw 드리프트가
               위치 추정에 전염되지 않는다.
    correct(): CCTV+ArUco 절대측정(x,y,yaw)으로 보정. Mahalanobis gate로
               오탐(마커 ID 오검출, occlusion 순간 글리치)을 걸러낸다.
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0,
                 pos_process_gain=0.05, yaw_process_gain=0.05,
                 pos_process_floor=0.0005, yaw_process_floor=0.001,
                 gate_chi2=11.34, max_reject_streak=5,
                 max_reacquire_pos_error=0.5,
                 max_reacquire_yaw_error=math.radians(45.0),
                 first_fix_max_position_m=10.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        # 초기 공분산 — 시동 직후엔 CCTV 첫 측정으로 빠르게 수렴하도록 크게 잡음
        self.P = [[0.25, 0.0, 0.0],
                  [0.0, 0.25, 0.0],
                  [0.0, 0.0, 1.0]]
        # 프로세스 노이즈 파라미터: 이동/회전량에 비례(슬립) + dt에 비례(바닥잡음)
        # 실측 전 placeholder — 엔코더 slip 실험(§calibration)으로 확정할 것
        self.k_pos = pos_process_gain     # 이동거리당 위치 노이즈 비율
        self.k_yaw = yaw_process_gain     # 회전각당 yaw 노이즈 비율
        self.pos_floor = pos_process_floor  # dt당 최소 위치 노이즈(분산)
        self.yaw_floor = yaw_process_floor  # dt당 최소 yaw 노이즈(분산)
        self.gate_chi2 = gate_chi2        # 3-DOF 카이제곱 99% ≈ 11.34
        self.max_reject_streak = max_reject_streak
        self.max_reacquire_pos_error = max_reacquire_pos_error
        self.max_reacquire_yaw_error = max_reacquire_yaw_error
        self._reject_streak = 0
        self.last_correction_accepted = True
        self.last_mahalanobis = 0.0
        self.forced_reacquire = False
        # v1.9-P1-2: 시동 직후 단 한 번도 보정이 반영되지 않은 상태를 구분한다.
        # init_* 파라미터는 도킹 홈의 명목값이라 실제 배치와 어긋날 수 있고,
        # 그 오차가 chi2 게이트(초기 P=0.25 기준 위치 약 1.7m)를 넘으면
        # reject streak -> 강제 재수렴 경로마저 reacquire 한계(0.5m/45deg)에
        # 막혀 필터가 영구 미수렴 상태로 고착됐다. 첫 유효 측정은 예측이 아니라
        # 초기화로 취급하는 것이 옳다.
        self._ever_accepted = False
        # 게이트를 완전히 비우면 첫 프레임의 마커 오검출이 그대로 초기 pose가
        # 된다. 대신 "작업공간 안에 있는 값인가"라는 약한 타당성 검사만 남긴다.
        # 주차장 전체가 6x4m이므로 기본 10m는 정상 배치를 절대 막지 않으면서
        # 명백한 쓰레기 좌표는 걸러낸다.
        self.first_fix_max_position_m = float(first_fix_max_position_m)

    def predict(self, dx_body, dy_body, dtheta, dt):
        """엔코더 델타로 상태 전파. dt는 이번 구간 경과시간(초, Q 스케일용)."""
        yaw_mid = self.yaw + dtheta / 2.0
        c, s = math.cos(yaw_mid), math.sin(yaw_mid)
        dx_w = dx_body * c - dy_body * s
        dy_w = dx_body * s + dy_body * c

        self.x += dx_w
        self.y += dy_w
        self.yaw = _ang_norm(self.yaw + dtheta)

        # 상태천이 자코비안 (yaw에 대한 편미분, 소각 근사로 yaw_mid 대신 yaw 사용)
        F = [[1.0, 0.0, -dy_w],
             [0.0, 1.0, dx_w],
             [0.0, 0.0, 1.0]]

        trans = math.hypot(dx_body, dy_body)
        q_pos = (self.k_pos * trans) ** 2 + (self.pos_floor * max(dt, 1e-3))
        q_yaw = (self.k_yaw * abs(dtheta)) ** 2 + (self.yaw_floor * max(dt, 1e-3))
        Q = [[q_pos, 0.0, 0.0], [0.0, q_pos, 0.0], [0.0, 0.0, q_yaw]]

        FP = _mat_mul(F, self.P)
        self.P = _mat_add(_mat_mul(FP, _mat_transpose(F)), Q)

    def correct(self, x_meas, y_meas, yaw_meas, R=None):
        """CCTV+ArUco 절대 pose 측정으로 보정.
        R: [[sx2,0,0],[0,sy2,0],[0,0,syaw2]] — 측정 노이즈 공분산 (미지정 시 기본값)
        반환: True(반영됨) / False(게이트 탈락 — 오탐으로 보고 무시)"""
        if R is None:
            R = [[0.02 ** 2, 0.0, 0.0],
                 [0.0, 0.02 ** 2, 0.0],
                 [0.0, 0.0, math.radians(3.0) ** 2]]

        # 첫 유효 측정은 게이트를 적용하지 않고 상태를 그대로 이식한다.
        # 게이트는 "이미 신뢰할 수 있는 추정치"가 있을 때 오탐을 거르는 장치이므로,
        # 신뢰할 추정치가 아직 없는 시점에 적용하면 초기 배치 오차만으로
        # 필터를 영구히 잠글 수 있다.
        if not self._ever_accepted:
            if (abs(float(x_meas)) > self.first_fix_max_position_m or
                    abs(float(y_meas)) > self.first_fix_max_position_m):
                # 작업공간 밖 좌표는 초기값으로도 받지 않는다.
                self._reject_streak += 1
                self.last_correction_accepted = False
                self.forced_reacquire = False
                return False
            self.x = float(x_meas)
            self.y = float(y_meas)
            self.yaw = _ang_norm(float(yaw_meas))
            self.P = [list(row) for row in R]
            self._reject_streak = 0
            self._ever_accepted = True
            self.last_correction_accepted = True
            self.last_mahalanobis = 0.0
            self.forced_reacquire = False
            return True

        ex = x_meas - self.x
        ey = y_meas - self.y
        eyaw = _ang_norm(yaw_meas - self.yaw)
        e = [ex, ey, eyaw]

        S = _mat_add(self.P, R)
        S_inv = _mat_inv3(S)

        # Mahalanobis distance^2 = e^T S^-1 e — 오탐 게이팅
        Se = [sum(S_inv[i][j] * e[j] for j in range(3)) for i in range(3)]
        d2 = sum(e[i] * Se[i] for i in range(3))
        self.last_mahalanobis = d2
        self.forced_reacquire = False
        if d2 > self.gate_chi2:
            self._reject_streak += 1
            # 연속 거부가 계속되면 "오탐 1회"가 아니라 "필터가 실제로 틀어짐"
            # (카메라 재캘리브레이션 필요, 장기 미검출 후 재획득 등)으로 보고
            # 강제로 재수렴시킨다. 그렇지 않으면 게이트가 영구히 잠겨 필터가
            # 계속 틀린 상태로 고착된다.
            if self._reject_streak < self.max_reject_streak:
                self.last_correction_accepted = False
                return False
            # 반복 관측이라도 물리적으로 터무니없는 좌표로 강제 점프시키지 않는다.
            # 더 큰 재배치는 명시적 재초기화 절차로 처리해야 한다.
            if (math.hypot(ex, ey) > self.max_reacquire_pos_error or
                    abs(eyaw) > self.max_reacquire_yaw_error):
                self._reject_streak = 0
                self.last_correction_accepted = False
                return False
            self.forced_reacquire = True
        self._reject_streak = 0

        K = _mat_mul(self.P, S_inv)
        dx = sum(K[0][j] * e[j] for j in range(3))
        dy = sum(K[1][j] * e[j] for j in range(3))
        dyaw = sum(K[2][j] * e[j] for j in range(3))
        self.x += dx
        self.y += dy
        self.yaw = _ang_norm(self.yaw + dyaw)

        # Joseph form: 수치적으로 더 안정적 (P가 반정치를 잃는 것 방지)
        identity = [[1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0]]
        identity_minus_gain = _mat_sub(identity, K)  # H=I이므로 KH=K
        term1 = _mat_mul(
            _mat_mul(identity_minus_gain, self.P),
            _mat_transpose(identity_minus_gain))
        term2 = _mat_mul(_mat_mul(K, R), _mat_transpose(K))
        self.P = _mat_add(term1, term2)

        self.last_correction_accepted = True
        return True

    def reset_reject_streak(self):
        '''Break reject continuity after marker loss or an invalid measurement.'''
        self._reject_streak = 0
        self.forced_reacquire = False

    @property
    def initialized(self):
        """True once at least one absolute measurement has been absorbed."""
        return self._ever_accepted

    def pose(self):
        return self.x, self.y, self.yaw

    def position_std(self):
        """대략적 위치 표준편차(m) — 상태/진단 로깅용"""
        return math.sqrt(max(self.P[0][0], 0.0)), math.sqrt(max(self.P[1][1], 0.0))

    def yaw_std(self):
        return math.sqrt(max(self.P[2][2], 0.0))
