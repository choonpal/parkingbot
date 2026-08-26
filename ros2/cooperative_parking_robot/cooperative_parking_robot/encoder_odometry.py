#!/usr/bin/env python3
"""엔코더 카운트 → odometry (메카넘 정기구학)

v1.6: 누적 pose와 별개로 이번 주기의 body-frame 델타(dx,dy,dtheta)도
반환한다. PoseEKF의 predict()는 반드시 이 델타만 사용해야 한다 —
누적 x/y/theta를 predict에 흘려보내면 "칼만 predict 덮어쓰기" 버그가
재발한다 (CLAUDE.md §4 참조).

v1.7: 카운터 불연속(STM32 재부팅으로 누적 카운트가 0으로 리셋, 또는
하드웨어 카운터 rollover) 방어 추가. 이전엔 counts[i]-prev[i]를 그대로
믿었기 때문에, 예를 들어 10000→0으로 리셋되면 -10000틱이 "실제로 순간
이동한 것"처럼 해석돼 한 주기에 수십cm 위치 점프가 발생했다.
"""
import math


class EncoderOdometry:
    def __init__(self, wheel_radius=0.05, ppr=5182.0, lx=0.10, ly=0.10,
                 max_delta_ticks=2591, axis_sign=(1.0, 1.0, 1.0)):
        self.r = float(wheel_radius)
        self.ppr = float(ppr)
        self.L = float(lx) + float(ly)
        self.max_delta_ticks = int(max_delta_ticks)
        self.axis_sign = tuple(float(value) for value in axis_sign)
        if self.r <= 0.0:
            raise ValueError('wheel_radius must be positive')
        if self.ppr <= 0.0:
            raise ValueError('ppr must be positive')
        if self.L <= 0.0:
            raise ValueError('lx + ly must be positive')
        if self.max_delta_ticks <= 0:
            raise ValueError('max_delta_ticks must be positive')
        if (len(self.axis_sign) != 3 or
                any(value not in (-1.0, 1.0) for value in self.axis_sign)):
            raise ValueError('axis_sign must contain three +1/-1 values')
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev = None
        # 한 주기(엔코더 메시지 간)에 물리적으로 나올 수 없는 틱 변화량.
        # 기본값 2591 = ppr(5182)의 0.5바퀴 — 로봇 최대속도(§sync_params
        # max_speed 0.08m/s) 기준 정상 주기당 변화량(~수십 틱)의 50배 이상
        # 여유를 둔 값이라 정상 주행/슬립은 절대 안 걸리고, 리셋/rollover
        # 같은 진짜 불연속만 걸러진다. 실측 후 필요시 조정.

    def update(self, counts):
        """counts: [fl, fr, rl, rr] 엔코더 누적 카운트.

        반환: {'x','y','theta'}          — 순수 dead-reckoning 누적 pose (진단용)
              {'dx_body','dy_body','dtheta'} — 이번 호출分 BODY-FRAME 델타 (Kalman predict용)
              {'discontinuity'}          — True면 이번 주기는 카운터 리셋/rollover로
                                            판단해 델타를 0으로 버리고 재동기화만 함

        dx_body/dy_body는 일부러 world-frame으로 회전시키지 않는다.
        여기서 self.theta(드리프트 누적치)로 회전시켜 버리면 그 오차가
        델타에 섞여 들어가 PoseEKF에 전달된다. world 프레임 회전은
        PoseEKF가 "자기 자신의(=CCTV로 보정된) yaw 추정치"로 직접 해야
        엔코더 yaw 드리프트가 위치 추정까지 전염되지 않는다.
        """
        zero = {'x': self.x, 'y': self.y, 'theta': self.theta,
                'dx_body': 0.0, 'dy_body': 0.0, 'dtheta': 0.0,
                'discontinuity': False}
        if self.prev is None:
            self.prev = counts
            return zero

        raw_delta = [counts[i] - self.prev[i] for i in range(4)]
        if any(abs(v) > self.max_delta_ticks for v in raw_delta):
            # 카운터 리셋/rollover로 판단 — 이번 주기 모션은 버리고 새 기준점으로만 재동기화
            self.prev = counts
            out = dict(zero)
            out['discontinuity'] = True
            return out

        # 각 바퀴 회전량 (rad)
        d = [v / self.ppr * 2 * math.pi for v in raw_delta]
        self.prev = counts

        # 메카넘 정기구학 (역기구학의 역) — 이번 주기 body-frame 변위
        fl, fr, rl, rr = d
        dx_body = (fl + fr + rl + rr) * self.r / 4
        dy_body = (-fl + fr + rl - rr) * self.r / 4
        dtheta = (-fl + fr - rl + rr) * self.r / (4 * self.L)

        # 위 정기구학 결과는 STM32/기체 장착축이다. 명령에 적용한 것과 같은
        # +/-1 변환은 자기 자신의 역변환이므로 ROS body 축으로 되돌릴 때도
        # 동일하게 곱한다. 이를 빼먹으면 물리 전진에서 wheel odom이 음수가 된다.
        sx, sy, sw = self.axis_sign
        dx_body *= sx
        dy_body *= sy
        dtheta *= sw

        # 글로벌 적분 (내부 진단용 누적치 — Kalman predict 입력으로 쓰지 말 것)
        # 중점 heading으로 적분해 회전 중 호(arc) 근사 오차를 줄인다.
        theta_mid = self.theta + dtheta / 2.0
        self.x += dx_body * math.cos(theta_mid) - dy_body * math.sin(theta_mid)
        self.y += dx_body * math.sin(theta_mid) + dy_body * math.cos(theta_mid)
        self.theta += dtheta
        return {'x': self.x, 'y': self.y, 'theta': self.theta,
                'dx_body': dx_body, 'dy_body': dy_body, 'dtheta': dtheta,
                'discontinuity': False}
