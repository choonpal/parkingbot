#!/usr/bin/env python3
"""PID 제어기 (거리/yaw 오차 보정용)"""


class PID:
    def __init__(self, Kp, Ki, Kd, out_limit=0.05):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.integral = 0.0
        self.prev = 0.0
        self.out_limit = out_limit

    def compute(self, error, dt):
        p = self.Kp * error
        self.integral += error * dt
        self.integral = max(-1.0, min(1.0, self.integral))  # anti-windup
        i = self.Ki * self.integral
        d = self.Kd * (error - self.prev) / dt if dt > 0 else 0.0
        self.prev = error
        out = p + i + d
        return max(-self.out_limit, min(self.out_limit, out))

    def reset(self):
        self.integral = 0.0
        self.prev = 0.0
