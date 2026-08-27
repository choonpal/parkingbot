#!/usr/bin/env python3
"""PID controller used by rigid-body distance/lateral/yaw correction."""


class PID:
    def __init__(self, Kp, Ki, Kd, out_limit=0.05):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.integral = 0.0
        self.prev = 0.0
        self.out_limit = out_limit

    def compute(self, error, dt):
        error = float(error)
        # Rigid-body callers apply a deadband before PID. An exact zero means
        # the mechanical error is intentionally ignored; retaining integral
        # output here would keep pushing the vehicle after entering deadband.
        if abs(error) <= 1.0e-12:
            self.reset()
            return 0.0
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
