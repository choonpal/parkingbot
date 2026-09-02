"""ROS-independent automatic/commissioning command arbitration."""


ZERO_VELOCITY = (0.0, 0.0, 0.0)

class VelocityCommandArbiter:
    """수동 모드가 자동주행보다 항상 우선하는 fail-safe selector."""

    def __init__(self, manual_timeout_s=0.25, release_guard_s=0.50):
        self.manual_timeout_s = float(manual_timeout_s)
        self.release_guard_s = float(release_guard_s)
        self.manual_enabled = False
        self.auto_command = ZERO_VELOCITY
        self.auto_time = 0.0
        self.manual_command = ZERO_VELOCITY
        self.manual_time = 0.0
        self.release_guard_until = 0.0

    def update_auto(self, command, now):
        self.auto_command = tuple(float(value) for value in command)
        self.auto_time = float(now)

    def update_manual(self, command, now):
        if not self.manual_enabled:
            return False
        self.manual_command = tuple(float(value) for value in command)
        self.manual_time = float(now)
        return True

    def set_manual_enabled(self, enabled, now):
        now = float(now)
        enabled = bool(enabled)
        if enabled:
            if not self.manual_enabled:
                self.manual_command = ZERO_VELOCITY
                self.manual_time = now
            self.manual_enabled = True
            return
        if self.manual_enabled:
            self.release_guard_until = now + self.release_guard_s
        self.manual_enabled = False
        self.auto_command = ZERO_VELOCITY
        self.auto_time = now

    def force_zero(self, now):
        now = float(now)
        self.auto_command = ZERO_VELOCITY
        self.auto_time = now
        self.manual_command = ZERO_VELOCITY
        self.manual_time = now

    def output(self, now):
        now = float(now)
        if self.manual_enabled:
            if now - self.manual_time > self.manual_timeout_s:
                return ZERO_VELOCITY
            return self.manual_command
        if now < self.release_guard_until:
            return ZERO_VELOCITY

        age = now - self.auto_time
        vx, vy, w = self.auto_command
        if age > 0.5:
            return ZERO_VELOCITY
        if age > 0.2:
            decay = 1.0 - (age - 0.2) / 0.3
            return vx * decay, vy * decay, w * decay
        return vx, vy, w
