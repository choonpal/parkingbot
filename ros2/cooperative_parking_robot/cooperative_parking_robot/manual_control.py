"""ROS keyboard teleop의 순수 상태 로직.

rclpy 없이 테스트할 수 있도록 키 매핑과 auto/manual 명령 선택을 분리한다.
"""


ZERO_VELOCITY = (0.0, 0.0, 0.0)

# 실차 벤치에서 검증된 수동 조작 속도. STM32의 MANUAL_WHEEL_RAD_S(12rpm,
# 1.2566 rad/s)와 같은 바퀴 속도가 되도록 역산했다. 이보다 낮추면 펌웨어의
# feedforward PWM이 200 아래로 떨어져 정상 구동 구간(186~216)을 벗어나고,
# 모터가 힘없이 덜컥거린다.
#   linear  = 1.2566 * WHEEL_RADIUS(0.05)            = 0.0628 m/s
#   angular = 1.2566 * WHEEL_RADIUS / (LX + LY)(0.2) = 0.3142 rad/s
DEFAULT_LINEAR_SPEED_MPS = 0.0628
DEFAULT_ANGULAR_SPEED_RPS = 0.3142


class KeyboardTeleopState:
    def __init__(self, linear_speed=DEFAULT_LINEAR_SPEED_MPS,
                 angular_speed=DEFAULT_ANGULAR_SPEED_RPS,
                 deadman_s=0.30):
        self.linear_speed = float(linear_speed)
        self.angular_speed = float(angular_speed)
        self.deadman_s = float(deadman_s)
        if self.linear_speed <= 0.0 or self.angular_speed <= 0.0:
            raise ValueError('teleop speeds must be positive')
        if self.deadman_s <= 0.0:
            raise ValueError('deadman_s must be positive')
        self._velocity = ZERO_VELOCITY
        self._deadline = 0.0

    def handle_key(self, key, now):
        """키를 적용하고 선택적 gripper action을 반환한다."""
        movement = {
            'w': (self.linear_speed, 0.0, 0.0),
            's': (-self.linear_speed, 0.0, 0.0),
            'a': (0.0, self.linear_speed, 0.0),
            'd': (0.0, -self.linear_speed, 0.0),
            'q': (0.0, 0.0, self.angular_speed),
            'e': (0.0, 0.0, -self.angular_speed),
        }
        key = key.lower()
        if key in movement:
            self._velocity = movement[key]
            self._deadline = float(now) + self.deadman_s
            return None
        if key == ' ':
            self.stop()
            return None
        if key == 't':
            self.stop()
            return 'grip'
        if key == 'g':
            self.stop()
            return 'release'
        return None

    def velocity(self, now):
        if float(now) >= self._deadline:
            self.stop()
        return self._velocity

    def stop(self):
        self._velocity = ZERO_VELOCITY
        self._deadline = 0.0


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
