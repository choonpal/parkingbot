"""Phase-scoped ultrasonic activation and debounce state."""


class UltrasonicPhaseHealth:
    SIDES = ('left', 'right')

    def __init__(self, *, required_valid_samples, invalid_samples_to_drop,
                 max_sample_age_s, activation_timeout_s):
        if required_valid_samples <= 0 or invalid_samples_to_drop <= 0:
            raise ValueError('ultrasonic sample counts must be positive')
        if max_sample_age_s <= 0.0:
            raise ValueError('ultrasonic max sample age must be positive')
        if activation_timeout_s <= max_sample_age_s:
            raise ValueError(
                'ultrasonic activation timeout must exceed sample age')
        self.required_valid_samples = int(required_valid_samples)
        self.invalid_samples_to_drop = int(invalid_samples_to_drop)
        self.max_sample_age_s = float(max_sample_age_s)
        self.activation_timeout_s = float(activation_timeout_s)
        self.generation = 0
        self.reset()

    def reset(self):
        self.enabled_requested = False
        self.enabled_acknowledged = False
        self.ready = False
        self.started_at = 0.0
        self.valid_count = {side: 0 for side in self.SIDES}
        self.invalid_count = {side: 0 for side in self.SIDES}
        self.last_valid_at = {side: 0.0 for side in self.SIDES}

    def start(self, now):
        self.generation += 1
        self.reset()
        self.enabled_requested = True
        self.started_at = float(now)
        return self.generation

    def acknowledge_enabled(self, now):
        if not self.enabled_requested:
            return False
        self.enabled_acknowledged = True
        self.started_at = float(now)
        return True

    def disable(self):
        self.generation += 1
        self.reset()

    def observe(self, side, valid, now):
        if side not in self.SIDES:
            raise ValueError(f'unknown ultrasonic side: {side}')
        if not self.enabled_acknowledged:
            return self.ready
        now = float(now)
        if valid:
            self.invalid_count[side] = 0
            self.valid_count[side] += 1
            self.last_valid_at[side] = now
        else:
            self.valid_count[side] = 0
            self.invalid_count[side] += 1
            if self.invalid_count[side] >= self.invalid_samples_to_drop:
                self.ready = False
        if all(self.valid_count[side] >= self.required_valid_samples
               for side in self.SIDES):
            self.ready = True
        return self.ready

    def update(self, now):
        now = float(now)
        if not self.enabled_acknowledged:
            return self.ready
        if self.ready and any(
                stamp <= 0.0 or now - stamp > self.max_sample_age_s
                for stamp in self.last_valid_at.values()):
            self.ready = False
        return self.ready

    def activation_timed_out(self, now):
        return bool(
            self.enabled_requested and not self.ready and
            float(now) - self.started_at >= self.activation_timeout_s)
