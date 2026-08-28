"""Ultrasonic stream-health helpers shared by the production STM32 bridge."""

ULTRASONIC_SIDES = ('left', 'right')


def mark_ultrasonic_frame(frame_stamps, side, now):
    """Record receipt of a UART ultrasonic frame, regardless of echo validity."""
    if side not in ULTRASONIC_SIDES:
        raise ValueError(f'unknown ultrasonic side: {side}')
    frame_stamps[side] = float(now)


def stale_ultrasonic_sides(frame_stamps, now, timeout_s):
    """Return sensors whose UART stream has not produced a recent frame."""
    now = float(now)
    timeout_s = float(timeout_s)
    if timeout_s <= 0.0:
        raise ValueError('ultrasonic stream timeout must be positive')
    return [
        side for side in ULTRASONIC_SIDES
        if float(frame_stamps.get(side, 0.0)) <= 0.0 or
        now - float(frame_stamps.get(side, 0.0)) >= timeout_s
    ]


def ultrasonic_streams_fresh(frame_stamps, now, timeout_s, required=True):
    """True when both sensor streams are alive; TIMEOUT frames count as alive."""
    if not required:
        return True
    return not stale_ultrasonic_sides(frame_stamps, now, timeout_s)
