'''Pure helpers for mission-scoped JSON envelopes.'''

import math


def normalize_yaw(yaw):
    return math.atan2(math.sin(float(yaw)), math.cos(float(yaw)))


def make_arrival_status(x, y, yaw, plan_stamp_ns):
    values = (float(x), float(y), float(yaw))
    if not all(math.isfinite(value) for value in values):
        raise ValueError('final vehicle pose must be finite')
    stamp_ns = int(plan_stamp_ns)
    if stamp_ns <= 0:
        raise ValueError('plan_stamp_ns must be positive')
    return {
        'error': 'ARRIVED',
        'plan_stamp_ns': stamp_ns,
        'final_vehicle_pose': {
            'frame_id': 'map',
            'x': values[0],
            'y': values[1],
            'yaw': normalize_yaw(values[2]),
        },
    }


def parse_arrival_status(payload, expected_plan_stamp_ns):
    try:
        if payload.get('error') != 'ARRIVED':
            return None
        if int(payload['plan_stamp_ns']) != int(expected_plan_stamp_ns):
            return None
        pose = payload['final_vehicle_pose']
        if pose.get('frame_id') != 'map':
            return None
        values = (float(pose['x']), float(pose['y']), float(pose['yaw']))
    except (KeyError, TypeError, ValueError, AttributeError):
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    return values[0], values[1], normalize_yaw(values[2])
