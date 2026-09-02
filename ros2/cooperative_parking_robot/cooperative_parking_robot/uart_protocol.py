#!/usr/bin/env python3
"""RPi와 STM32 사이의 줄 단위 UART 프로토콜.

RPi → STM32 (기존 단일문자 시험 명령과 충돌하지 않도록 @ prefix 사용)
  @HELLO,2,session_id
  @V,vx,vy,w
  @V,0.000,0.000,0.000,session_id  (startup command-channel probe)
  @S,attach,pulse1_us,pulse2_us
  @S,grip | @S,release
  @HB,session_id:sequence
  @U,ON | @U,OFF
  @ESTOP

STM32 → RPi
  E,fl,fr,rl,rr
  U,L,distance_mm | U,R,distance_mm
  U,L,TIMEOUT    | U,R,TIMEOUT
  LIFT,status
  ACK,value
  ERR,code

초음파 거리는 STM32에서 마이크로초 펄스 폭으로 측정해 mm 정수로 보낸다.
RPi는 이 프레임을 sensor_msgs/Range로 변환하고, 바퀴 에지·중심 판단은
ultrasonic_edge_node에서 수행한다.
"""

import math
import re


# STM32 BAD_SERVO_ATTACH guard와 동일한 hobby-servo pulse 계약.
# 실차 open pulse가 이 범위의 양끝을 사용한다.
SERVO_PULSE_MIN_US = 400
SERVO_PULSE_MAX_US = 2600

# Version 2 means that HELLO is mandatory, HB ACKs are session-bound, and the
# tokenized startup zero-velocity frame receives ACK,V:<session_id>.
PROTOCOL_VERSION = 2
UART_BAUD_RATE = 115200
PROTOCOL_CAPABILITIES = frozenset({
    'SESSION_HELLO', 'SESSION_HEARTBEAT_ACK', 'ZERO_V_ACK', 'SERVO_ATTACH',
    'ULTRASONIC_CONTROL',
})
SESSION_ID_PATTERN = re.compile(r'^[A-Za-z0-9_-]{8,16}$')
ACK_TOKEN_PATTERN = re.compile(r'^[A-Za-z0-9_.:-]{1,48}$')


class UartProtocol:
    ULTRASONIC_SIDE = {'L': 'left', 'R': 'right'}

    def encode_velocity(self, vx, vy, w):
        return f"@V,{vx:.3f},{vy:.3f},{w:.3f}\n"

    @staticmethod
    def validate_session_id(session_id):
        if not isinstance(session_id, str) or not SESSION_ID_PATTERN.fullmatch(
                session_id):
            raise ValueError(
                'session_id must be 8-16 ASCII letters, digits, _ or -')
        return session_id

    def encode_hello(self, session_id):
        session_id = self.validate_session_id(session_id)
        return f"@HELLO,{PROTOCOL_VERSION},{session_id}\n"

    def hello_ack_value(self, session_id):
        session_id = self.validate_session_id(session_id)
        return f"HELLO:{PROTOCOL_VERSION}:{session_id}"

    def encode_zero_velocity(self, session_id):
        """Initialize the command watchdog and request a session-bound ACK."""
        session_id = self.validate_session_id(session_id)
        return f"@V,0.000,0.000,0.000,{session_id}\n"

    def zero_velocity_ack_value(self, session_id):
        session_id = self.validate_session_id(session_id)
        return f"V:{session_id}"

    def encode_servo(self, action):
        if action not in ('grip', 'release'):
            raise ValueError("servo action must be 'grip' or 'release'")
        return f"@S,{action}\n"

    def encode_servo_attach(self, pulse1, pulse2):
        """STM32 servo RAM 기준값을 RPi가 알고 있는 pulse로 동기화한다."""
        pulses = (pulse1, pulse2)
        if any(isinstance(pulse, bool) or not isinstance(pulse, int)
               for pulse in pulses):
            raise ValueError('servo attach pulses must be integers')
        if any(not SERVO_PULSE_MIN_US <= pulse <= SERVO_PULSE_MAX_US
               for pulse in pulses):
            raise ValueError(
                'servo attach pulses must be between '
                f'{SERVO_PULSE_MIN_US} and {SERVO_PULSE_MAX_US} us')
        return f"@S,attach,{pulse1},{pulse2}\n"

    def encode_heartbeat(self, timestamp):
        if isinstance(timestamp, str):
            if not ACK_TOKEN_PATTERN.fullmatch(timestamp):
                raise ValueError('heartbeat token contains invalid characters')
            token = timestamp
        else:
            value = float(timestamp)
            if not math.isfinite(value):
                raise ValueError('heartbeat timestamp must be finite')
            token = f'{value:.3f}'
        return f"@HB,{token}\n"

    def encode_estop(self):
        return "@ESTOP\n"

    @staticmethod
    def encode_ultrasonic(enabled):
        return f"@U,{'ON' if bool(enabled) else 'OFF'}\n"

    def parse(self, line):
        """STM32 → RPi 응답을 엄격하게 파싱한다."""
        if not isinstance(line, str):
            return None
        parts = [part.strip() for part in line.strip().split(',')]
        if not parts or not parts[0]:
            return None
        tag = parts[0]
        if tag == 'E' and len(parts) == 5:
            try:
                return {
                    'type': 'encoder',
                    'values': [int(parts[i]) for i in range(1, 5)],
                }
            except ValueError:
                return None
        if tag == 'U' and len(parts) == 3:
            side = self.ULTRASONIC_SIDE.get(parts[1])
            if side is None:
                return None
            if parts[2] == 'TIMEOUT':
                return {
                    'type': 'ultrasonic',
                    'side': side,
                    'valid': False,
                    'status': 'TIMEOUT',
                    'distance_mm': None,
                    'distance_m': float('inf'),
                }
            try:
                distance_mm = int(parts[2])
            except ValueError:
                return None
            # UART 파손·단위 실수를 조기에 막는다. 실제 HC-SR04 운용 범위는
            # 펌웨어에서 20~4000 mm로 제한하지만 parser는 약간의 여유를 둔다.
            if not 1 <= distance_mm <= 10000:
                return None
            return {
                'type': 'ultrasonic',
                'side': side,
                'valid': True,
                'status': 'OK',
                'distance_mm': distance_mm,
                'distance_m': distance_mm / 1000.0,
            }
        if tag == 'LIFT' and len(parts) == 2 and parts[1]:
            return {'type': 'lift', 'status': parts[1]}
        if tag == 'ACK' and len(parts) == 2 and parts[1]:
            return {
                'type': 'ack',
                'value': parts[1],
            }
        if tag == 'ERR' and len(parts) == 2 and parts[1]:
            return {
                'type': 'error',
                'code': parts[1],
            }
        # 저수준 정비 도구와 공유하는 14-field 진단 telemetry.
        if tag == 'T' and len(parts) == 14 and len(parts[1]) == 1:
            try:
                return {
                    'type': 'telemetry',
                    'command': parts[1],
                    'rpm_x10': [int(value) for value in parts[2:6]],
                    'pwm': [int(value) for value in parts[6:10]],
                    'servo_us': [int(value) for value in parts[10:12]],
                    'ultrasonic_mm': [int(value) for value in parts[12:14]],
                }
            except ValueError:
                return None
        return None
