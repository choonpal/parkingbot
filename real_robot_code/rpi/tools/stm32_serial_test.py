#!/usr/bin/env python3

import sys
import time

import serial


PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
TEST_SECONDS = 5.0
TELEMETRY_FIELD_COUNT = 14


def format_telemetry(fields: list[str]) -> str:
    return (
        f"cmd={fields[1]} "
        f"rpm=[{', '.join(fields[2:6])}] "
        f"pwm=[{', '.join(fields[6:10])}] "
        f"servo_us=[{', '.join(fields[10:12])}] "
        f"ultrasonic_mm=[{', '.join(fields[12:14])}]"
    )


def main() -> int:
    received = 0

    try:
        with serial.Serial(PORT, BAUD_RATE, timeout=0.4) as stm32:
            time.sleep(0.5)
            stm32.reset_input_buffer()

            print(f"Connected to {PORT} at {BAUD_RATE} baud")
            print("Sending STOP only. Motors must remain stopped.")

            end_time = time.monotonic() + TEST_SECONDS
            next_stop = 0.0

            while time.monotonic() < end_time:
                now = time.monotonic()

                if now >= next_stop:
                    stm32.write(b"X")
                    stm32.flush()
                    next_stop = now + 0.1

                raw = stm32.readline()
                if not raw:
                    continue

                text = raw.decode("ascii", errors="replace").strip()
                fields = text.split(",")

                if (
                    len(fields) != TELEMETRY_FIELD_COUNT
                    or fields[0] != "T"
                ):
                    print(f"Unrecognized: {text}")
                    continue

                received += 1
                print(f"OK {format_telemetry(fields)}")

    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 1

    if received == 0:
        print("No STM32 telemetry received.", file=sys.stderr)
        return 1

    print(f"PASS: received {received} telemetry packets.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
