#!/usr/bin/env python3

import argparse
import time

import serial


BAUD_RATE = 115200
TELEMETRY_FIELD_COUNT = 14


def read_latest(stm32: serial.Serial, duration: float) -> list[str] | None:
    deadline = time.monotonic() + duration
    latest = None

    while time.monotonic() < deadline:
        raw = stm32.readline()
        if not raw:
            continue

        fields = raw.decode("ascii", errors="replace").strip().split(",")
        if len(fields) == TELEMETRY_FIELD_COUNT and fields[0] == "T":
            latest = fields

    return latest


def send(stm32: serial.Serial, command: str) -> None:
    stm32.write(command.encode("ascii"))
    stm32.flush()


def servo_values(fields: list[str] | None) -> tuple[int, int] | None:
    if fields is None:
        return None
    return int(fields[10]), int(fields[11])


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Move both servos one 50 us step closed, then return open."
    )
    parser.add_argument("--port", default="/dev/ttyACM0")
    args = parser.parse_args()

    expected_start = (400, 2600)
    expected_closed_step = (450, 2550)

    with serial.Serial(args.port, BAUD_RATE, timeout=0.1) as stm32:
        time.sleep(0.5)
        stm32.reset_input_buffer()

        try:
            send(stm32, "X")
            start = servo_values(read_latest(stm32, 0.5))

            send(stm32, "T")
            time.sleep(0.3)
            send(stm32, "X")
            closed_step = servo_values(read_latest(stm32, 0.5))

            send(stm32, "G")
            time.sleep(0.3)
            send(stm32, "X")
            returned_open = servo_values(read_latest(stm32, 0.5))
        finally:
            send(stm32, "X")

    print(f"START={start}")
    print(f"CLOSE_STEP={closed_step}")
    print(f"RETURN_OPEN={returned_open}")

    if (
        start == expected_start
        and closed_step == expected_closed_step
        and returned_open == expected_start
    ):
        print("PASS: paired close/open command and hold verified.")
        return 0

    print("FAIL: unexpected servo telemetry.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
