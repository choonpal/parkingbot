#!/usr/bin/env python3

import argparse
import csv
import os
import select
import socket
import sys
import termios
import time
import tty

import serial


BAUD_RATE = 115200
COMMAND_REFRESH_SECONDS = 0.10
DRIVE_DEADMAN_SECONDS = 0.30
SERVO_DEADMAN_SECONDS = 0.30
TELEMETRY_FIELD_COUNT = 14

DRIVE_KEYS = {
    "w": "FORWARD",
    "s": "BACKWARD",
    "a": "STRAFE LEFT",
    "d": "STRAFE RIGHT",
    "q": "ROTATE LEFT",
    "e": "ROTATE RIGHT",
}

# Map operator-intent keys to each robot's installed wheel orientation.
# This applies to both lowercase open-loop and uppercase PID commands.
DRIVE_COMMAND_KEYS_BY_HOST = {
    "robot-1": {
        "w": "s",
        "s": "w",
        "a": "d",
        "d": "a",
        "q": "e",
        "e": "q",
    },
    "robot-2": {
        "w": "s",
        "s": "w",
        "a": "d",
        "d": "a",
        "q": "e",
        "e": "q",
    },
}

SERVO_KEYS = {
    "u": ("U", "SERVO 1 +"),
    "j": ("J", "SERVO 1 -"),
    "i": ("I", "SERVO 2 +"),
    "k": ("K", "SERVO 2 -"),
    "t": ("T", "BOTH SERVOS CLOSE"),
    "g": ("G", "BOTH SERVOS OPEN"),
    "o": ("O", "SERVO PWM OFF"),
}

SENSOR_KEYS = {
    "1": ("1", "ULTRASONIC 1"),
    "2": ("2", "ULTRASONIC 2"),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Safe Raspberry Pi keyboard control for the STM32 robot."
    )
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument(
        "--mode",
        choices=("open-loop", "pid"),
        default="open-loop",
        help="Drive control mode (default: fixed-PWM open-loop).",
    )
    parser.add_argument(
        "--log-csv",
        action="store_true",
        help="Record transmitted commands and all STM32 frames under ~/pid_logs.",
    )
    parser.add_argument(
        "--test-window",
        type=float,
        default=None,
        help=(
            "Run one tapped drive command for a bounded duration in seconds "
            "(0.2 to 2.0); Space stops and rearms the next test."
        ),
    )
    args = parser.parse_args()
    if args.test_window is not None and not 0.2 <= args.test_window <= 2.0:
        parser.error("--test-window must be between 0.2 and 2.0 seconds")
    return args


def format_telemetry(fields: list[str]) -> str:
    return (
        f"rpm=[{', '.join(fields[2:6])}] "
        f"pwm=[{', '.join(fields[6:10])}] "
        f"servo_us=[{', '.join(fields[10:12])}] "
        f"us_mm=[{', '.join(fields[12:14])}]"
    )


def send_command(stm32: serial.Serial, command: str) -> None:
    stm32.write(command.encode("ascii"))
    stm32.flush()


def main() -> int:
    args = parse_args()
    hostname = socket.gethostname().split(".", 1)[0]
    drive_command_keys = DRIVE_COMMAND_KEYS_BY_HOST.get(hostname)

    if drive_command_keys is None:
        print(
            f"Unsupported hostname: {hostname}. Expected robot-1 or robot-2.",
            file=sys.stderr,
        )
        return 1

    if not sys.stdin.isatty():
        print("This program must run in an interactive terminal.", file=sys.stderr)
        return 1

    old_terminal = termios.tcgetattr(sys.stdin)
    active_drive = "X"
    drive_deadline = 0.0
    servo_deadline = 0.0
    next_refresh = 0.0
    label = "STOP"
    rx_buffer = bytearray()
    test_drive_latched = False
    log_file = None
    log_writer = None
    log_path = None
    log_start = time.monotonic()

    if args.log_csv:
        log_directory = os.path.expanduser("~/pid_logs")
        os.makedirs(log_directory, exist_ok=True)
        log_path = os.path.join(
            log_directory,
            time.strftime("keyboard_pid_%Y%m%d_%H%M%S.csv"),
        )
        log_file = open(log_path, "w", newline="", encoding="utf-8")
        log_writer = csv.writer(log_file)
        log_writer.writerow(("elapsed_s", "event", "label", "command", "raw"))
        log_file.flush()

    def write_log(event: str, command: str = "", raw: str = "") -> None:
        if log_writer is None:
            return
        log_writer.writerow(
            (
                f"{time.monotonic() - log_start:.6f}",
                event,
                label,
                command,
                raw,
            )
        )
        log_file.flush()

    def send_logged(stm32: serial.Serial, command: str) -> None:
        send_command(stm32, command)
        write_log("TX", command=command)

    try:
        with serial.Serial(args.port, BAUD_RATE, timeout=0) as stm32:
            time.sleep(0.5)
            stm32.reset_input_buffer()
            send_logged(stm32, "X")

            tty.setcbreak(sys.stdin.fileno())

            print("Raspberry Pi STM32 keyboard control")
            print(f"Robot profile: {hostname}")
            print(f"Drive mode: {args.mode}")
            print("WASD: move, Q/E: rotate, Space: stop, Ctrl+C: exit")
            print(
                "U/J: servo 1, I/K: servo 2, "
                "T/G: both close/open, O: servo PWM off"
            )
            print("1/2: ultrasonic measurement")
            print(
                "Drive commands stop automatically after "
                f"{DRIVE_DEADMAN_SECONDS:.2f}s without another key event."
            )
            if args.test_window is not None:
                print(
                    f"Bounded test: tap one drive key for {args.test_window:.2f}s "
                    "maximum; Space stops and rearms."
                )
            print(
                "Servo motion holds its current position after "
                f"{SERVO_DEADMAN_SECONDS:.2f}s without another key event."
            )
            if log_path is not None:
                print(f"CSV log: {log_path}")

            while True:
                now = time.monotonic()
                readable, _, _ = select.select(
                    [sys.stdin, stm32.fileno()], [], [], 0.02
                )

                if sys.stdin in readable:
                    key = os.read(sys.stdin.fileno(), 1).decode(
                        "ascii", errors="ignore"
                    ).lower()

                    if key == "\x03":
                        raise KeyboardInterrupt

                    if key == " ":
                        active_drive = "X"
                        label = "STOP"
                        drive_deadline = 0.0
                        servo_deadline = 0.0
                        test_drive_latched = False
                        send_logged(stm32, "X")
                    elif key in DRIVE_KEYS:
                        if args.test_window is not None and test_drive_latched:
                            continue
                        command_key = drive_command_keys[key]
                        active_drive = (
                            command_key
                            if args.mode == "open-loop"
                            else command_key.upper()
                        )
                        label = DRIVE_KEYS[key]
                        if args.test_window is None:
                            drive_deadline = now + DRIVE_DEADMAN_SECONDS
                        else:
                            drive_deadline = now + args.test_window
                            test_drive_latched = True
                        servo_deadline = 0.0
                        next_refresh = 0.0
                    elif key in SERVO_KEYS:
                        command, label = SERVO_KEYS[key]
                        active_drive = "X"
                        drive_deadline = 0.0
                        send_logged(stm32, command)
                        servo_deadline = (
                            0.0
                            if command == "O"
                            else now + SERVO_DEADMAN_SECONDS
                        )
                    elif key in SENSOR_KEYS:
                        command, label = SENSOR_KEYS[key]
                        active_drive = "X"
                        drive_deadline = 0.0
                        servo_deadline = 0.0
                        send_logged(stm32, command)

                if active_drive != "X":
                    if now >= drive_deadline:
                        active_drive = "X"
                        label = (
                            "STOP (TEST LIMIT)"
                            if args.test_window is not None
                            else "STOP"
                        )
                        send_logged(stm32, "X")
                    elif now >= next_refresh:
                        send_logged(stm32, active_drive)
                        next_refresh = now + COMMAND_REFRESH_SECONDS

                if servo_deadline > 0.0 and now >= servo_deadline:
                    servo_deadline = 0.0
                    label = "SERVO HOLD"
                    send_logged(stm32, "X")

                if stm32.fileno() in readable:
                    rx_buffer.extend(stm32.read(stm32.in_waiting or 1))

                    while b"\n" in rx_buffer:
                        raw_line, _, remainder = rx_buffer.partition(b"\n")
                        rx_buffer = bytearray(remainder)
                        text = raw_line.decode(
                            "ascii", errors="replace"
                        ).strip()
                        write_log("RX", raw=text)
                        fields = text.split(",")

                        if (
                            len(fields) == TELEMETRY_FIELD_COUNT
                            and fields[0] == "T"
                        ):
                            status = format_telemetry(fields)
                            print(
                                f"\r{label:<16} {status:<100}",
                                end="",
                                flush=True,
                            )

    except KeyboardInterrupt:
        pass
    except serial.SerialException as exc:
        print(f"\nSerial error: {exc}", file=sys.stderr)
        return 1
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_terminal)
        try:
            with serial.Serial(args.port, BAUD_RATE, timeout=0.2) as stm32:
                send_command(stm32, "X")
        except serial.SerialException:
            pass
        if log_file is not None:
            log_file.close()
            print(f"CSV saved: {log_path}")
        print("\nSTOP sent. Serial port closed.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
