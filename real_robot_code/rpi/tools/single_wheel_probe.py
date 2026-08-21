#!/usr/bin/env python3
"""잭업 상태에서 STM32 모터 출력 한 채널만 짧게 확인한다."""

import argparse
import time

import serial


DEFAULT_PORT = (
    "/dev/serial/by-id/"
    "usb-STMicroelectronics_STM32_STLink_0667FF485270535067112920-if02"
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("wheel", choices=("FL", "FR", "RL", "RR"))
    parser.add_argument("--pwm", type=int, default=120)
    parser.add_argument("--seconds", type=float, default=1.0)
    parser.add_argument("--port", default=DEFAULT_PORT)
    args = parser.parse_args()

    if args.pwm == 0 or abs(args.pwm) > 120:
        parser.error("--pwm must be between -120 and 120, excluding 0")
    if not 0.2 <= args.seconds <= 2.0:
        parser.error("--seconds must be between 0.2 and 2.0")

    frame = f"@M,{args.wheel},{args.pwm}\n".encode("ascii")
    stop = b"@M,STOP\n"
    deadline = time.monotonic() + args.seconds

    with serial.Serial(args.port, 115200, timeout=0.02) as stm32:
        stm32.reset_input_buffer()
        try:
            while time.monotonic() < deadline:
                stm32.write(frame)
                time.sleep(0.05)
                for line in stm32.read_all().decode("ascii", errors="replace").splitlines():
                    if line.startswith(("T,", "ERR,")):
                        print(line)
        finally:
            stm32.write(stop)
            stm32.flush()
            time.sleep(0.05)
            stm32.write(b"X")
            stm32.flush()


if __name__ == "__main__":
    main()
