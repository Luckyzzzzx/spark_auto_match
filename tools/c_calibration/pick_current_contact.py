#!/usr/bin/env python3
"""Pump on at the verified contact point, then lift vertically in two stages."""

import re
import sys
import time

import serial


POSITION = re.compile(r"@3 X(-?\d+(?:\.\d+)?) Y(-?\d+(?:\.\d+)?) Z(-?\d+(?:\.\d+)?)")


def read_lines(port, seconds):
    deadline = time.monotonic() + seconds
    positions = []
    errors = []
    while time.monotonic() < deadline:
        line = port.readline().decode("ascii", errors="replace").strip()
        match = POSITION.search(line)
        if match:
            positions.append(tuple(float(value) for value in match.groups()))
        if " E" in line or line.startswith("E"):
            errors.append(line)
    return positions, errors


def move_and_confirm(port, x, y, z):
    port.reset_input_buffer()
    command = "G0 X{:.2f} Y{:.2f} Z{:.2f} F1200\r\n".format(x, y, z)
    print("COMMAND:", command.strip(), flush=True)
    port.write(command.encode("ascii"))
    deadline = time.monotonic() + 15.0
    last = None
    while time.monotonic() < deadline:
        positions, errors = read_lines(port, 0.4)
        if errors:
            raise RuntimeError("Controller rejected lift: " + "; ".join(errors))
        if positions:
            last = positions[-1]
            print("ACTUAL: x={:.2f} y={:.2f} z={:.2f}".format(*last), flush=True)
            if abs(last[2] - z) <= 3.0:
                return last
    raise RuntimeError("Lift not confirmed; last actual=" + repr(last))


def main():
    contact = (315.50, 21.33, 62.43)
    with serial.Serial("/dev/ttyACM0", 115200, timeout=0.2, exclusive=True) as port:
        time.sleep(2.0)
        port.reset_input_buffer()
        port.write(b"M2120 V0.2\r\n")
        positions, errors = read_lines(port, 2.0)
        if errors or not positions:
            raise RuntimeError("No actual position before pickup: " + repr(errors))
        actual = positions[-1]
        print("ACTUAL BEFORE: x={:.2f} y={:.2f} z={:.2f}".format(*actual), flush=True)
        if max(abs(actual[i] - contact[i]) for i in range(3)) > 5.0:
            raise RuntimeError("Arm moved away from verified contact point; refusing pickup")

        port.write(b"M17\r\n")
        port.write(b"M2231 V1\r\n")
        print("PUMP ON; waiting 2 seconds", flush=True)
        time.sleep(2.0)
        move_and_confirm(port, contact[0], contact[1], 82.0)
        final = move_and_confirm(port, contact[0], contact[1], 105.0)
        port.write(b"M2120 V0\r\n")
        print("PICKUP COMPLETE; pump remains ON; final={}".format(final), flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("ERROR:", exc, file=sys.stderr)
        sys.exit(1)
