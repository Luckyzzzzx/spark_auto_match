#!/usr/bin/env python3
"""Return the held block to the verified contact point, release, and lift."""

import re
import sys
import time

import serial


POSITION = re.compile(r"@3 X(-?\d+(?:\.\d+)?) Y(-?\d+(?:\.\d+)?) Z(-?\d+(?:\.\d+)?)")
X, Y, CONTACT_Z = 315.50, 21.33, 62.43


def read_lines(port, seconds):
    deadline = time.monotonic() + seconds
    positions, errors = [], []
    while time.monotonic() < deadline:
        line = port.readline().decode("ascii", errors="replace").strip()
        match = POSITION.search(line)
        if match:
            positions.append(tuple(float(value) for value in match.groups()))
        if " E" in line or line.startswith("E"):
            errors.append(line)
    return positions, errors


def move_and_confirm(port, z):
    port.reset_input_buffer()
    command = "G0 X{:.2f} Y{:.2f} Z{:.2f} F1000\r\n".format(X, Y, z)
    print("COMMAND:", command.strip(), flush=True)
    port.write(command.encode("ascii"))
    deadline = time.monotonic() + 18.0
    last = None
    while time.monotonic() < deadline:
        positions, errors = read_lines(port, 0.4)
        if errors:
            raise RuntimeError("Controller rejected movement: " + "; ".join(errors))
        if positions:
            last = positions[-1]
            print("ACTUAL: x={:.2f} y={:.2f} z={:.2f}".format(*last), flush=True)
            if abs(last[2] - z) <= 2.0:
                return last
    raise RuntimeError("Movement not confirmed; last actual=" + repr(last))


def main():
    with serial.Serial("/dev/ttyACM0", 115200, timeout=0.2, exclusive=True) as port:
        time.sleep(2.0)
        port.reset_input_buffer()
        port.write(b"M2120 V0.2\r\n")
        positions, errors = read_lines(port, 2.0)
        if errors or not positions:
            raise RuntimeError("No actual position before placement: " + repr(errors))
        actual = positions[-1]
        print("ACTUAL BEFORE: x={:.2f} y={:.2f} z={:.2f}".format(*actual), flush=True)
        if abs(actual[0] - X) > 5 or abs(actual[1] - Y) > 5 or actual[2] < 90:
            raise RuntimeError("Arm is not at the verified lifted pose; refusing placement")

        port.write(b"M17\r\n")
        # Opening the serial port can reset the controller's output state.
        # Reassert suction before any descent so the block cannot be released in air.
        port.write(b"M2231 V1\r\n")
        print("PUMP ON confirmed before descent", flush=True)
        time.sleep(1.0)
        move_and_confirm(port, 78.0)
        move_and_confirm(port, CONTACT_Z)
        port.write(b"M2231 V0\r\n")
        print("PUMP OFF; waiting 2 seconds for release", flush=True)
        time.sleep(2.0)
        move_and_confirm(port, 82.0)
        final = move_and_confirm(port, 105.0)
        port.write(b"M2120 V0\r\n")
        print("PLACE COMPLETE; pump OFF; final={}".format(final), flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("ERROR:", exc, file=sys.stderr)
        sys.exit(1)
