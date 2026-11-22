#!/usr/bin/env python3
"""Read real SwiftPro position and optionally send one bounded Cartesian move.

Run only while the ROS SwiftPro serial node is stopped. The default is read only.
The controller's @3 reports are physical feedback; /SwiftproState_topic from
swiftpro_write_node is only a last-command echo on this robot.
"""

import argparse
import re
import subprocess
import sys
import time

import serial


POSITION = re.compile(r"@3 X(-?\d+(?:\.\d+)?) Y(-?\d+(?:\.\d+)?) Z(-?\d+(?:\.\d+)?)")
BOUNDS = ((110.0, 300.0), (-100.0, 100.0), (-50.0, 140.0))


def receive(port, seconds):
    deadline = time.monotonic() + seconds
    reports = []
    errors = []
    buffer = ""
    while time.monotonic() < deadline:
        buffer += port.read(256).decode("ascii", errors="replace")
        while "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            line = line.strip()
            match = POSITION.search(line)
            if match:
                reports.append(tuple(float(value) for value in match.groups()))
            if " E" in line or line.startswith("E"):
                errors.append(line)
    return reports, errors


def check_port_unused(port):
    result = subprocess.run(["fuser", port], capture_output=True, text=True)
    if result.returncode == 0:
        raise RuntimeError("Serial port is in use by PID(s): " + result.stdout.strip())
    if result.returncode not in (1,):
        raise RuntimeError("Could not check serial port ownership: " + result.stderr.strip())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--move", nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument("--speed", type=int, default=2000)
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()

    if args.move:
        for name, value, (low, high) in zip("xyz", args.move, BOUNDS):
            if not low <= value <= high:
                parser.error("{}={} outside {}..{} mm".format(name, value, low, high))
        if not 500 <= args.speed <= 5000:
            parser.error("--speed must be 500..5000")

    check_port_unused(args.port)
    with serial.Serial(args.port, 115200, timeout=0.2, exclusive=True) as port:
        try:
            time.sleep(2.0)
            port.reset_input_buffer()
            port.write(b"M2120 V0.2\r\n")
            reports, errors = receive(port, 1.0)
            if not reports:
                raise RuntimeError("No real position report from controller: " + repr(errors))
            print("ACTUAL before: x={:.2f} y={:.2f} z={:.2f} mm".format(*reports[-1]))

            if args.move:
                port.write(b"M17\r\n")
                receive(port, 0.5)
                command = "G0 X{:.2f} Y{:.2f} Z{:.2f} F{}\r\n".format(
                    args.move[0], args.move[1], args.move[2], args.speed)
                print("COMMAND:", command.strip())
                port.write(command.encode("ascii"))
                start = time.monotonic()
                last = reports[-1]
                while time.monotonic() - start < args.timeout:
                    updates, errors = receive(port, 0.4)
                    if errors:
                        raise RuntimeError("Controller rejected command: " + "; ".join(errors))
                    if updates:
                        last = updates[-1]
                        print("ACTUAL: x={:.2f} y={:.2f} z={:.2f} mm".format(*last))
                        if max(abs(a - b) for a, b in zip(last, args.move)) <= 5.0:
                            print("Reached target within 5 mm feedback tolerance")
                            return
                raise RuntimeError("Timed out before reaching target; last actual: " + repr(last))
        finally:
            port.write(b"M2120 V0\r\n")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("ERROR:", exc, file=sys.stderr)
        sys.exit(1)
