#!/usr/bin/env python3
"""Interactive SwiftPro jog using real serial position reports (new robot).

Stop swiftpro_write_node before running: both programs use /dev/ttyACM0.
The pump is kept off. No movement occurs on startup or exit.
"""

import argparse
import csv
import math
import re
import subprocess
import sys
import time

import serial


POSITION = re.compile(r"@3 X(-?\d+(?:\.\d+)?) Y(-?\d+(?:\.\d+)?) Z(-?\d+(?:\.\d+)?)")


def read_reports(port, seconds):
    end = time.monotonic() + seconds
    positions, errors = [], []
    while time.monotonic() < end:
        line = port.readline().decode("ascii", errors="replace").strip()
        match = POSITION.search(line)
        if match:
            positions.append(tuple(map(float, match.groups())))
        if " E" in line or line.startswith("E"):
            errors.append(line)
    return positions, errors


def actual(port):
    reports, errors = read_reports(port, 2.0)
    if errors:
        raise RuntimeError("Controller error: " + "; ".join(errors))
    if not reports:
        raise RuntimeError("No actual position report; movement stopped")
    return reports[-1]


def show(pos):
    print("ACTUAL: x={:.2f} y={:.2f} z={:.2f} mm".format(*pos), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--min-z", type=float, default=35.0)
    parser.add_argument("--log", default="/home/spark/c_calibration/new_robot_jog.csv")
    args = parser.parse_args()
    if not 35 <= args.min_z <= 100:
        parser.error("--min-z must be 35..100 mm")
    owners = subprocess.run(["fuser", args.port], capture_output=True, text=True)
    if owners.returncode == 0:
        parser.error("Serial port already in use by PID(s): " + owners.stdout.strip())
    if owners.returncode != 1:
        parser.error("Could not check serial port: " + owners.stderr.strip())

    with serial.Serial(args.port, 115200, timeout=0.2, exclusive=True) as port:
        time.sleep(2.0)
        port.reset_input_buffer()
        port.write(b"M2120 V0.2\r\n")
        try:
            reports, errors = read_reports(port, 2.0)
            if errors or not reports:
                raise RuntimeError("No actual position report: " + repr(errors))
            position = reports[-1]
            port.write(b"M2231 V0\r\n")  # pump off
            port.write(b"M17\r\n")       # attach motors
            print("Pump OFF. x+ is forward. Commands: x+/x-/y+/y-/z+/z-, "
                  "step 1|2|5|10, where, watch, quit.")
            print("Bounds: x 90..330, y -120..120, z {:.0f}..160 mm".format(args.min_z))
            show(position)
            step = 2.0
            with open(args.log, "a", newline="", encoding="utf-8") as logfile:
                writer = csv.writer(logfile)
                if logfile.tell() == 0:
                    writer.writerow(["time", "target_x", "target_y", "target_z",
                                     "actual_x", "actual_y", "actual_z", "error_mm"])
                while True:
                    try:
                        command = input("jog [{:g} mm]> ".format(step)).strip().lower()
                    except (EOFError, KeyboardInterrupt):
                        print()
                        break
                    if command in ("quit", "q", "exit"):
                        break
                    if command == "where":
                        position = actual(port)
                        show(position)
                        continue
                    if command == "watch":
                        for _ in range(10):
                            position = actual(port)
                            show(position)
                        continue
                    if command.startswith("step "):
                        try:
                            requested = float(command.split()[1])
                        except (ValueError, IndexError):
                            requested = None
                        if requested in (1.0, 2.0, 5.0, 10.0):
                            step = requested
                        else:
                            print("Use: step 1, step 2, step 5, or step 10")
                        continue
                    if command not in ("x+", "x-", "y+", "y-", "z+", "z-"):
                        print("Unknown command")
                        continue
                    before = actual(port)
                    axis = "xyz".index(command[0])
                    target = list(before)
                    target[axis] += step if command[1] == "+" else -step
                    limits = ((90, 330), (-120, 120), (args.min_z, 160))
                    if any(not low <= value <= high
                           for value, (low, high) in zip(target, limits)):
                        print("REJECTED: outside calibration bounds")
                        continue
                    port.write(b"M2231 V0\r\n")
                    port.reset_input_buffer()
                    code = "G0 X{:.2f} Y{:.2f} Z{:.2f} F1500\r\n".format(*target)
                    port.write(code.encode("ascii"))
                    start = time.monotonic()
                    moved = False
                    while time.monotonic() - start < 12:
                        reports, errors = read_reports(port, 0.3)
                        if errors:
                            raise RuntimeError("Controller rejected movement: " + "; ".join(errors))
                        if reports:
                            position = reports[-1]
                            moved |= abs(position[axis] - before[axis]) >= step * 0.5
                            if moved and time.monotonic() - start >= 0.6:
                                break
                    else:
                        raise RuntimeError("No confirmed movement; stopped")
                    position = actual(port)
                    error = math.dist(position, target)
                    writer.writerow([time.strftime("%Y-%m-%dT%H:%M:%S"),
                                     *target, *position, round(error, 2)])
                    logfile.flush()
                    show(position)
                    if not moved or error > 4:
                        raise RuntimeError("Target not confirmed; stopped")
        finally:
            port.write(b"M2231 V0\r\n")
            port.write(b"M2120 V0\r\n")
            print("Pump OFF. No automatic motion on exit.")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("ERROR:", exc, file=sys.stderr)
        sys.exit(1)
