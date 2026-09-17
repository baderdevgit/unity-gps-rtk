"""Standalone IMU relay - no GPS, no NTRIP, no corrections on the movement data.

Built for testing IMU-driven movement in isolation. Sends the same
{"type":"imu",...} UDP messages gps.py does, so Unity's ImuTestCube picks it
up with no changes - but the acceleration is passed through RAW, with none of
gps.py's mount-correction / heading-offset / frame-rotation math applied.

Heading DOES get the known-good corrections by default (so rotation keeps
working the way you already like it) while acceleration stays raw - that's
the combination for testing movement without fighting rotation at the same
time. Use --raw-heading if you want genuinely everything uncorrected.

Also prints every sample to the console with magnitudes and a running max, so
you can see directly whether walking actually produces acceleration or whether
the numbers are just too small to integrate into anything.

Run:
    python3 imu_relay.py --relayhost <server-ip>
    python3 imu_relay.py --relayhost <server-ip> --accel-mode body
    python3 imu_relay.py            (console only, no relay)
"""

import argparse
import json
import math
import socket
import sys
import time

import board
import busio
import digitalio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR, BNO_REPORT_LINEAR_ACCELERATION

# Same calibration constants gps.py uses - applied to HEADING ONLY, and only
# when --raw-heading isn't passed.
_MOUNT_REFERENCE_QUATERNION = (0.722, 0.030, 0.038, 0.690)
MOUNT_CORRECTION_QUATERNION = (
    -_MOUNT_REFERENCE_QUATERNION[0],
    -_MOUNT_REFERENCE_QUATERNION[1],
    -_MOUNT_REFERENCE_QUATERNION[2],
    _MOUNT_REFERENCE_QUATERNION[3],
)
HEADING_OFFSET_DEG = 46.3


def quaternion_to_heading(i, j, k, real):
    yaw_rad = math.atan2(2.0 * (real * k + i * j), 1.0 - 2.0 * (j * j + k * k))
    return (90.0 - math.degrees(yaw_rad)) % 360.0


def quaternion_multiply(q1, q2):
    i1, j1, k1, w1 = q1
    i2, j2, k2, w2 = q2
    return (
        w1 * i2 + i1 * w2 + j1 * k2 - k1 * j2,
        w1 * j2 - i1 * k2 + j1 * w2 + k1 * i2,
        w1 * k2 + i1 * j2 - j1 * i2 + k1 * w2,
        w1 * w2 - i1 * i2 - j1 * j2 - k1 * k2,
    )


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def rotate_body_accel_to_world(accel_xyz, i, j, k, real):
    qv = (i, j, k)
    t = tuple(2.0 * c for c in _cross(qv, accel_xyz))
    cross_qv_t = _cross(qv, t)
    return (
        accel_xyz[0] + real * t[0] + cross_qv_t[0],
        accel_xyz[1] + real * t[1] + cross_qv_t[1],
        accel_xyz[2] + real * t[2] + cross_qv_t[2],
    )


_current_i2c = None
_reset_pin = None


def connect():
    global _current_i2c, _reset_pin
    if _current_i2c is not None:
        try:
            _current_i2c.deinit()
        except Exception:
            pass

    if _reset_pin is None:
        _reset_pin = digitalio.DigitalInOut(board.D17)
        _reset_pin.direction = digitalio.Direction.OUTPUT
        _reset_pin.value = True  # active-LOW, held high = released

    _current_i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    imu = BNO08X_I2C(_current_i2c, address=0x4A)
    imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    imu.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
    return imu


def connect_with_retry(attempts=3):
    for attempt in range(1, attempts + 1):
        try:
            return connect()
        except Exception as e:
            print("Connect attempt %d/%d failed (%s)" % (attempt, attempts, e))
            if attempt < attempts:
                time.sleep(1)
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--relayhost", default=None,
                        help="Server IP to send UDP messages to. Omit for console-only.")
    parser.add_argument("--relayport", type=int, default=5002,
                        help="Server's Pi-facing UDP port (default: 5002, same as gps.py)")
    parser.add_argument("--rate", type=float, default=50.0,
                        help="Samples per second (default: 50 - higher than gps.py's 10, since "
                             "integrating acceleration benefits from a finer timestep)")
    parser.add_argument("--accel-mode", choices=["world", "body"], default="world",
                        help="'world' rotates acceleration into the sensor's world frame using the raw "
                             "quaternion (no mount/offset correction). 'body' passes the sensor's own "
                             "body-frame axes straight through, completely unrotated. (default: world)")
    parser.add_argument("--raw-heading", action="store_true",
                        help="Send heading with NO mount correction or offset either. Rotation will look "
                             "wrong in Unity, but nothing is corrected anywhere.")
    parser.add_argument("--quiet", action="store_true", help="Don't print every sample")
    args = parser.parse_args()

    imu = connect_with_retry()
    if imu is None:
        print("Could not connect to IMU - giving up.")
        sys.exit(1)
    print("IMU connected.")

    sock = None
    if args.relayhost:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect((args.relayhost, args.relayport))
        print("Relaying to %s:%d over UDP" % (args.relayhost, args.relayport))
    else:
        print("No --relayhost given: console output only.")

    print("accel-mode=%s  heading=%s  rate=%.0fHz" % (
        args.accel_mode, "raw" if args.raw_heading else "corrected", args.rate))
    print("Ctrl+C to stop.\n")

    interval = 1.0 / args.rate
    max_accel = 0.0
    sample = 0
    start = time.time()

    while True:
        time.sleep(interval)
        now = time.time()

        try:
            i, j, k, real = imu.quaternion
            accel_body = imu.linear_acceleration
        except Exception as e:
            print("IMU read failed (%s) - reconnecting" % e)
            imu = connect_with_retry() or imu
            continue

        magnitude = math.sqrt(i * i + j * j + k * k + real * real)
        if abs(magnitude - 1.0) > 0.05:
            continue  # corrupt quaternion, skip

        heading_raw = quaternion_to_heading(i, j, k, real)
        if args.raw_heading:
            heading = heading_raw
        else:
            corrected = quaternion_multiply((i, j, k, real), MOUNT_CORRECTION_QUATERNION)
            heading = (quaternion_to_heading(*corrected) + HEADING_OFFSET_DEG) % 360.0

        # No frame correction applied to acceleration in either mode - that's
        # the whole point of this script.
        if args.accel_mode == "world":
            accel_east, accel_north, accel_up = rotate_body_accel_to_world(accel_body, i, j, k, real)
        else:
            accel_east, accel_north, accel_up = accel_body[0], accel_body[1], accel_body[2]

        horizontal = math.sqrt(accel_east * accel_east + accel_north * accel_north)
        max_accel = max(max_accel, horizontal)
        sample += 1

        if sock is not None:
            line = (json.dumps({
                "type": "imu",
                "heading": heading,
                "accelEast": accel_east,
                "accelNorth": accel_north,
                "timestamp": now,
            }) + "\n").encode("utf-8")
            try:
                sock.send(line)
            except OSError as e:
                print("Send failed (%s)" % e)

        if not args.quiet:
            print("[+%6.1fs] #%-6d heading=%6.1f  accel(E=%7.3f N=%7.3f U=%7.3f)  |horiz|=%6.3f  max=%6.3f"
                  % (now - start, sample, heading, accel_east, accel_north, accel_up, horizontal, max_accel))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
