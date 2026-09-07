"""Standalone BNO08x test script - no GPS/NTRIP/relay involved, just the
sensor. Prints raw quaternion, magnitude-validated, plus the same
quaternion_to_heading()/rotate_body_accel_to_world() math gps.py uses, so you
can eyeball whether wiring/mode-select pins are correct before touching the
full pipeline.

The printed heading has the mounting-correction quaternion (calibrated from
a captured reference pose, matching gps.py) applied, so it should read
actual vehicle-relative heading rather than the sensor's raw/tilted one.

Run: python3 test_imu.py
"""

import sys
import time
import math

import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR, BNO_REPORT_LINEAR_ACCELERATION


def quaternion_to_heading(i, j, k, real):
    yaw_rad = math.atan2(2.0 * (real * k + i * j), 1.0 - 2.0 * (j * j + k * k))
    yaw_deg = math.degrees(yaw_rad)
    return (90.0 - yaw_deg) % 360.0


# Must match gps.py's calibrated mounting correction exactly.
_MOUNT_REFERENCE_QUATERNION = (0.722, 0.030, 0.038, 0.690)  # (i, j, k, real) captured at reference pose
MOUNT_CORRECTION_QUATERNION = (
    -_MOUNT_REFERENCE_QUATERNION[0],
    -_MOUNT_REFERENCE_QUATERNION[1],
    -_MOUNT_REFERENCE_QUATERNION[2],
    _MOUNT_REFERENCE_QUATERNION[3],
)


def quaternion_multiply(q1, q2):
    i1, j1, k1, w1 = q1
    i2, j2, k2, w2 = q2
    w = w1 * w2 - i1 * i2 - j1 * j2 - k1 * k2
    i = w1 * i2 + i1 * w2 + j1 * k2 - k1 * j2
    j = w1 * j2 - i1 * k2 + j1 * w2 + k1 * i2
    k = w1 * k2 + i1 * j2 - j1 * i2 + k1 * w2
    return (i, j, k, w)


# Must match gps.py's calibrated heading offset exactly (see gps.py for why
# this constant exists - quaternion_to_heading's own +90 baseline).
HEADING_OFFSET_DEG = -73.7


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


def connect():
    # Release the previous bus before acquiring a new one - skipping this
    # leaves the I2C peripheral locked, so every reconnect after the first
    # silently fails to re-acquire it.
    global _current_i2c
    if _current_i2c is not None:
        try:
            _current_i2c.deinit()
        except Exception:
            pass

    _current_i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    # ADO reads high on this particular board despite being wired to GND, so
    # it answers on the secondary address (0x4B) instead of the default 0x4A.
    imu = BNO08X_I2C(_current_i2c, address=0x4B)
    imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    imu.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
    return imu


def main():
    imu = connect()
    print("IMU connected. Reading... (Ctrl+C to stop)")

    consecutive_failures = 0

    while True:
        try:
            i, j, k, real = imu.quaternion
            accel = imu.linear_acceleration
        except Exception as e:
            consecutive_failures += 1
            print("Read failed (%s) - failure #%d" % (e, consecutive_failures))
            if consecutive_failures >= 3:
                print("Reconnecting...")
                try:
                    imu = connect()
                    consecutive_failures = 0
                    print("Reconnected.")
                except Exception as reconnect_error:
                    print("Reconnect failed (%s), retrying in 1s" % reconnect_error)
                    time.sleep(1)
            time.sleep(0.1)
            continue

        magnitude = math.sqrt(i * i + j * j + k * k + real * real)
        if abs(magnitude - 1.0) > 0.05:
            consecutive_failures += 1
            print("Corrupt quaternion (|q|=%.3f, expected ~1.0) - failure #%d" % (magnitude, consecutive_failures))
            if consecutive_failures >= 3:
                print("Reconnecting...")
                try:
                    imu = connect()
                    consecutive_failures = 0
                    print("Reconnected.")
                except Exception as reconnect_error:
                    print("Reconnect failed (%s), retrying in 1s" % reconnect_error)
                    time.sleep(1)
            time.sleep(0.1)
            continue

        consecutive_failures = 0
        corrected = quaternion_multiply((i, j, k, real), MOUNT_CORRECTION_QUATERNION)
        heading = quaternion_to_heading(*corrected)
        heading = (heading + HEADING_OFFSET_DEG) % 360.0
        east, north, up = rotate_body_accel_to_world(accel, i, j, k, real)

        print(
            "heading=%6.1f  quat(i=%6.3f j=%6.3f k=%6.3f real=%6.3f |q|=%.3f)  "
            "accel_world(E=%5.2f N=%5.2f U=%5.2f)"
            % (heading, i, j, k, real, magnitude, east, north, up)
        )

        time.sleep(0.1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
