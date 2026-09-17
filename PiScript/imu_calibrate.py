"""BNO08x health check and calibration.

Answers "is my sensor actually bad, or is this just what IMUs do?" with a
number instead of a vibe.

The measurement: sit the sensor perfectly still and look at the horizontal
components of linear_acceleration after rotating into world frame. The sensor
has already subtracted gravity on-chip, so a still sensor should read ~0. It
doesn't, and whatever is left over is mostly gravity leaking in through an
imperfect orientation estimate. Since gravity is 9.81 m/s^2, that residual
converts straight into an equivalent tilt error:

    tilt_error = asin(residual / 9.81)

A healthy, calibrated BNO085 should hold well under 1 degree. We measured
~0.86 m/s^2 on this rig earlier, which is ~5 degrees - bad enough to be worth
fixing, and it's the single biggest thing wrecking any attempt at integrating
acceleration into movement.

Run:
    python3 imu_calibrate.py              measure only
    python3 imu_calibrate.py --calibrate  measure, calibrate, measure again
"""

import argparse
import math
import sys
import time

import board
import busio
import digitalio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR, BNO_REPORT_LINEAR_ACCELERATION

try:
    from adafruit_bno08x import BNO_REPORT_MAGNETOMETER
    HAVE_MAG = True
except ImportError:
    HAVE_MAG = False

GRAVITY = 9.81


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
        _reset_pin.value = True

    _current_i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    imu = BNO08X_I2C(_current_i2c, address=0x4A)
    imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    imu.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
    if HAVE_MAG:
        try:
            imu.enable_feature(BNO_REPORT_MAGNETOMETER)
        except Exception as e:
            print("Could not enable magnetometer report (%s) - calibration status may read 0" % e)
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


def measure(imu, seconds, label):
    """Samples the resting residual and reports it as an equivalent tilt error."""
    print("\n=== %s ===" % label)
    print("Hold the sensor COMPLETELY STILL for %d seconds..." % seconds)
    for countdown in (3, 2, 1):
        print("  starting in %d..." % countdown)
        time.sleep(1)
    print("  measuring...")

    horiz_samples = []
    up_samples = []
    deadline = time.time() + seconds

    while time.time() < deadline:
        try:
            i, j, k, real = imu.quaternion
            accel_body = imu.linear_acceleration
        except Exception:
            continue

        magnitude = math.sqrt(i * i + j * j + k * k + real * real)
        if abs(magnitude - 1.0) > 0.05:
            continue

        east, north, up = rotate_body_accel_to_world(accel_body, i, j, k, real)
        horiz_samples.append(math.sqrt(east * east + north * north))
        up_samples.append(abs(up))
        time.sleep(0.02)

    if not horiz_samples:
        print("  No valid samples collected.")
        return None

    mean_h = sum(horiz_samples) / len(horiz_samples)
    max_h = max(horiz_samples)
    var = sum((x - mean_h) ** 2 for x in horiz_samples) / len(horiz_samples)
    std_h = math.sqrt(var)
    mean_up = sum(up_samples) / len(up_samples)

    tilt_deg = math.degrees(math.asin(min(mean_h / GRAVITY, 1.0)))

    print("  samples:            %d" % len(horiz_samples))
    print("  horizontal residual: mean %.3f  max %.3f  std %.3f  m/s^2" % (mean_h, max_h, std_h))
    print("  vertical residual:   mean %.3f m/s^2" % mean_up)
    print("  equivalent tilt error: %.2f degrees" % tilt_deg)

    if mean_h < 0.05:
        verdict = "EXCELLENT - sensor is healthy and well calibrated."
    elif mean_h < 0.15:
        verdict = "GOOD - normal for a well-behaved BNO085."
    elif mean_h < 0.40:
        verdict = "MEDIOCRE - calibration would likely help."
    else:
        verdict = "POOR - needs calibration, or something is wrong with the mount/board."
    print("  verdict: %s" % verdict)

    # Position error this residual alone would produce if double-integrated,
    # which is the number that actually matters for movement.
    for t in (1, 5, 10):
        print("    -> integrated position error after %2ds: %.2f m" % (t, 0.5 * mean_h * t * t))

    return mean_h


def run_calibration(imu):
    print("\n=== CALIBRATION ===")
    imu.begin_calibration()
    print("Calibration started. Work through these, slowly:")
    print("  1. ACCELEROMETER - rest the board on each of its 6 faces,")
    print("     a few seconds per face (flat, upside down, each edge).")
    print("  2. GYRO - set it down and leave it completely still ~5s.")
    print("  3. MAGNETOMETER - pick it up and draw figure-8s in the air,")
    print("     rotating your wrist as you go, for ~20s.")
    print("\nWatching calibration status (want 'High Accuracy'). Ctrl+C when done.\n")

    labels = ["Accuracy Unreliable", "Low Accuracy", "Medium Accuracy", "High Accuracy"]
    try:
        while True:
            try:
                status = imu.calibration_status
                name = labels[status] if 0 <= status < len(labels) else "unknown(%s)" % status
                print("  calibration status: %d (%s)" % (status, name))
            except Exception as e:
                print("  status read failed (%s)" % e)
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nSaving calibration data to the sensor...")
        try:
            imu.save_calibration_data()
            print("Saved.")
        except Exception as e:
            print("Save failed (%s)" % e)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--calibrate", action="store_true",
                        help="Run the calibration routine between a before and after measurement")
    parser.add_argument("--seconds", type=int, default=10, help="Measurement duration (default: 10)")
    args = parser.parse_args()

    imu = connect_with_retry()
    if imu is None:
        print("Could not connect to IMU.")
        sys.exit(1)
    print("IMU connected.")

    before = measure(imu, args.seconds, "BASELINE MEASUREMENT")

    if args.calibrate:
        run_calibration(imu)
        after = measure(imu, args.seconds, "MEASUREMENT AFTER CALIBRATION")
        if before is not None and after is not None:
            print("\n=== RESULT ===")
            print("  before: %.3f m/s^2   after: %.3f m/s^2" % (before, after))
            if after < before * 0.7:
                print("  Clear improvement - calibration helped.")
            elif after > before * 1.3:
                print("  Got worse - the calibration motions may not have been completed properly.")
            else:
                print("  Roughly unchanged. If it's still high, suspect the board or the mount")
                print("  rather than calibration.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
