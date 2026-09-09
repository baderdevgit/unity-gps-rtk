"""Terminal 3D visualizer for the BNO08x - draws a wireframe cube that
rotates live with the sensor's fused orientation, using only the standard
library (curses) so it works fine over SSH.

The cube's local axes match the project's world-frame convention used
elsewhere (X=East/right, Y=North/forward, Z=Up) - so when the sensor is
level and pointing north, the cube should look "normal"; tilt/rotate the
sensor and the cube tilts/rotates the same way on screen.

Run: python3 visualize_imu.py   (press q to quit)
"""

import curses
import math
import os
import sys
import time

import board
import busio
import digitalio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR, BNO_REPORT_LINEAR_ACCELERATION


def quaternion_to_heading(i, j, k, real):
    yaw_rad = math.atan2(2.0 * (real * k + i * j), 1.0 - 2.0 * (j * j + k * k))
    yaw_deg = math.degrees(yaw_rad)
    return (90.0 - yaw_deg) % 360.0


# Must match gps.py's calibrated mounting correction and heading offset
# exactly (see gps.py for the calibration procedure/explanation).
_MOUNT_REFERENCE_QUATERNION = (0.722, 0.030, 0.038, 0.690)  # (i, j, k, real) captured at reference pose
MOUNT_CORRECTION_QUATERNION = (
    -_MOUNT_REFERENCE_QUATERNION[0],
    -_MOUNT_REFERENCE_QUATERNION[1],
    -_MOUNT_REFERENCE_QUATERNION[2],
    _MOUNT_REFERENCE_QUATERNION[3],
)
HEADING_OFFSET_DEG = 46.3  # was 16.3, +30 more after the arrow was still off to the left


def quaternion_multiply(q1, q2):
    i1, j1, k1, w1 = q1
    i2, j2, k2, w2 = q2
    w = w1 * w2 - i1 * i2 - j1 * j2 - k1 * k2
    i = w1 * i2 + i1 * w2 + j1 * k2 - k1 * j2
    j = w1 * j2 - i1 * k2 + j1 * w2 + k1 * i2
    k = w1 * k2 + i1 * j2 - j1 * i2 + k1 * w2
    return (i, j, k, w)


def rotate_vector(v, i, j, k, real):
    """Rotates body-frame vector v=(x,y,z) into world frame by quaternion
    (i,j,k,real) - standard optimized quaternion-vector rotation formula."""
    qv = (i, j, k)
    t = (
        2.0 * (qv[1] * v[2] - qv[2] * v[1]),
        2.0 * (qv[2] * v[0] - qv[0] * v[2]),
        2.0 * (qv[0] * v[1] - qv[1] * v[0]),
    )
    cross_qv_t = (
        qv[1] * t[2] - qv[2] * t[1],
        qv[2] * t[0] - qv[0] * t[2],
        qv[0] * t[1] - qv[1] * t[0],
    )
    return (
        v[0] + real * t[0] + cross_qv_t[0],
        v[1] + real * t[1] + cross_qv_t[1],
        v[2] + real * t[2] + cross_qv_t[2],
    )


# Cube corners in body frame: X=right, Y=forward, Z=up.
_CUBE_VERTS = [
    (x, y, z)
    for x in (-1, 1)
    for y in (-1, 1)
    for z in (-1, 1)
]

_CUBE_EDGES = [
    (a, b)
    for a in range(8)
    for b in range(a + 1, 8)
    # Connect vertices that differ in exactly one axis (i.e. share an edge).
    if sum(1 for c in range(3) if _CUBE_VERTS[a][c] != _CUBE_VERTS[b][c]) == 1
]


_current_i2c = None
_reset_pin = None


def connect():
    # Release the previous bus before acquiring a new one - skipping this
    # leaves the I2C peripheral locked, so every reconnect after the first
    # silently fails to re-acquire it.
    global _current_i2c, _reset_pin
    if _current_i2c is not None:
        try:
            _current_i2c.deinit()
        except Exception:
            pass

    # RST is wired to GPIO17 (physical pin 11) instead of straight to 3.3V.
    # NOT passed to BNO08X_I2C below - the library's own hard_reset() pulse
    # consistently broke the connection in testing, so back off the pulse
    # for now and just hold it released ourselves.
    if _reset_pin is None:
        _reset_pin = digitalio.DigitalInOut(board.D17)
        _reset_pin.direction = digitalio.Direction.OUTPUT
        _reset_pin.value = True  # active-LOW - held high = released

    _current_i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    imu = BNO08X_I2C(_current_i2c, address=0x4A)
    imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    imu.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
    return imu


def draw_line(buf, w, h, x0, y0, x1, y1, ch):
    x0, y0, x1, y1 = int(round(x0)), int(round(y0)), int(round(x1)), int(round(y1))
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        if 0 <= x0 < w and 0 <= y0 < h:
            buf[y0][x0] = ch
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


def main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)

    # adafruit_bno08x prints raw "DBG:: ..." packet dumps straight to
    # stdout on certain packets regardless of the debug flag, which
    # corrupts curses' screen buffer. Curses draws via direct terminal
    # calls (not Python's sys.stdout), so it's safe to swallow those here.
    sys.stdout = open(os.devnull, "w")
    sys.stderr = open(os.devnull, "w")

    # A fresh RST pulse means the chip may need a moment to finish booting
    # before it's ready to talk - retry a few times rather than crashing on
    # a single flaky first attempt.
    imu = None
    for _attempt in range(3):
        try:
            imu = connect()
            break
        except Exception:
            time.sleep(1)
    if imu is None:
        imu = connect()  # let the final attempt's exception propagate normally

    consecutive_failures = 0

    while True:
        key = stdscr.getch()
        if key in (ord("q"), ord("Q")):
            break

        try:
            i, j, k, real = imu.quaternion
        except Exception:
            consecutive_failures += 1
            if consecutive_failures >= 3:
                try:
                    imu = connect()
                    consecutive_failures = 0
                except Exception:
                    time.sleep(1)
            time.sleep(0.05)
            continue

        magnitude = math.sqrt(i * i + j * j + k * k + real * real)
        if abs(magnitude - 1.0) > 0.05:
            consecutive_failures += 1
            if consecutive_failures >= 3:
                try:
                    imu = connect()
                    consecutive_failures = 0
                except Exception:
                    time.sleep(1)
            time.sleep(0.05)
            continue
        consecutive_failures = 0

        corrected = quaternion_multiply((i, j, k, real), MOUNT_CORRECTION_QUATERNION)
        heading = (quaternion_to_heading(*corrected) + HEADING_OFFSET_DEG) % 360.0

        h, w = stdscr.getmaxyx()
        h = max(h, 10)
        w = max(w, 20)
        buf = [[" "] * w for _ in range(h)]

        cx, cy = w // 2, (h - 2) // 2  # leave room for header rows
        scale = min(w, (h - 2) * 2) / 5.0

        ci, cj, ck, creal = corrected
        projected = []
        for vx, vy, vz in _CUBE_VERTS:
            rx, ry, rz = rotate_vector((vx, vy, vz), ci, cj, ck, creal)
            # Screen X from world right (East), screen Y from world up (Z),
            # widened horizontally to compensate for tall terminal glyphs.
            sx = cx + rx * scale * 2.0
            sy = cy - rz * scale
            projected.append((sx, sy))

        for a, b in _CUBE_EDGES:
            x0, y0 = projected[a]
            x1, y1 = projected[b]
            draw_line(buf, w, h, x0, y0 + 2, x1, y1 + 2, "#")

        stdscr.erase()
        stdscr.addstr(0, 0, f"heading={heading:6.1f}  |q|={magnitude:.3f}  (q to quit)"[: w - 1])
        for row in range(h - 2):
            stdscr.addstr(row + 2, 0, "".join(buf[row + 2])[: w - 1])
        stdscr.refresh()

        time.sleep(1.0 / 15.0)


if __name__ == "__main__":
    curses.wrapper(main)
