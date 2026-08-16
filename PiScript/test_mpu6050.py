#!/usr/bin/env python3
"""Standalone sanity check for a GY-521 (MPU-6050) wired over I2C.
Prints raw accelerometer/gyro readings so wiring can be confirmed before
integrating with gps.py. Run this on its own first."""

import time
from smbus2 import SMBus

MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43


def read_word(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    if value >= 0x8000:
        value -= 0x10000
    return value


def read_gyro(bus):
    return (
        read_word(bus, MPU6050_ADDR, GYRO_XOUT_H) / 131.0,
        read_word(bus, MPU6050_ADDR, GYRO_XOUT_H + 2) / 131.0,
        read_word(bus, MPU6050_ADDR, GYRO_XOUT_H + 4) / 131.0,
    )


def calibrate_gyro(bus, samples=200):
    """Every MEMS gyro has a small per-axis manufacturing bias - averaging
    readings while stationary lets us cancel it out in software."""
    print(f"Calibrating gyro bias ({samples} samples, keep it still)...")
    sum_x = sum_y = sum_z = 0.0
    for _ in range(samples):
        x, y, z = read_gyro(bus)
        sum_x += x
        sum_y += y
        sum_z += z
        time.sleep(0.005)
    bias = (sum_x / samples, sum_y / samples, sum_z / samples)
    print(f"Gyro bias: x={bias[0]:+.2f} y={bias[1]:+.2f} z={bias[2]:+.2f}\n")
    return bias


def main():
    with SMBus(1) as bus:
        # MPU-6050 starts in sleep mode; this wakes it up.
        bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
        time.sleep(0.1)

        gyro_bias = calibrate_gyro(bus)

        print("Reading MPU-6050 (gyro bias-corrected). Should show ~1.0g on one "
              "accel axis (gravity) and ~0 on gyro axes while stationary. Ctrl+C to stop.\n")

        while True:
            accel_x = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H) / 16384.0
            accel_y = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 2) / 16384.0
            accel_z = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H + 4) / 16384.0
            raw_gyro_x, raw_gyro_y, raw_gyro_z = read_gyro(bus)
            gyro_x = raw_gyro_x - gyro_bias[0]
            gyro_y = raw_gyro_y - gyro_bias[1]
            gyro_z = raw_gyro_z - gyro_bias[2]

            print(f"Accel (g): x={accel_x:+.3f} y={accel_y:+.3f} z={accel_z:+.3f}   "
                  f"Gyro (deg/s): x={gyro_x:+.2f} y={gyro_y:+.2f} z={gyro_z:+.2f}")
            time.sleep(0.2)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
