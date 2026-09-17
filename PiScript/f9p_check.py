"""Standalone check for the SparkFun GPS-RTK-SMA (u-blox ZED-F9P) over I2C.

Verifies the receiver in isolation - no NTRIP, relay, or Unity involved:
  1. it answers on I2C
  2. it accepts gps.py's configuration
  3. it produces GGA at the requested rate
  4. its fixes are fresh (GNSS time-of-fix vs this Pi's clock) - the check
     that would have caught the 44-second backlog bug immediately

No RTK corrections are sent, so expect "GPS fix" (quality 1) at best, not
RTK. Indoors you may get no position at all; the rate check still works.

Run: python3 f9p_check.py [--rate 50] [--seconds 15]
"""

import argparse
import datetime
import statistics
import sys
import time

from pynmeagps import NMEAReader

from ublox_i2c import DEFAULT_ADDRESS, UbloxI2C, configure_ublox, parse_constellations

FIX_QUALITY = {0: "no fix", 1: "GPS fix", 2: "DGPS", 4: "RTK fixed", 5: "RTK float", 6: "estimated"}


def gnss_age_seconds(time_field):
    """How old a fix is, from GGA's hhmmss.ss UTC time-of-fix vs the Pi's
    UTC clock. None if the receiver doesn't have time yet."""
    try:
        hours = int(time_field[0:2])
        minutes = int(time_field[2:4])
        seconds = float(time_field[4:])
    except (ValueError, IndexError):
        return None
    now = datetime.datetime.now(datetime.timezone.utc)
    fix_time = now.replace(hour=hours, minute=minutes, second=int(seconds),
                           microsecond=int((seconds % 1) * 1_000_000))
    age = (now - fix_time).total_seconds()
    # Around UTC midnight the fix and the clock can land on different days.
    if age < -43200:
        age += 86400
    elif age > 43200:
        age -= 86400
    return age


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--rate", type=int, default=50, help="Measurement period in ms (default: 50 = 20Hz, the RTK maximum)")
    parser.add_argument("--seconds", type=int, default=15, help="How long to measure (default: 15)")
    parser.add_argument("--constellations", default="gps",
                        help="Comma-separated: gps,gal,glo,bds. Default 'gps' - the only choice that "
                             "reaches 20Hz with RTK. 'gps,gal' gives more satellites at 15Hz RTK.")
    args = parser.parse_args()
    try:
        constellations = parse_constellations(args.constellations)
    except ValueError as e:
        parser.error(str(e))

    stream = UbloxI2C()

    print("Configuring receiver at I2C 0x%02X..." % DEFAULT_ADDRESS)
    try:
        configured = configure_ublox(stream, args.rate, constellations)
    except OSError as e:
        print("\nNo receiver responding at 0x%02X (%s)." % (DEFAULT_ADDRESS, e))
        print("Check the Qwiic cable, then run `sudo i2cdetect -y 1` and look for 42.")
        sys.exit(1)
    if not configured:
        print("Not every setting was accepted - results below may not reflect gps.py's setup.")

    reader = NMEAReader(stream)

    # Changing constellations restarts the receiver's GNSS engine; counting
    # through that gap would understate the rate. Drain output meanwhile so
    # nothing stale is left buffered when measuring starts.
    print("\nLetting the receiver settle after the constellation change...")
    settle_until = time.monotonic() + 5
    while time.monotonic() < settle_until:
        reader.read()

    print("\nReading GGA for %d seconds...\n" % args.seconds)
    count = 0
    ages = []
    last_quality = None
    last_sats = None
    start = time.monotonic()

    while time.monotonic() - start < args.seconds:
        raw, _ = reader.read()
        if raw is None or b"GGA" not in raw:
            continue
        count += 1
        fields = raw.decode("ascii", errors="replace").split(",")
        if len(fields) < 8:
            continue

        age = gnss_age_seconds(fields[1])
        if age is not None:
            ages.append(age)
        last_quality = int(fields[6]) if fields[6].isdigit() else 0
        last_sats = int(fields[7]) if fields[7].isdigit() else 0

        age_text = "%.2fs" % age if age is not None else "n/a"
        print("GGA #%-4d quality=%-9s sats=%-3d age=%-7s lat=%s lon=%s" % (
            count, FIX_QUALITY.get(last_quality, last_quality), last_sats, age_text,
            fields[2] or "-", fields[4] or "-"))

    elapsed = time.monotonic() - start
    stream.close()

    target_hz = 1000.0 / args.rate
    actual_hz = count / elapsed
    print("\n=== RESULT ===")
    print("GGA received:  %d in %.1fs" % (count, elapsed))
    print("Rate:          %.1f Hz (target %.1f Hz) - %s" % (
        actual_hz, target_hz, "OK" if abs(actual_hz - target_hz) <= 0.1 * target_hz else "OFF TARGET"))

    if ages:
        median_age = statistics.median(ages)
        print("Fix age:       median %.2fs, max %.2fs - %s" % (
            median_age, max(ages),
            "fresh" if median_age < 0.5 else "STALE - data is backing up somewhere"))
        print("               (only meaningful if this Pi's clock is NTP-synced)")
    else:
        print("Fix age:       no time-of-fix yet - receiver still acquiring")

    print("Last fix:      %s, %s satellites" % (FIX_QUALITY.get(last_quality, last_quality), last_sats))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
