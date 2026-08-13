#!/bin/bash
# Wrapper so the full command lives in one place, testable by hand
# (sudo ./run_gps.sh) and reused as-is by the systemd service below.
set -e
cd "$(dirname "$0")"

exec python3 -u gps3.py \
  -u mojoaerial0101 \
  -p uavdrone \
  -f gpsdata.txt \
  --fixrate 100 \
  -r 20 \
  --relayhost 65.184.36.188 \
  --relayport 5002 \
  207.4.96.201 \
  2101 \
  VRS_RTCM34_MSM4
