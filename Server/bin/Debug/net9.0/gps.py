#!/usr/bin/env -S python3 -u
"""
This is heavily based on the NtripPerlClient program written by BKG.
Then heavily based on a unavco original.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

MODIFIED:
  - Attempts to configure the LC29H fix rate via $PQTMCFGFIXRATE at startup
    (checksum computed automatically - no more hand-typed checksums).
  - Reads back and prints the module's response so you know immediately
    whether the rate change was accepted or rejected (some LC29H(DA)
    firmware versions reject this with ERROR,1 - known Quectel forum issue).
  - Periodically re-sends GGA to the NTRIP caster on a timer (not just once
    at connection start) - VRS casters generally expect refreshed position
    updates to keep the virtual base station synced to you.
  - Relays each parsed GNGGA fix as a JSON line to the Domain-Expansion-Server
    C# relay (which forwards it on to Unity), over its own persistent,
    self-reconnecting TCP connection so a relay outage never blocks GPS
    processing.
  - Reads a GY-521 (MPU-6050) IMU over I2C to add a heading estimate,
    relayed separately at ~10Hz (independent of the ~1Hz GPS fix rate) so
    Unity can rotate smoothly. Re-anchored to the GPS-derived course
    whenever the rover has moved far enough for that to be reliable, so
    gyro drift never accumulates for more than one fix interval. Heading
    only - no IMU-based position estimation, which proved too unreliable.
"""

import math
import socket
import sys
import datetime
import base64
import time
import json
import threading
from optparse import OptionParser

import serial
from pynmeagps import NMEAReader
from smbus2 import SMBus

version = 0.3
useragent = "NTRIP JCMBsoftPythonClient/%.1f" % version

# reconnect parameter (fixed values):
factor = 3  # How much the sleep time increases with each failed attempt
maxReconnect = 1
maxReconnectTime = 1200
sleepTime = 3  # So the first one is 1 second


def nmea_checksum(payload):
    """Compute the XOR checksum for an NMEA/PQTM payload (everything
    between '$' and '*', not including either character)."""
    cksum = 0
    for c in payload:
        cksum ^= ord(c)
    return "%02X" % cksum


def build_pqtm_command(payload):
    """Build a full $PQTM... sentence with correct checksum + CRLF, as
    bytes ready for the serial port (matches what _sendAndCapture expects)."""
    return ("$%s*%s\r\n" % (payload, nmea_checksum(payload))).encode('ascii')


# --- GY-521 (MPU-6050) IMU, over I2C - heading only, no position estimate --

MPU6050_ADDR = 0x68
MPU6050_PWR_MGMT_1 = 0x6B
MPU6050_GYRO_ZOUT_H = 0x47


def _read_word(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    if value >= 0x8000:
        value -= 0x10000
    return value


def read_gyro_z(bus):
    """Yaw rate in degrees/second."""
    return _read_word(bus, MPU6050_ADDR, MPU6050_GYRO_ZOUT_H) / 131.0


def calibrate_gyro_z(bus, samples=200):
    """Every MEMS gyro has a small per-axis manufacturing bias - averaging
    readings while stationary lets us cancel it out in software."""
    total = 0.0
    for _ in range(samples):
        total += read_gyro_z(bus)
        time.sleep(0.005)
    return total / samples


def meters_distance(lat1, lon1, lat2, lon2):
    """Flat-earth approximation - matches GpsPositioner.cs on the Unity side,
    accurate enough for the short distances between consecutive fixes."""
    meters_per_deg_lat = 111320.0
    meters_per_deg_lon = 111320.0 * math.cos(math.radians(lat1))
    dy = (lat2 - lat1) * meters_per_deg_lat
    dx = (lon2 - lon1) * meters_per_deg_lon
    return math.hypot(dx, dy)


def compute_bearing(lat1, lon1, lat2, lon2):
    """Initial compass bearing (degrees, 0=North, 90=East) from point 1 to
    point 2."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    x = math.sin(dlambda) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


class ServerRelay:
    """Persistent TCP connection to the Domain-Expansion-Server relay
    (the C# server's Raspberry Pi-facing listener), which forwards each
    line on to Unity. Reconnects lazily on send failure; a message is
    dropped rather than blocking GPS processing if the relay is
    unreachable."""

    def __init__(self, host, port, verbose=False):
        self.host = host
        self.port = port
        self.verbose = verbose
        self.sock = None
        self.lock = threading.Lock()
        self._connect()

    def _connect(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5)
            s.connect((self.host, self.port))
            s.settimeout(None)
            self.sock = s
            if self.verbose:
                sys.stderr.write("Connected to relay server %s:%d\n" % (self.host, self.port))
        except OSError as e:
            if self.verbose:
                sys.stderr.write("Relay server connect failed: %s\n" % e)
            self.sock = None

    def send(self, obj):
        line = (json.dumps(obj) + "\n").encode("utf-8")
        with self.lock:
            if self.sock is None:
                self._connect()
            if self.sock is None:
                return
            try:
                self.sock.sendall(line)
            except OSError as e:
                if self.verbose:
                    sys.stderr.write("Relay send failed (%s); will reconnect next time\n" % e)
                try:
                    self.sock.close()
                except OSError:
                    pass
                self.sock = None

    def close(self):
        with self.lock:
            if self.sock:
                try:
                    self.sock.close()
                except OSError:
                    pass
                self.sock = None


class NtripClient(object):
    def __init__(self,
                 buffer=50,
                 user="",
                 out=sys.stdout,
                 port=2101,
                 caster="",
                 mountpoint="",
                 host=False,
                 lat=50,
                 lon=19,
                 height=300,
                 ssl=False,
                 verbose=False,
                 UDP_Port=None,
                 V2=False,
                 headerFile=sys.stderr,
                 headerOutput=False,
                 maxConnectTime=0,
                 gps_file_path=None,
                 fix_rate_ms=None,
                 gga_resend_interval=10,
                 relay_host=None,
                 relay_port=5002,
                 ):
        self.buffer = buffer
        self.user = base64.b64encode(bytes(user, 'utf-8')).decode("utf-8")
        self.out = out
        self.port = port
        self.caster = caster
        self.mountpoint = mountpoint
        self.setPosition(lat, lon)
        self.height = height
        self.verbose = verbose
        self.ssl = ssl
        self.host = host
        self.UDP_Port = UDP_Port
        self.V2 = V2
        self.headerFile = headerFile
        self.headerOutput = headerOutput
        self.maxConnectTime = maxConnectTime
        self.socket = None
        self.fix_rate_ms = fix_rate_ms
        self.gga_resend_interval = gga_resend_interval

        self.stream = serial.Serial('/dev/ttyS0', 115200, timeout=3)

        self.queryFirmwareVersion()

        if self.fix_rate_ms:
            self.configureFixRate(self.fix_rate_ms)

        self.nmr = NMEAReader(self.stream)

        self.gps_file_path = gps_file_path
        if self.gps_file_path:
            self.gps_file = open(self.gps_file_path, 'ab')
        else:
            self.gps_file = None

        if relay_host:
            self.relay = ServerRelay(relay_host, relay_port, verbose=self.verbose)
        else:
            self.relay = None

        self.imu_bus = None
        self.gyro_bias = 0.0
        self.heading = 0.0
        self.heading_lock = threading.Lock()
        self.last_position = None
        try:
            bus = SMBus(1)
            bus.write_byte_data(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0)
            time.sleep(0.1)
            sys.stderr.write("Calibrating IMU gyro bias (keep it still)...\n")
            self.gyro_bias = calibrate_gyro_z(bus)
            self.imu_bus = bus
            sys.stderr.write("IMU ready. Gyro bias: %.2f deg/s\n" % self.gyro_bias)
            threading.Thread(target=self.runImuLoop, daemon=True).start()
        except OSError as e:
            sys.stderr.write("IMU (GY-521) not available (%s) - heading will be omitted.\n" % e)

        if UDP_Port:
            self.UDP_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.UDP_socket.bind(('', 0))
            self.UDP_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        else:
            self.UDP_socket = None

    def _sendAndCapture(self, cmd_bytes, label, wait_seconds=2):
        """Write a command to the serial port and capture whatever comes
        back for wait_seconds. Returns the raw bytes captured."""
        sys.stderr.write("Sending %s: %s" % (label, cmd_bytes.decode('ascii', errors='replace')))
        self.stream.write(cmd_bytes)
        self.stream.flush()

        time.sleep(0.3)
        response = b""
        deadline = time.time() + wait_seconds
        while time.time() < deadline:
            waiting = self.stream.in_waiting
            if waiting:
                response += self.stream.read(waiting)
            else:
                time.sleep(0.05)
        return response

    def queryFirmwareVersion(self):
        """Send $PQTMVERNO and print the module's firmware string, so it
        can be compared against known forum reports of the fix-rate bug."""
        cmd = build_pqtm_command("PQTMVERNO")
        response = self._sendAndCapture(cmd, "firmware version query")
        if b"PQTMVERNO" in response:
            for line in response.split(b"\r\n"):
                if b"PQTMVERNO" in line:
                    sys.stderr.write("Firmware version: %s\n" % line.decode('ascii', errors='replace'))
        else:
            sys.stderr.write("No PQTMVERNO response seen (raw bytes: %s)\n" % response[:200])

    def configureFixRate(self, rate_ms):
        """Attempt to set the GNSS fix/output rate. Tries PQTMCFGFIXRATE
        first, then falls back to PAIR050 if no clear ACK was seen -
        two different command families reported in Quectel forum threads
        as behaving differently across LC29H(DA) firmware builds.
        Known issue: some LC29H(DA) firmware silently ignores or rejects
        both, per multiple Quectel forum reports as of mid-2026."""

        # Attempt 1: PQTMCFGFIXRATE
        cmd = build_pqtm_command("PQTMCFGFIXRATE,W,%d" % rate_ms)
        response = self._sendAndCapture(cmd, "PQTMCFGFIXRATE")

        pqtm_ok = False
        if b"PQTMCFGFIXRATE" in response:
            if b"ERROR" in response:
                sys.stderr.write(
                    "PQTMCFGFIXRATE REJECTED by module.\nResponse: %s\n" % response
                )
            elif b"OK" in response:
                sys.stderr.write("PQTMCFGFIXRATE ACCEPTED. Response: %s\n" % response)
                pqtm_ok = True
            else:
                sys.stderr.write("PQTMCFGFIXRATE response (unclear): %s\n" % response)
        else:
            sys.stderr.write(
                "No PQTMCFGFIXRATE response seen (module likely ignored it silently).\n"
            )

        if pqtm_ok:
            return

        # Attempt 2: PAIR050 fallback (different command family, some
        # forum reports suggest it behaves differently on some builds)
        sys.stderr.write("Trying PAIR050 fallback...\n")
        # PAIR sentences use the same $...*checksum framing as PQTM ones,
        # just a different payload prefix - reuse the same checksum logic.
        pair_payload = "PAIR050,%d" % rate_ms
        pair_cmd = ("$%s*%s\r\n" % (pair_payload, nmea_checksum(pair_payload))).encode('ascii')
        response2 = self._sendAndCapture(pair_cmd, "PAIR050")

        if b"PAIR001" in response2 or b"PAIR050" in response2:
            sys.stderr.write("PAIR050 response: %s\n" % response2)
        else:
            sys.stderr.write(
                "No PAIR050 response seen either (raw bytes: %s).\n"
                "Both command families were ignored/rejected - this points to a "
                "firmware-level limitation rather than a syntax issue. Check the "
                "firmware version above against Quectel forum reports, and look "
                "for a firmware update from Quectel/Waveshare.\n" % response2[:200]
            )

    def setPosition(self, lat, lon):
        self.flagN = "N"
        self.flagE = "E"
        if lon > 180:
            lon = (lon - 360) * -1
            self.flagE = "W"
        elif (lon < 0 and lon >= -180):
            lon = lon * -1
            self.flagE = "W"
        elif lon < -180:
            lon = lon + 360
            self.flagE = "E"
        else:
            self.lon = lon
        if lat < 0:
            lat = lat * -1
            self.flagN = "S"
        self.lonDeg = int(lon)
        self.latDeg = int(lat)
        self.lonMin = (lon - self.lonDeg) * 60
        self.latMin = (lat - self.latDeg) * 60

    def runImuLoop(self, rate_hz=10):
        """Runs on its own thread so heading updates at ~rate_hz regardless
        of how slowly GPS fixes arrive (~1Hz). Sends a lightweight
        {"type":"imu"} message per tick - readData() still periodically
        re-anchors self.heading to the GPS-derived course, so this loop
        never needs to know about position itself. Heading only - no
        position/speed estimation from the IMU."""
        interval = 1.0 / rate_hz
        last_time = time.time()
        while True:
            time.sleep(interval)
            now = time.time()
            dt = now - last_time
            last_time = now

            # Negated: the board is mounted upside-down, so its Z-axis points
            # down instead of up, flipping the sign of the raw yaw rate.
            gyro_z = -(read_gyro_z(self.imu_bus) - self.gyro_bias)
            with self.heading_lock:
                self.heading = (self.heading + gyro_z * dt) % 360
                heading = self.heading

            if self.relay:
                self.relay.send({"type": "imu", "heading": heading, "timestamp": now})

    def getMountPointBytes(self):
        mountPointString = "GET %s HTTP/1.1\r\nUser-Agent: %s\r\nAuthorization: Basic %s\r\n" % (
            self.mountpoint, useragent, self.user)
        if self.host or self.V2:
            hostString = "Host: %s:%i\r\n" % (self.caster, self.port)
            mountPointString += hostString
        if self.V2:
            mountPointString += "Ntrip-Version: Ntrip/2.0\r\n"
        mountPointString += "\r\n"
        if self.verbose:
            print(mountPointString)
        return bytes(mountPointString, 'ascii')

    def getGGABytes(self):
        while True:
            (raw_data, parsed_data) = self.nmr.read()
            if bytes("GNGGA", 'ascii') in raw_data:
                return raw_data

    def __del__(self):
        if getattr(self, "gps_file", None):
            self.gps_file.close()
        if getattr(self, "relay", None):
            self.relay.close()

    def parse_gngga_sentence(self, nmea_sentence):
        parts = nmea_sentence.split(',')
        if len(parts) < 10:
            return None

        try:
            lat = float(parts[2][:2]) + float(parts[2][2:]) / 60.0
            if parts[3] == 'S':
                lat = -lat
            lon = float(parts[4][:3]) + float(parts[4][3:]) / 60.0
            if parts[5] == 'W':
                lon = -lon
            alt = float(parts[9]) if parts[9] else 0.0
            fix_quality = int(parts[6]) if parts[6].isdigit() else 0
        except ValueError:
            return None
        return lat, lon, alt, fix_quality

    def readData(self):
        reconnectTry = 1
        sleepTime = 1
        if self.maxConnectTime > 0:
            EndConnect = datetime.timedelta(seconds=maxConnectTime)
        try:
            while reconnectTry <= maxReconnect:
                time.sleep(1)
                found_header = False
                if self.verbose:
                    sys.stderr.write('Connection {0} of {1}\n'.format(reconnectTry, maxReconnect))

                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                if self.ssl:
                    import ssl as ssl_module
                    self.socket = ssl_module.wrap_socket(self.socket)

                try:
                    error_indicator = self.socket.connect_ex((self.caster, self.port))
                except OSError as e:
                    # connect_ex() normally reports connection failures via its
                    # return code, but DNS lookup failures (e.g. right after
                    # boot, before the network's DNS is fully up) raise instead
                    # - fold that into the same retry path rather than crashing.
                    if self.verbose:
                        sys.stderr.write("Connection to caster failed: %s\n" % e)
                    error_indicator = -1
                if error_indicator == 0:
                    sleepTime = 1
                    connectTime = datetime.datetime.now()

                    self.socket.settimeout(10)
                    self.socket.sendall(self.getMountPointBytes())
                    while not found_header:
                        casterResponse = self.socket.recv(4096)
                        header_lines = casterResponse.decode('utf-8').split("\r\n")

                        for line in header_lines:
                            if line == "":
                                if not found_header:
                                    found_header = True
                                    if self.verbose:
                                        sys.stderr.write("End Of Header" + "\n")
                            else:
                                if self.verbose:
                                    sys.stderr.write("Header: " + line + "\n")
                            if self.headerOutput:
                                self.headerFile.write(line + "\n")

                        for line in header_lines:
                            if line.find("SOURCETABLE") >= 0:
                                sys.stderr.write("Mount point does not exist")
                                sys.exit(1)
                            elif line.find("401 Unauthorized") >= 0:
                                sys.stderr.write("Unauthorized request\n")
                                sys.exit(1)
                            elif line.find("404 Not Found") >= 0:
                                sys.stderr.write("Mount Point does not exist\n")
                                sys.exit(2)
                            elif (line.find("ICY 200 OK") >= 0
                                  or line.find("HTTP/1.0 200 OK") >= 0
                                  or line.find("HTTP/1.1 200 OK") >= 0):
                                self.socket.sendall(self.getGGABytes())

                    data = "Initial data"
                    lastGGAsend = time.time()

                    while data:
                        try:
                            data = self.socket.recv(self.buffer)
                            self.stream.write(data)

                            (raw_data, parsed_data) = self.nmr.read()
                            if bytes("GNGGA", 'ascii') in raw_data:
                                print(raw_data)
                                location = self.parse_gngga_sentence(raw_data.decode('ascii'))
                                if location:
                                    lat, lon, alt, fix_quality = location
                                    fix_status = {
                                        0: "Invalid",
                                        1: "GPS fix",
                                        2: "DGPS fix",
                                        3: "PPS fix",
                                        4: "RTK fixed",
                                        5: "RTK float",
                                        6: "Estimated",
                                        7: "Manual input",
                                        8: "Simulation"
                                    }.get(fix_quality, "Unknown")
                                    print(f"Parsed Location: Latitude={lat}, Longitude={lon}, Fix Status={fix_status}")

                                    # GPS course is only reliable once the rover has
                                    # moved enough that fix noise isn't the dominant
                                    # signal - re-anchor heading here so the fast IMU
                                    # thread's gyro integration never drifts for more
                                    # than one fix interval.
                                    if self.last_position is not None:
                                        dist = meters_distance(self.last_position[0], self.last_position[1], lat, lon)
                                        if dist > 0.5:
                                            with self.heading_lock:
                                                self.heading = compute_bearing(
                                                    self.last_position[0], self.last_position[1], lat, lon)
                                    self.last_position = (lat, lon)

                                    with self.heading_lock:
                                        heading = self.heading

                                    if self.relay:
                                        self.relay.send({
                                            "lat": lat,
                                            "lon": lon,
                                            "alt": alt,
                                            "fixQuality": fix_quality,
                                            "fixStatus": fix_status,
                                            "heading": heading,
                                            "timestamp": time.time(),
                                        })
                                if self.gps_file:
                                    self.gps_file.write(raw_data)
                                    self.gps_file.flush()

                            if self.UDP_socket:
                                self.UDP_socket.sendto(data, ('<broadcast>', self.UDP_Port))

                            # Periodically resend GGA so the VRS caster keeps
                            # generating corrections for your current position
                            if time.time() - lastGGAsend >= self.gga_resend_interval:
                                self.socket.sendall(self.getGGABytes())
                                lastGGAsend = time.time()
                                if self.verbose:
                                    sys.stderr.write("Resent GGA to caster\n")

                            if self.maxConnectTime:
                                if datetime.datetime.now() > connectTime + EndConnect:
                                    if self.verbose:
                                        sys.stderr.write("Connection Timed exceeded\n")
                                    sys.exit(0)

                        except socket.timeout:
                            if self.verbose:
                                sys.stderr.write('Connection TimedOut\n')
                            data = False
                        except socket.error as e:
                            if self.verbose:
                                sys.stderr.write('Connection Error: %s\n' % e)
                            data = False

                    if self.verbose:
                        sys.stderr.write('Closing Connection\n')
                    self.socket.close()
                    self.socket = None

                    if reconnectTry < maxReconnect:
                        sys.stderr.write("%s No Connection to NtripCaster.  Trying again in %i seconds\n" % (
                            datetime.datetime.now(), sleepTime))
                        time.sleep(sleepTime)
                        sleepTime = factor
                        if sleepTime > maxReconnectTime:
                            sleepTime = maxReconnectTime
                    else:
                        sys.exit(1)

                    reconnectTry += 1
                else:
                    self.socket = None
                    if self.verbose:
                        print("Error indicator: ", error_indicator)

                    if reconnectTry < maxReconnect:
                        sys.stderr.write("%s No Connection to NtripCaster.  Trying again in %i seconds\n" % (
                            datetime.datetime.now(), sleepTime))
                        time.sleep(sleepTime)
                        sleepTime = factor
                        if sleepTime > maxReconnectTime:
                            sleepTime = maxReconnectTime
                    reconnectTry += 1

        except KeyboardInterrupt:
            if self.socket:
                self.socket.close()
            self.stream.close()
            if self.relay:
                self.relay.close()
            sys.exit()


if __name__ == '__main__':
    usage = "gps.py [options] [caster] [port] mountpoint"
    parser = OptionParser(version=version, usage=usage)
    parser.add_option("-u", "--user", type="string", dest="user", default="IBS",
                       help="The Ntripcaster username.  Default: %default")
    parser.add_option("-p", "--password", type="string", dest="password", default="IBS",
                       help="The Ntripcaster password. Default: %default")
    parser.add_option("-o", "--org", type="string", dest="org",
                       help="Use IBSS and the provided organization for the user. Caster and Port are not needed in this case Default: %default")
    parser.add_option("-b", "--baseorg", type="string", dest="baseorg",
                       help="The org that the base is in. IBSS Only, assumed to be the user org")
    parser.add_option("-t", "--latitude", type="float", dest="lat", default=50.09,
                       help="Your latitude.  Default: %default")
    parser.add_option("-g", "--longitude", type="float", dest="lon", default=8.66,
                       help="Your longitude.  Default: %default")
    parser.add_option("-e", "--height", type="float", dest="height", default=1200,
                       help="Your ellipsoid height.  Default: %default")
    parser.add_option("-v", "--verbose", action="store_true", dest="verbose", default=True, help="Verbose")
    parser.add_option("-s", "--ssl", action="store_true", dest="ssl", default=False, help="Use SSL for the connection")
    parser.add_option("-H", "--host", action="store_true", dest="host", default=False,
                       help="Include host header, should be on for IBSS")
    parser.add_option("-r", "--Reconnect", type="int", dest="maxReconnect", default=1,
                       help="Number of reconnections")
    parser.add_option("-D", "--UDP", type="int", dest="UDP", default=None,
                       help="Broadcast recieved data on the provided port")
    parser.add_option("-2", "--V2", action="store_true", dest="V2", default=False,
                       help="Make a NTRIP V2 Connection")
    parser.add_option("-f", "--outputFile", type="string", dest="outputFile", default=None,
                       help="Write to this file, instead of stdout")
    parser.add_option("-m", "--maxtime", type="int", dest="maxConnectTime", default=None,
                       help="Maximum length of the connection, in seconds")
    parser.add_option("--Header", action="store_true", dest="headerOutput", default=False,
                       help="Write headers to stderr")
    parser.add_option("--HeaderFile", type="string", dest="headerFile", default=None,
                       help="Write headers to this file, instead of stderr.")
    parser.add_option("--fixrate", type="int", dest="fixrate", default=None,
                       help="Attempt to set GNSS fix rate in milliseconds via PQTMCFGFIXRATE "
                            "(e.g. 100 for 10Hz, 200 for 5Hz). May be rejected on some LC29H(DA) firmware.")
    parser.add_option("--ggainterval", type="int", dest="ggainterval", default=10,
                       help="Seconds between GGA resends to the caster while connected. Default: %default")
    parser.add_option("--relayhost", type="string", dest="relayhost", default=None,
                       help="Hostname/IP of the Domain-Expansion-Server C# relay (its public IP or "
                            "dynamic DNS name, since the caster and PC are on different networks). "
                            "If omitted, no data is relayed to Unity.")
    parser.add_option("--relayport", type="int", dest="relayport", default=5002,
                       help="Port the C# relay is listening on for the Pi. Default: %default")

    (options, args) = parser.parse_args()
    ntripArgs = {}
    ntripArgs['lat'] = options.lat
    ntripArgs['lon'] = options.lon
    ntripArgs['height'] = options.height
    ntripArgs['host'] = options.host
    ntripArgs['fix_rate_ms'] = options.fixrate
    ntripArgs['gga_resend_interval'] = options.ggainterval
    ntripArgs['relay_host'] = options.relayhost
    ntripArgs['relay_port'] = options.relayport

    if options.outputFile:
        ntripArgs['gps_file_path'] = options.outputFile

    if options.ssl:
        ntripArgs['ssl'] = True
    else:
        ntripArgs['ssl'] = False

    if options.org:
        if len(args) != 1:
            print("Incorrect number of arguments for IBSS. You do not need to provide the server and port\n")
            parser.print_help()
            sys.exit(1)
        ntripArgs['user'] = options.user + "." + options.org + ":" + options.password
        if options.baseorg:
            ntripArgs['caster'] = options.baseorg + ".ibss.trimbleos.com"
        else:
            ntripArgs['caster'] = options.org + ".ibss.trimbleos.com"
        if options.ssl:
            ntripArgs['port'] = 52101
        else:
            ntripArgs['port'] = 2101
        ntripArgs['mountpoint'] = args[0]
    else:
        if len(args) != 3:
            print("Incorrect number of arguments for NTRIP\n")
            parser.print_help()
            sys.exit(1)
        ntripArgs['user'] = options.user + ":" + options.password
        ntripArgs['caster'] = args[0]
        ntripArgs['port'] = int(args[1])
        ntripArgs['mountpoint'] = args[2]

    if ntripArgs['mountpoint'][0:1] != "/":
        ntripArgs['mountpoint'] = "/" + ntripArgs['mountpoint']

    ntripArgs['V2'] = options.V2
    ntripArgs['verbose'] = options.verbose
    ntripArgs['headerOutput'] = options.headerOutput

    if options.UDP:
        ntripArgs['UDP_Port'] = int(options.UDP)

    maxReconnect = options.maxReconnect
    maxConnectTime = options.maxConnectTime

    if options.verbose:
        print("Server: " + ntripArgs['caster'])
        print("Port: " + str(ntripArgs['port']))
        print("User: " + ntripArgs['user'])
        print("mountpoint: " + ntripArgs['mountpoint'])
        print("Reconnects: " + str(maxReconnect))
        print("Max Connect Time: " + str(maxConnectTime))
        print("GGA resend interval: " + str(options.ggainterval) + "s")
        if options.fixrate:
            print("Requested fix rate: %d ms (%.1f Hz)" % (options.fixrate, 1000.0 / options.fixrate))
        if options.relayhost:
            print("Relaying fixes to: %s:%d" % (options.relayhost, options.relayport))
        else:
            print("Relay: disabled (pass --relayhost to enable)")
        if ntripArgs['V2']:
            print("NTRIP: V2")
        else:
            print("NTRIP: V1")
        if ntripArgs["ssl"]:
            print("SSL Connection")
        else:
            print("Uncrypted Connection")
        print("")

    fileOutput = False
    if options.outputFile:
        f = open(options.outputFile, 'wb')
        ntripArgs['out'] = f
        fileOutput = True

    if options.headerFile:
        h = open(options.headerFile, 'w')
        ntripArgs['headerFile'] = h
        ntripArgs['headerOutput'] = True

    n = NtripClient(**ntripArgs)
    try:
        n.readData()
    finally:
        if fileOutput:
            f.close()
        if options.headerFile:
            h.close()
