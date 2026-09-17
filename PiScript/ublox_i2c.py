"""u-blox GNSS receiver over I2C (DDC), exposed as a serial-like stream.

Gives a u-blox receiver on I2C the read()/readline()/write()/close()
interface that pynmeagps' NMEAReader and pyubx2's UBXReader expect from a
serial port, so gps.py's reader thread works unchanged.

u-blox DDC protocol, per the integration manual:
  - Registers 0xFD/0xFE hold the number of bytes waiting (MSB, LSB).
  - Register 0xFF is the data stream itself.
  - Writes carry no register byte, but MUST be at least 2 bytes long - a
    1-byte write is interpreted as setting the read-address pointer, not as
    data.

Requires: pip install smbus2 pyubx2
"""

import threading
import time

from pyubx2 import (
    ERR_IGNORE,
    SET_LAYER_RAM,
    TXN_NONE,
    UBX_PROTOCOL,
    UBXMessage,
    UBXReader,
)
from smbus2 import SMBus, i2c_msg

DEFAULT_ADDRESS = 0x42

_REG_BYTES_AVAILABLE = 0xFD
_REG_DATA = 0xFF

# Per-transfer sizes. Larger means fewer transactions but holds the bus
# longer each time, and the BNO08x IMU shares this bus - keep them short
# enough that its transfers can interleave.
_MAX_READ = 255
_MAX_WRITE = 128

_UBX_CFG_VALSET = (0x06, 0x8A)

# The ZED-F9P's maximum navigation rate depends on how many constellations
# it tracks (ZED-F9P data sheet, UBX-21044850, Table 1). Measured on this
# rig: all four defaults to ~9Hz no matter what CFG_RATE_MEAS asks for.
#   constellations        RTK    no RTK
#   gps,glo,gal,bds        7Hz     9Hz
#   gps,glo,gal           10Hz    10Hz
#   gps,gal               15Hz    20Hz
#   gps,glo               14Hz    20Hz
#   gps                   20Hz    25Hz
# Fewer constellations means fewer satellites, which makes RTK slower to
# converge and less robust under trees/buildings - rate vs. robustness.
_CONSTELLATION_KEYS = {
    "gps": "CFG_SIGNAL_GPS_ENA",
    "gal": "CFG_SIGNAL_GAL_ENA",
    "glo": "CFG_SIGNAL_GLO_ENA",
    "bds": "CFG_SIGNAL_BDS_ENA",
}


def parse_constellations(text):
    """'gps,gal' -> ('gps', 'gal'). GPS is required - every rate row above
    includes it."""
    names = tuple(sorted({part.strip().lower() for part in text.split(",") if part.strip()}))
    unknown = [n for n in names if n not in _CONSTELLATION_KEYS]
    if unknown:
        raise ValueError("unknown constellation(s) %s - choose from %s"
                         % (", ".join(unknown), ", ".join(_CONSTELLATION_KEYS)))
    if "gps" not in names:
        raise ValueError("gps must be enabled")
    return names


class UbloxI2C:
    def __init__(self, bus=1, address=DEFAULT_ADDRESS, timeout=3.0, poll_interval=0.01,
                 busy_timeout=2.0):
        self._bus = SMBus(bus)
        self._address = address
        self.timeout = timeout
        self._poll_interval = poll_interval
        self._busy_timeout = busy_timeout
        self._rx = bytearray()
        self._held_byte = b""
        # Reads happen on the NMEA reader thread and writes (RTCM) on the
        # caster thread; a compound register-select-then-read must not have
        # a write land in the middle of it.
        self._lock = threading.Lock()

    def _transfer(self, *msgs):
        """i2c_rdwr, retrying while the receiver briefly stops answering.

        The receiver NAKs its own address for a moment while it restarts its
        GNSS engine - which it does right after a constellation change (seen
        on this board as Errno 121 on the very next transfer). Without
        retrying, that restart looked like a missing receiver, and inside
        gps.py's caster loop an OSError is indistinguishable from a socket
        error, so it would also drop the NTRIP connection. Only a device
        that stays silent past busy_timeout is treated as gone."""
        deadline = time.monotonic() + self._busy_timeout
        while True:
            try:
                self._bus.i2c_rdwr(*msgs)
                return
            except OSError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.05)

    def _bytes_available(self):
        select = i2c_msg.write(self._address, [_REG_BYTES_AVAILABLE])
        result = i2c_msg.read(self._address, 2)
        self._transfer(select, result)
        msb, lsb = list(result)
        # Bit 15 occasionally reads back set - an undocumented glitch SparkFun
        # traced to I2C setup-time violations. Left alone it looks like ~32k
        # bytes are waiting.
        return ((msb << 8) | lsb) & 0x7FFF

    def _fill(self):
        """Pulls whatever the receiver has waiting into the local buffer.
        Returns how many bytes were added (0 if nothing was waiting)."""
        with self._lock:
            count = min(self._bytes_available(), _MAX_READ)
            if count == 0:
                return 0
            select = i2c_msg.write(self._address, [_REG_DATA])
            result = i2c_msg.read(self._address, count)
            self._transfer(select, result)
            self._rx.extend(bytes(result))
            return count

    def read(self, size=1):
        deadline = time.monotonic() + self.timeout
        while len(self._rx) < size:
            if self._fill() == 0:
                if time.monotonic() >= deadline:
                    break
                time.sleep(self._poll_interval)
        data = bytes(self._rx[:size])
        del self._rx[:size]
        return data

    def readline(self):
        deadline = time.monotonic() + self.timeout
        while True:
            newline = self._rx.find(b"\n")
            if newline >= 0:
                line = bytes(self._rx[: newline + 1])
                del self._rx[: newline + 1]
                return line
            if self._fill() == 0:
                if time.monotonic() >= deadline:
                    line = bytes(self._rx)
                    self._rx.clear()
                    return line
                time.sleep(self._poll_interval)

    def write(self, data):
        data = self._held_byte + bytes(data)
        self._held_byte = b""
        if len(data) < 2:
            # A lone byte would be taken as a register-pointer write rather
            # than data. Hold it for the next call - RTCM keeps arriving, so
            # it goes out with the next chunk.
            self._held_byte = data
            return len(data)

        with self._lock:
            start = 0
            while start < len(data):
                end = min(start + _MAX_WRITE, len(data))
                if len(data) - end == 1:
                    end -= 1  # never leave a 1-byte final transfer
                self._transfer(i2c_msg.write(self._address, data[start:end]))
                start = end
        return len(data)

    def close(self):
        self._bus.close()


def configure_ublox(stream, rate_ms=50, constellations=("gps",), log=print):
    """Applies the receiver settings gps.py relies on, in RAM only - the
    module's own saved configuration is never touched, and a power cycle
    restores it.

    Sent as separate messages on purpose: if any single key in a CFG-VALSET
    is invalid, the receiver rejects the entire message, so one bad key in a
    combined message would silently undo every other setting.

    Order matters too: trimming NMEA output down to GGA goes out BEFORE the
    rate change. Raising the rate first would briefly produce every default
    sentence (including a large GSV burst at 30+ satellites) at 20Hz, which
    can outrun I2C and push the later acknowledgements out of the receiver's
    transmit buffer.

    Constellations go in ONE message (u-blox requires signal changes to be
    applied together, and the receiver restarts its GNSS engine afterward),
    and before the rate - the rate above the constellation limit is silently
    capped otherwise.

    Returns True only if every group was acknowledged.
    """
    signal_keys = [(key, int(name in constellations)) for name, key in _CONSTELLATION_KEYS.items()]
    groups = [
        ("NMEA output: GGA only", [
            ("CFG_MSGOUT_NMEA_ID_GGA_I2C", 1),
            ("CFG_MSGOUT_NMEA_ID_GLL_I2C", 0),
            ("CFG_MSGOUT_NMEA_ID_GSA_I2C", 0),
            ("CFG_MSGOUT_NMEA_ID_GSV_I2C", 0),
            ("CFG_MSGOUT_NMEA_ID_RMC_I2C", 0),
            ("CFG_MSGOUT_NMEA_ID_VTG_I2C", 0),
        ]),
        ("high-precision NMEA coordinates", [("CFG_NMEA_HIGHPREC", 1)]),
        ("RTCM3 correction input on I2C", [("CFG_I2CINPROT_RTCM3X", 1)]),
        ("constellations: %s" % "+".join(constellations), signal_keys),
        ("measurement rate %dms (%.0fHz)" % (rate_ms, 1000.0 / rate_ms), [
            ("CFG_RATE_MEAS", rate_ms),
            ("CFG_RATE_NAV", 1),
        ]),
    ]

    all_ok = True
    for label, keys in groups:
        result = _send_valset(stream, keys)
        if result is True:
            log("u-blox config OK:   %s" % label)
        elif result is False:
            log("u-blox config NAK:  %s - receiver rejected it" % label)
            all_ok = False
        else:
            log("u-blox config ???:  %s - no response from receiver" % label)
            all_ok = False
    return all_ok


def _send_valset(stream, keys, timeout=2.0):
    """True on ACK, False on NAK, None if the receiver never answered."""
    stream.write(UBXMessage.config_set(SET_LAYER_RAM, TXN_NONE, keys).serialize())

    reader = UBXReader(stream, protfilter=UBX_PROTOCOL, quitonerror=ERR_IGNORE)
    previous_timeout = stream.timeout
    stream.timeout = 0.25
    try:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            _, parsed = reader.read()
            if parsed is None or parsed.identity not in ("ACK-ACK", "ACK-NAK"):
                continue
            if (parsed.clsID, parsed.msgID) == _UBX_CFG_VALSET:
                return parsed.identity == "ACK-ACK"
    finally:
        stream.timeout = previous_timeout
    return None
