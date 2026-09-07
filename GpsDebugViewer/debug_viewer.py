"""Standalone GPS debugging tool - completely independent of Unity.

Purpose: isolate whether "character isn't moving sometimes" is a Unity bug
or an upstream problem (Pi / network / GPS hardware itself). Run this INSTEAD
of Server/Program.cs for a debugging session - it's a drop-in replacement for
the "receive from Pi" half of that server:

    Pi (gps.py, unchanged) --UDP--> this tool --UDP--> Unity (unchanged)
                                        |
                                        +--> http://localhost:8000 (live view)

It listens on the same port the Pi already sends to (5002), forwards every
message on to Unity exactly as Program.cs would (so Unity keeps working
normally, no need to change gps.py or any port forwarding), and ALSO shows a
live webpage with a grid, current position, breadcrumb trail, and staleness
stats.

How to use it: stop the C# server, run this script on the same machine
instead, then watch both Unity and http://localhost:8000 side by side.
    - If Unity freezes but this page keeps updating smoothly -> Unity-side bug.
    - If this page ALSO freezes/goes stale -> the problem is upstream of
      Unity entirely (Pi, network, or the GPS module), not Unity's fault.

Run: python3 debug_viewer.py
(all ports are overridable - see --help)
"""

import argparse
import json
import math
import socket
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

METERS_PER_DEGREE_LAT = 111320.0
MAX_HISTORY = 1000
STALE_AFTER_SECONDS = 2.0


class SharedState:
    """All fields are written only from the UDP receiver thread and read
    only from HTTP handler threads (each request creates a fresh dict
    snapshot under the lock), so this is the one piece of shared mutable
    state in the whole tool."""

    def __init__(self):
        self.lock = threading.Lock()
        self.origin_lat = None
        self.origin_lon = None
        self.history = deque(maxlen=MAX_HISTORY)  # dicts: x, z, t
        self.latest_fix = None  # raw dict from the Pi's GPS message
        self.latest_heading = None
        self.latest_heading_time = None
        self.last_message_time = None
        self.last_fix_time = None
        self.last_fix_gap_ms = None
        self.fix_count = 0
        self.last_seq = None
        self.seq_gap_count = 0

    def reset(self):
        with self.lock:
            self.origin_lat = None
            self.origin_lon = None
            self.history.clear()
            self.latest_fix = None
            self.last_seq = None
            self.seq_gap_count = 0
            self.fix_count = 0
            self.last_fix_gap_ms = None

    def record_message(self):
        with self.lock:
            self.last_message_time = time.time()

    def record_fix(self, fix):
        now = time.time()
        with self.lock:
            if self.origin_lat is None:
                self.origin_lat = fix["lat"]
                self.origin_lon = fix["lon"]

            meters_per_degree_lon = METERS_PER_DEGREE_LAT * math.cos(math.radians(self.origin_lat))
            x = (fix["lon"] - self.origin_lon) * meters_per_degree_lon
            z = (fix["lat"] - self.origin_lat) * METERS_PER_DEGREE_LAT

            self.history.append({"x": x, "z": z, "t": now})
            self.latest_fix = fix
            self.fix_count += 1

            if self.last_fix_time is not None:
                self.last_fix_gap_ms = (now - self.last_fix_time) * 1000.0
            self.last_fix_time = now

            seq = fix.get("seq")
            if seq is not None and self.last_seq is not None and seq != self.last_seq + 1:
                self.seq_gap_count += 1
            if seq is not None:
                self.last_seq = seq

    def record_heading(self, heading):
        with self.lock:
            self.latest_heading = heading
            self.latest_heading_time = time.time()

    def snapshot(self):
        now = time.time()
        with self.lock:
            return {
                "history": list(self.history),
                "latestFix": self.latest_fix,
                "latestHeading": self.latest_heading,
                "headingAgeSec": (now - self.latest_heading_time) if self.latest_heading_time else None,
                "fixAgeSec": (now - self.last_fix_time) if self.last_fix_time else None,
                "messageAgeSec": (now - self.last_message_time) if self.last_message_time else None,
                "fixCount": self.fix_count,
                "lastFixGapMs": self.last_fix_gap_ms,
                "seqGapCount": self.seq_gap_count,
                "originLat": self.origin_lat,
                "originLon": self.origin_lon,
            }


def run_udp_receiver(state, pi_port, unity_addr, verbose):
    listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listen_sock.bind(("", pi_port))
    print("Listening for the Pi on UDP port %d..." % pi_port)

    forward_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if unity_addr else None
    if unity_addr:
        print("Forwarding every message on to Unity at %s:%d" % unity_addr)
    else:
        print("Forwarding to Unity disabled (--no-forward) - Unity will see nothing while this runs.")

    while True:
        data, _addr = listen_sock.recvfrom(65535)

        if forward_sock:
            try:
                forward_sock.sendto(data, unity_addr)
            except OSError as e:
                print("Forward to Unity failed: %s" % e)

        state.record_message()

        try:
            msg = json.loads(data.decode("utf-8").strip())
        except (ValueError, UnicodeDecodeError) as e:
            print("Ignoring unparseable message (%s)" % e)
            continue

        now_str = time.strftime("%H:%M:%S", time.localtime()) + (".%03d" % (time.time() % 1 * 1000))

        if msg.get("type") == "imu":
            state.record_heading(msg.get("heading"))
            if verbose:
                print("[%s] IMU heading=%.1f" % (now_str, msg.get("heading", -1)))
        elif "lat" in msg:
            state.record_fix(msg)
            gap = state.snapshot()["lastFixGapMs"]
            gap_str = ("%.0fms" % gap) if gap is not None else "n/a"
            print("[%s] Fix #%s: lat=%.7f lon=%.7f (gap since last fix: %s)" % (
                now_str, msg.get("seq", "?"), msg["lat"], msg["lon"], gap_str))
        else:
            if verbose:
                print("[%s] Other message: %s" % (now_str, msg))


PAGE_HTML = """<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>GPS Debug Viewer</title>
<style>
  body { background:#111; color:#ddd; font-family: monospace; margin:0; padding:16px; }
  h1 { font-size:16px; margin:0 0 12px; color:#8cf; }
  #layout { display:flex; gap:16px; flex-wrap:wrap; }
  canvas { background:#000; border:1px solid #444; }
  #stats div { margin-bottom:4px; }
  .stale { color:#f55; font-weight:bold; }
  .ok { color:#5f5; }
  button { background:#333; color:#ddd; border:1px solid #666; padding:6px 10px; cursor:pointer; margin-top:8px; }
  button:hover { background:#444; }
</style>
</head>
<body>
<h1>GPS Debug Viewer - independent of Unity</h1>
<div id="layout">
  <canvas id="canvas" width="700" height="600"></canvas>
  <div id="stats">
    <div>Fix count: <span id="fixCount">-</span></div>
    <div>Last fix age: <span id="fixAge">-</span></div>
    <div>Last message age (any type): <span id="msgAge">-</span></div>
    <div>Gap since previous fix: <span id="fixGap">-</span></div>
    <div>Sequence gaps detected: <span id="seqGaps">-</span></div>
    <div>Heading: <span id="heading">-</span></div>
    <div>Heading age: <span id="headingAge">-</span></div>
    <div>Lat/Lon: <span id="latlon">-</span></div>
    <div>Local X/Z (m): <span id="xz">-</span></div>
    <button onclick="fetch('/reset', {method:'POST'})">Reset / re-anchor origin</button>
  </div>
</div>
<script>
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');

function niceStep(range) {
  const steps = [0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000];
  for (const s of steps) {
    if (range / s <= 12) return s;
  }
  return 1000;
}

function draw(data) {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const history = data.history || [];

  if (history.length === 0) {
    ctx.fillStyle = '#888';
    ctx.font = '16px monospace';
    ctx.fillText('Waiting for GPS data from the Pi...', 20, 40);
    return;
  }

  let minX = Infinity, maxX = -Infinity, minZ = Infinity, maxZ = -Infinity;
  for (const p of history) {
    minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x);
    minZ = Math.min(minZ, p.z); maxZ = Math.max(maxZ, p.z);
  }
  const pad = 2;
  minX -= pad; maxX += pad; minZ -= pad; maxZ += pad;
  const rangeX = Math.max(maxX - minX, 1);
  const rangeZ = Math.max(maxZ - minZ, 1);
  const centerX = (minX + maxX) / 2, centerZ = (minZ + maxZ) / 2;
  const scale = Math.min(canvas.width / rangeX, canvas.height / rangeZ) * 0.85;

  function toScreen(x, z) {
    return [
      canvas.width / 2 + (x - centerX) * scale,
      canvas.height / 2 - (z - centerZ) * scale,
    ];
  }

  // Grid
  const step = niceStep(Math.max(rangeX, rangeZ));
  ctx.strokeStyle = '#223'; ctx.lineWidth = 1; ctx.font = '10px monospace'; ctx.fillStyle = '#556';
  const startX = Math.floor(minX / step) * step;
  for (let gx = startX; gx <= maxX; gx += step) {
    const [sx] = toScreen(gx, 0);
    ctx.beginPath(); ctx.moveTo(sx, 0); ctx.lineTo(sx, canvas.height); ctx.stroke();
    ctx.fillText(gx.toFixed(0) + 'm', sx + 2, 12);
  }
  const startZ = Math.floor(minZ / step) * step;
  for (let gz = startZ; gz <= maxZ; gz += step) {
    const [, sy] = toScreen(0, gz);
    ctx.beginPath(); ctx.moveTo(0, sy); ctx.lineTo(canvas.width, sy); ctx.stroke();
    ctx.fillText(gz.toFixed(0) + 'm', 2, sy - 2);
  }

  // Trail
  ctx.strokeStyle = '#4af'; ctx.lineWidth = 2;
  ctx.beginPath();
  history.forEach((p, i) => {
    const [sx, sy] = toScreen(p.x, p.z);
    if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
  });
  ctx.stroke();

  // Current position + heading arrow
  const last = history[history.length - 1];
  const [lx, ly] = toScreen(last.x, last.z);
  ctx.fillStyle = '#fff';
  ctx.beginPath(); ctx.arc(lx, ly, 6, 0, Math.PI * 2); ctx.fill();

  if (data.latestHeading !== null && data.latestHeading !== undefined) {
    const theta = data.latestHeading * Math.PI / 180;
    const dx = Math.sin(theta), dy = -Math.cos(theta);
    ctx.strokeStyle = '#fa4'; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(lx, ly); ctx.lineTo(lx + dx * 24, ly + dy * 24); ctx.stroke();
  }
}

function setStat(id, text, staleClass) {
  const el = document.getElementById(id);
  el.textContent = text;
  el.className = staleClass || '';
}

async function tick() {
  try {
    const res = await fetch('/data.json');
    const data = await res.json();
    draw(data);

    setStat('fixCount', data.fixCount);
    setStat('fixAge', data.fixAgeSec !== null ? data.fixAgeSec.toFixed(1) + 's' : 'n/a',
      (data.fixAgeSec !== null && data.fixAgeSec > 2) ? 'stale' : 'ok');
    setStat('msgAge', data.messageAgeSec !== null ? data.messageAgeSec.toFixed(1) + 's' : 'n/a',
      (data.messageAgeSec !== null && data.messageAgeSec > 2) ? 'stale' : 'ok');
    setStat('fixGap', data.lastFixGapMs !== null ? data.lastFixGapMs.toFixed(0) + 'ms' : 'n/a');
    setStat('seqGaps', data.seqGapCount);
    setStat('heading', data.latestHeading !== null && data.latestHeading !== undefined ? data.latestHeading.toFixed(1) + 'deg' : 'n/a');
    setStat('headingAge', data.headingAgeSec !== null ? data.headingAgeSec.toFixed(1) + 's' : 'n/a',
      (data.headingAgeSec !== null && data.headingAgeSec > 2) ? 'stale' : 'ok');
    if (data.latestFix) {
      setStat('latlon', data.latestFix.lat.toFixed(7) + ', ' + data.latestFix.lon.toFixed(7));
    }
    if (data.history && data.history.length) {
      const last = data.history[data.history.length - 1];
      setStat('xz', last.x.toFixed(2) + ', ' + last.z.toFixed(2));
    }
  } catch (e) {
    console.error(e);
  }
}

setInterval(tick, 200);
tick();
</script>
</body>
</html>
"""


def make_handler(state):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass  # Quiet - the UDP receiver thread already prints what matters.

        def do_GET(self):
            if self.path == "/" or self.path == "/index.html":
                body = PAGE_HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/data.json":
                body = json.dumps(state.snapshot()).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_response(404)
                self.end_headers()

        def do_POST(self):
            if self.path == "/reset":
                state.reset()
                self.send_response(200)
                self.end_headers()
            else:
                self.send_response(404)
                self.end_headers()

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pi-port", type=int, default=5002, help="UDP port to receive from the Pi on (default: 5002, matches Server/Program.cs)")
    parser.add_argument("--unity-port", type=int, default=5001, help="UDP port to forward messages to on localhost for Unity (default: 5001)")
    parser.add_argument("--no-forward", action="store_true", help="Don't forward to Unity at all - use this to test the Pi/network in isolation")
    parser.add_argument("--http-port", type=int, default=8000, help="Port for the live debug webpage (default: 8000)")
    parser.add_argument("--verbose", action="store_true", help="Also log IMU heading messages, not just GPS fixes")
    args = parser.parse_args()

    state = SharedState()
    unity_addr = None if args.no_forward else ("127.0.0.1", args.unity_port)

    udp_thread = threading.Thread(target=run_udp_receiver, args=(state, args.pi_port, unity_addr, args.verbose), daemon=True)
    udp_thread.start()

    server = ThreadingHTTPServer(("", args.http_port), make_handler(state))
    print("Debug page: http://localhost:%d" % args.http_port)
    server.serve_forever()


if __name__ == "__main__":
    main()
