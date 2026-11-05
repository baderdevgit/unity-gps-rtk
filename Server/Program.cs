using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Text.RegularExpressions;

const int piPort = 5002;
const int unityPort = 5001;
const int webPort = 5003;
const int relayPort = 5004;

// Shared secret for the mobile web UI's /reset and /snapshot endpoints,
// since this server is reachable from the internet. Set it via:
//   $env:RESET_AUTH_TOKEN = "your-secret-here"
// and keep it in sync with CameraSnapshotUploader.cs's authToken in Unity.

string authToken = Environment.GetEnvironmentVariable("RESET_AUTH_TOKEN") ?? "changeme-please-set-a-real-token";
if (authToken == "changeme-please-set-a-real-token")
{
    Console.WriteLine("WARNING: RESET_AUTH_TOKEN is not set - using an insecure placeholder token.");
    Console.WriteLine("Set it with: $env:RESET_AUTH_TOKEN = \"your-secret-here\"  (before starting the server)");
}

string indexHtml = File.ReadAllText(Path.Combine(AppContext.BaseDirectory, "wwwroot", "index.html"));

// Every fix line the Pi sends gets appended here, one JSON object per line,
// so past readings can be reviewed later (e.g. for accuracy testing).
string gpsLogPath = Path.Combine(AppContext.BaseDirectory, "gps_log.jsonl");
var gpsLogLock = new object();
Console.WriteLine($"Logging GPS fixes to {gpsLogPath}");

// Separate from gpsLogPath above: this only gets a line added when the
// mobile UI's "Log Position" button is pressed, not on every fix.
string loggedPositionsPath = Path.Combine(AppContext.BaseDirectory, "logged_positions.jsonl");
var loggedPositionsLock = new object();

var snapshots = new SnapshotStore();
var latestFix = new TextStore();
var maze = new MazeState();
var relay = new RelayState();

// Unity runs on this same PC, so the Server->Unity hop is loopback traffic -
// no "connection" to accept/track at all under UDP, just a fixed
// destination to send() to. This also deletes the old stuck-write/
// unityClients bookkeeping class of bug entirely (no longer applicable).
var unityEndpoint = new IPEndPoint(IPAddress.Loopback, unityPort);
var unitySendClient = new UdpClient();

_ = Task.Run(() => RunWebServer(webPort, authToken, indexHtml, unityEndpoint, unitySendClient, snapshots, latestFix, maze, relay, loggedPositionsPath, loggedPositionsLock));
_ = Task.Run(() => RunRelayListener(relayPort, relay));
await RunPiListener(piPort, unityEndpoint, unitySendClient, latestFix, gpsLogPath, gpsLogLock);

static async Task RunPiListener(int port, IPEndPoint unityEndpoint, UdpClient unitySendClient, TextStore latestFix, string gpsLogPath, object gpsLogLock)
{
    var udp = new UdpClient(port);
    Console.WriteLine($"Listening for Raspberry Pi on UDP port {port}...");

    DateTime? lastLineAt = null;
    while (true)
    {
        UdpReceiveResult result;
        try
        {
            result = await udp.ReceiveAsync();
        }
        catch (SocketException e)
        {
            // A previous send to an unreachable/closed remote port can
            // surface here as an ICMP-triggered error on some platforms -
            // log and keep listening rather than let the whole loop die.
            Console.WriteLine($"[{DateTime.UtcNow:HH:mm:ss.fff}] Pi listener socket error: {e.Message}");
            continue;
        }

        string line = Encoding.UTF8.GetString(result.Buffer);
        var now = DateTime.UtcNow;
        // GPS fixes arrive ~1Hz, IMU messages ~10Hz - a gap much bigger
        // than that means something upstream (Pi relay, network) is
        // stalling, not just normal spacing.
        if (lastLineAt.HasValue)
        {
            double gapMs = (now - lastLineAt.Value).TotalMilliseconds;
            if (gapMs > 1500)
                Console.WriteLine($"[{now:HH:mm:ss.fff}] [PERF] {gapMs:F0}ms gap since previous Pi message");
        }
        lastLineAt = now;

        Console.WriteLine($"[{now:HH:mm:ss.fff}] Received from Pi: {line}");

        // Both real fixes and {"type":"imu",...} messages carry a
        // "heading" field - log it plainly so a stuck/wrong value from the
        // BNO08x is visible directly here, without picking it out of the
        // raw JSON dump above by eye.
        var headingMatch = Regex.Match(line, "\"heading\":\\s*(-?[0-9.]+)");
        if (headingMatch.Success)
            Console.WriteLine($"[{now:HH:mm:ss.fff}] [HEADING] {headingMatch.Groups[1].Value}");

        // Control/IMU messages (e.g. {"type":"imu",...} at ~10Hz) share this
        // same relay port but aren't real GPS fixes - only cache/log the
        // real ones, or the mobile page's readout would mostly be reading
        // stale/incomplete IMU data instead.
        if (!line.Contains("\"type\":"))
        {
            latestFix.Set(line);
            lock (gpsLogLock)
            {
                File.AppendAllText(gpsLogPath, line + "\n");
            }
            // Lightweight extraction (not a full JSON parse) just to log
            // which fix this is - lines up against gps.py's "[SEQ] Pi
            // sending fix #N" and Unity's "[SEQ] Unity processed fix #N" to
            // isolate exactly which hop a given fix's delay happens on.
            var seqMatch = Regex.Match(line, "\"seq\":\\s*(\\d+)");
            if (seqMatch.Success)
                Console.WriteLine($"[{now:HH:mm:ss.fff}] [SEQ] Server received fix #{seqMatch.Groups[1].Value}");
        }
        SendToUnity(line, unityEndpoint, unitySendClient);
    }
}

// Unity's collision state, as "1" or "0". Bound to loopback only: Unity is on
// this machine, and nothing off it should be able to switch the relays.
static async Task RunRelayListener(int port, RelayState relay)
{
    var udp = new UdpClient(new IPEndPoint(IPAddress.Loopback, port));
    Console.WriteLine($"Listening for Unity's collision state on UDP port {port}...");

    while (true)
    {
        try
        {
            var result = await udp.ReceiveAsync();
            relay.Update(result.Buffer.Length == 1 && result.Buffer[0] == (byte)'1');
        }
        catch (SocketException)
        {
            // Windows reports a send to a closed port as an error on the next
            // receive. Harmless here, since the state simply goes stale.
        }
    }
}

static void SendToUnity(string message, IPEndPoint unityEndpoint, UdpClient unitySendClient)
{
    byte[] data = Encoding.UTF8.GetBytes(message);
    try
    {
        unitySendClient.Send(data, data.Length, unityEndpoint);
    }
    catch (Exception e)
    {
        Console.WriteLine($"[{DateTime.UtcNow:HH:mm:ss.fff}] Send to Unity failed: {e.Message}");
    }
}

static async Task RunWebServer(int port, string token, string indexHtml, IPEndPoint unityEndpoint, UdpClient unitySendClient, SnapshotStore snapshots, TextStore latestFix, MazeState maze, RelayState relay, string loggedPositionsPath, object loggedPositionsLock)
{
    var listener = new HttpListener();
    listener.Prefixes.Add($"http://+:{port}/");
    try
    {
        listener.Start();
    }
    catch (HttpListenerException e)
    {
        Console.WriteLine($"Web UI failed to start on port {port}: {e.Message}");
        Console.WriteLine($"On Windows, binding \"+\" (all interfaces) needs either an admin-elevated process, " +
            $"or a one-time reservation: netsh http add urlacl url=http://+:{port}/ user=Everyone");
        return;
    }
    Console.WriteLine($"Web UI listening on port {port}...");

    while (true)
    {
        var ctx = await listener.GetContextAsync();
        _ = Task.Run(() => HandleWebRequest(ctx, token, indexHtml, unityEndpoint, unitySendClient, snapshots, latestFix, maze, relay, loggedPositionsPath, loggedPositionsLock));
    }
}

static async Task HandleWebRequest(HttpListenerContext ctx, string token, string indexHtml, IPEndPoint unityEndpoint, UdpClient unitySendClient, SnapshotStore snapshots, TextStore latestFix, MazeState maze, RelayState relay, string loggedPositionsPath, object loggedPositionsLock)
{
    try
    {
        var req = ctx.Request;
        var res = ctx.Response;

        switch (req.Url?.AbsolutePath)
        {
            case "/":
                // Otherwise a phone can keep showing an old copy of the page
                // after the server has been restarted with a new one.
                res.Headers["Cache-Control"] = "no-store";
                await WriteResponse(res, 200, "text/html", Encoding.UTF8.GetBytes(indexHtml));
                break;

            // Read fresh per request rather than cached at startup like
            // indexHtml, so edits to the page show up without a restart.
            case "/position":
                string positionPath = Path.Combine(AppContext.BaseDirectory, "wwwroot", "position.html");
                if (!File.Exists(positionPath))
                {
                    await WriteResponse(res, 404, "text/plain", Encoding.UTF8.GetBytes("position.html not found"));
                    break;
                }
                await WriteResponse(res, 200, "text/html", await File.ReadAllBytesAsync(positionPath));
                break;

            case "/reset":
                if (req.HttpMethod != "POST" || req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                SendToUnity("{\"type\":\"reset\"}", unityEndpoint, unitySendClient);
                Console.WriteLine("Reset requested from web UI.");
                await WriteResponse(res, 200, "text/plain", Encoding.UTF8.GetBytes("OK"));
                break;

            case "/place-marker":
                if (req.HttpMethod != "POST" || req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                SendToUnity("{\"type\":\"place\"}", unityEndpoint, unitySendClient);
                Console.WriteLine("Place marker requested from web UI.");
                await WriteResponse(res, 200, "text/plain", Encoding.UTF8.GetBytes("OK"));
                break;

            case "/clear-markers":
                if (req.HttpMethod != "POST" || req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                SendToUnity("{\"type\":\"clear\"}", unityEndpoint, unitySendClient);
                Console.WriteLine("Clear markers requested from web UI.");
                await WriteResponse(res, 200, "text/plain", Encoding.UTF8.GetBytes("OK"));
                break;

            case "/spawn-wall":
                if (req.HttpMethod != "POST" || req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                SendToUnity("{\"type\":\"spawn-wall\"}", unityEndpoint, unitySendClient);
                Console.WriteLine("Spawn wall requested from web UI.");
                await WriteResponse(res, 200, "text/plain", Encoding.UTF8.GetBytes("OK"));
                break;

            case "/clear-walls":
                if (req.HttpMethod != "POST" || req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                SendToUnity("{\"type\":\"clear-walls\"}", unityEndpoint, unitySendClient);
                Console.WriteLine("Clear walls requested from web UI.");
                await WriteResponse(res, 200, "text/plain", Encoding.UTF8.GetBytes("OK"));
                break;

            case "/generate-maze":
                if (req.HttpMethod != "POST" || req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                // Picked here rather than in Unity so the page can show it
                // straight away, without a reply channel back from Unity.
                maze.NewSeed();
                SendToUnity($"{{\"type\":\"generate-maze\",\"seed\":{maze.Seed},\"extend\":{(maze.Extend ? 1 : 0)}}}", unityEndpoint, unitySendClient);
                Console.WriteLine($"Generate maze requested from web UI (seed {maze.Seed}).");
                await WriteResponse(res, 200, "application/json", Encoding.UTF8.GetBytes(maze.ToJson()));
                break;

            case "/clear-maze":
                if (req.HttpMethod != "POST" || req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                SendToUnity("{\"type\":\"clear-maze\"}", unityEndpoint, unitySendClient);
                Console.WriteLine("Clear maze requested from web UI.");
                await WriteResponse(res, 200, "text/plain", Encoding.UTF8.GetBytes("OK"));
                break;

            case "/maze-extend":
                if (req.HttpMethod != "POST" || req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                maze.Extend = req.QueryString["enabled"] == "1";
                SendToUnity($"{{\"type\":\"maze-extend\",\"extend\":{(maze.Extend ? 1 : 0)}}}", unityEndpoint, unitySendClient);
                Console.WriteLine($"Maze extend-at-exit {(maze.Extend ? "enabled" : "disabled")} from web UI.");
                await WriteResponse(res, 200, "application/json", Encoding.UTF8.GetBytes(maze.ToJson()));
                break;

            case "/maze.json":
                if (req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                await WriteResponse(res, 200, "application/json", Encoding.UTF8.GetBytes(maze.ToJson()));
                break;

            case "/start-recording":
                if (req.HttpMethod != "POST" || req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                SendToUnity("{\"type\":\"start-recording\"}", unityEndpoint, unitySendClient);
                Console.WriteLine("Start recording requested from web UI.");
                await WriteResponse(res, 200, "text/plain", Encoding.UTF8.GetBytes("OK"));
                break;

            case "/stop-recording":
                if (req.HttpMethod != "POST" || req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                SendToUnity("{\"type\":\"stop-recording\"}", unityEndpoint, unitySendClient);
                Console.WriteLine("Stop recording requested from web UI.");
                await WriteResponse(res, 200, "text/plain", Encoding.UTF8.GetBytes("OK"));
                break;

            case "/grid-size":
                if (req.HttpMethod != "POST" || req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                if (!int.TryParse(req.QueryString["cells"], out int cells) || cells < 1)
                {
                    await WriteResponse(res, 400, "text/plain", Encoding.UTF8.GetBytes("Invalid cells value"));
                    break;
                }
                SendToUnity($"{{\"type\":\"grid-size\",\"cells\":{cells}}}", unityEndpoint, unitySendClient);
                Console.WriteLine($"Grid size ({cells}) requested from web UI.");
                await WriteResponse(res, 200, "text/plain", Encoding.UTF8.GetBytes("OK"));
                break;

            case "/snapshot":
                if (req.HttpMethod != "POST" || req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                using (var ms = new MemoryStream())
                {
                    await req.InputStream.CopyToAsync(ms);
                    snapshots.Set(ms.ToArray());
                }
                await WriteResponse(res, 200, "text/plain", Encoding.UTF8.GetBytes("OK"));
                break;

            case "/snapshot.jpg":
                if (req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                var jpg = snapshots.Get();
                if (jpg.Length == 0)
                    await WriteResponse(res, 404, "text/plain", Encoding.UTF8.GetBytes("No snapshot yet"));
                else
                    await WriteResponse(res, 200, "image/jpeg", jpg);
                break;

            case "/snapshot.mjpeg":
                if (req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                await StreamSnapshots(res, snapshots);
                break;

            case "/relay.stream":
                if (req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                await StreamRelayState(res, relay, req.RemoteEndPoint?.ToString() ?? "unknown");
                break;

            case "/gps.json":
                if (req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                var fix = latestFix.Get();
                if (fix == null)
                    await WriteResponse(res, 404, "text/plain", Encoding.UTF8.GetBytes("No fix yet"));
                else
                    await WriteResponse(res, 200, "application/json", Encoding.UTF8.GetBytes(fix));
                break;

            case "/log-position":
                if (req.HttpMethod != "POST" || req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                var fixToLog = latestFix.Get();
                if (fixToLog == null)
                {
                    await WriteResponse(res, 404, "text/plain", Encoding.UTF8.GetBytes("No fix yet"));
                    break;
                }
                lock (loggedPositionsLock)
                {
                    File.AppendAllText(loggedPositionsPath, fixToLog + "\n");
                }
                Console.WriteLine($"Logged position from web UI: {fixToLog}");
                await WriteResponse(res, 200, "application/json", Encoding.UTF8.GetBytes(fixToLog));
                break;

            case "/clear-logged-positions":
                if (req.HttpMethod != "POST" || req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                lock (loggedPositionsLock)
                {
                    File.WriteAllText(loggedPositionsPath, string.Empty);
                }
                Console.WriteLine("Logged positions cleared from web UI.");
                await WriteResponse(res, 200, "text/plain", Encoding.UTF8.GetBytes("OK"));
                break;

            case "/logged-positions.json":
                if (req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                string[] loggedLines;
                lock (loggedPositionsLock)
                {
                    loggedLines = File.Exists(loggedPositionsPath)
                        ? File.ReadAllLines(loggedPositionsPath)
                        : Array.Empty<string>();
                }
                var json = "[" + string.Join(",", loggedLines.Where(l => l.Length > 0)) + "]";
                await WriteResponse(res, 200, "application/json", Encoding.UTF8.GetBytes(json));
                break;

            case "/gps.py":
                if (req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                // Read fresh every request (not cached at startup like indexHtml)
                // so an edited gps.py is available to curl without restarting the server.
                string scriptPath = Path.Combine(AppContext.BaseDirectory, "gps.py");
                if (!File.Exists(scriptPath))
                {
                    await WriteResponse(res, 404, "text/plain", Encoding.UTF8.GetBytes("gps.py not found on server"));
                    break;
                }
                byte[] scriptBytes = await File.ReadAllBytesAsync(scriptPath);
                await WriteResponse(res, 200, "text/x-python", scriptBytes);
                break;

            default:
                await WriteResponse(res, 404, "text/plain", Encoding.UTF8.GetBytes("Not found"));
                break;
        }
    }
    catch (Exception e)
    {
        Console.WriteLine($"Web request error: {e.Message}");
        try { ctx.Response.StatusCode = 500; ctx.Response.Close(); } catch { }
    }
}

// One HTTP response that never finishes, with each JPEG written into it as
// another MIME part (multipart/x-mixed-replace - what every IP camera serves).
// Browsers render it natively in a plain <img>, so frames appear as fast as
// Unity pushes them: no per-frame request, no connection setup per frame, and
// no polling interval putting a ceiling on the rate.
static async Task StreamSnapshots(HttpListenerResponse res, SnapshotStore snapshots)
{
    const string boundary = "frame";
    res.StatusCode = 200;
    res.ContentType = $"multipart/x-mixed-replace; boundary={boundary}";
    res.SendChunked = true;
    res.Headers.Add("Cache-Control", "no-store");

    long lastVersion = 0;
    var stream = res.OutputStream;
    var trailer = Encoding.ASCII.GetBytes("\r\n");

    // Browsers only paint a frame once the next one starts arriving, so the
    // newest frame stays unseen until another follows it. While Unity is sending
    // that's a frame of lag; when it isn't, a new viewer would stay blank with
    // the image already delivered. So a frame nothing has followed goes out once
    // more - once only, so an idle stream doesn't keep using the viewer's data.
    byte[]? latest = null;
    bool repeated = true;
    var sinceLastFrame = System.Diagnostics.Stopwatch.StartNew();

    try
    {
        while (true)
        {
            byte[]? frame = snapshots.GetNewer(ref lastVersion);
            if (frame != null)
            {
                latest = frame;
                repeated = false;
            }
            else if (!repeated && sinceLastFrame.ElapsedMilliseconds >= 200)
            {
                frame = latest;
                repeated = true;
            }

            if (frame == null)
            {
                // Nothing new yet. A short sleep rather than a signal: this is
                // a local lock check, not a round trip, so 10ms costs nothing
                // and caps the latency this loop can add at 10ms.
                await Task.Delay(10);
                continue;
            }

            sinceLastFrame.Restart();
            var header = Encoding.ASCII.GetBytes(
                $"--{boundary}\r\nContent-Type: image/jpeg\r\nContent-Length: {frame.Length}\r\n\r\n");
            await stream.WriteAsync(header);
            await stream.WriteAsync(frame);
            await stream.WriteAsync(trailer);
            await stream.FlushAsync();
        }
    }
    catch (HttpListenerException) { /* viewer closed the tab */ }
    catch (IOException) { /* connection dropped mid-write */ }
    finally
    {
        try { res.Close(); } catch { }
    }
}

// Held open by the ESP32. Sends the state the moment it changes and every 200ms
// regardless: the repeats are what let the ESP32 tell a quiet link from a dead
// one, and also make this write fail soon after the ESP32 disappears.
static async Task StreamRelayState(HttpListenerResponse res, RelayState relay, string viewer)
{
    res.StatusCode = 200;
    res.ContentType = "text/plain";
    res.SendChunked = true;
    res.Headers["Cache-Control"] = "no-store";

    var stream = res.OutputStream;
    var on = Encoding.ASCII.GetBytes("R1\n");
    var off = Encoding.ASCII.GetBytes("R0\n");
    bool? lastSent = null;
    var sinceSend = System.Diagnostics.Stopwatch.StartNew();
    Console.WriteLine($"Relay board connected from {viewer}.");

    try
    {
        while (true)
        {
            bool state = relay.IsOn;
            if (state != lastSent || sinceSend.ElapsedMilliseconds >= 200)
            {
                await stream.WriteAsync(state ? on : off);
                await stream.FlushAsync();
                if (state != lastSent && lastSent != null)
                    Console.WriteLine($"Relay {(state ? "ON" : "OFF")}.");
                lastSent = state;
                sinceSend.Restart();
            }
            await Task.Delay(10);
        }
    }
    catch (HttpListenerException) { /* board went away */ }
    catch (IOException) { /* connection dropped mid-write */ }
    finally
    {
        Console.WriteLine($"Relay board disconnected ({viewer}).");
        try { res.Close(); } catch { }
    }
}

static async Task WriteResponse(HttpListenerResponse res, int statusCode, string contentType, byte[] body)
{
    res.StatusCode = statusCode;
    res.ContentType = contentType;
    res.ContentLength64 = body.Length;
    await res.OutputStream.WriteAsync(body);
    res.OutputStream.Close();
}

class SnapshotStore
{
    private readonly object _lock = new();
    private byte[] _data = Array.Empty<byte>();
    private long _version;

    public void Set(byte[] data) { lock (_lock) { _data = data; _version++; } }
    public byte[] Get() { lock (_lock) return _data; }

    // Hands back a frame only when it's newer than the one the caller last
    // saw, so an MJPEG stream sends each frame exactly once rather than
    // re-sending whatever is current on a timer.
    public byte[]? GetNewer(ref long lastVersion)
    {
        lock (_lock)
        {
            if (_version == lastVersion || _data.Length == 0) return null;
            lastVersion = _version;
            return _data;
        }
    }
}

// What the web page's maze controls last asked for, so a reloaded or second
// page shows the current seed and checkbox instead of starting blank.
class MazeState
{
    private volatile int _seed;
    private volatile bool _extend;

    public int Seed => _seed;
    public bool Extend { get => _extend; set => _extend = value; }

    // 0 means "none yet", so it's left out of the range. Six digits at most so
    // it's easy to read off a phone and type into Unity's Seed field.
    public void NewSeed() => _seed = Random.Shared.Next(1, 1_000_000);

    public string ToJson() =>
        $"{{\"seed\":{(_seed == 0 ? "null" : _seed.ToString())},\"extend\":{(_extend ? "true" : "false")}}}";
}

// Whether the relays should be on, as last reported by Unity. Goes off by itself
// once Unity stops reporting, so stopping Play mode or a crash can't leave them on.
class RelayState
{
    private const long StaleAfterMs = 500;

    private volatile bool _on;
    private long _updatedAt;

    public void Update(bool on)
    {
        _on = on;
        Interlocked.Exchange(ref _updatedAt, Environment.TickCount64);
    }

    public bool IsOn => _on && Environment.TickCount64 - Interlocked.Read(ref _updatedAt) < StaleAfterMs;
}

class TextStore
{
    private readonly object _lock = new();
    private string? _data;
    public void Set(string data) { lock (_lock) _data = data; }
    public string? Get() { lock (_lock) return _data; }
}
