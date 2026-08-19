using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Text;

const int piPort = 5002;
const int unityPort = 5001;
const int webPort = 5003;

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

var unityClients = new List<TcpClient>();
var unityClientsLock = new object();
var snapshots = new SnapshotStore();
var latestFix = new TextStore();

_ = Task.Run(() => RunUnityListener(unityPort, unityClients, unityClientsLock));
_ = Task.Run(() => RunWebServer(webPort, authToken, indexHtml, unityClients, unityClientsLock, snapshots, latestFix, loggedPositionsPath, loggedPositionsLock));
await RunPiListener(piPort, unityClients, unityClientsLock, latestFix, gpsLogPath, gpsLogLock);

static async Task RunPiListener(int port, List<TcpClient> unityClients, object unityClientsLock, TextStore latestFix, string gpsLogPath, object gpsLogLock)
{
    var listener = new TcpListener(IPAddress.Any, port);
    listener.Start();
    Console.WriteLine($"Listening for Raspberry Pi on port {port}...");

    while (true)
    {
        var client = await listener.AcceptTcpClientAsync();
        Console.WriteLine($"Pi connected from {client.Client.RemoteEndPoint}");
        _ = Task.Run(() => HandlePiClient(client, unityClients, unityClientsLock, latestFix, gpsLogPath, gpsLogLock));
    }
}

static async Task HandlePiClient(TcpClient client, List<TcpClient> unityClients, object unityClientsLock, TextStore latestFix, string gpsLogPath, object gpsLogLock)
{
    using (client)
    using (var stream = client.GetStream())
    using (var reader = new StreamReader(stream, Encoding.UTF8))
    {
        try
        {
            string? line;
            DateTime? lastLineAt = null;
            while ((line = await reader.ReadLineAsync()) != null)
            {
                var now = DateTime.UtcNow;
                // GPS fixes arrive ~1Hz, IMU messages ~10Hz - a gap much bigger
                // than that (from either) means something upstream (Pi relay,
                // this listener, or a stalled broadcast to Unity - see
                // BroadcastToUnity below) is stalling, not just normal spacing.
                if (lastLineAt.HasValue)
                {
                    double gapMs = (now - lastLineAt.Value).TotalMilliseconds;
                    if (gapMs > 1500)
                        Console.WriteLine($"[{now:HH:mm:ss.fff}] [PERF] {gapMs:F0}ms gap since previous Pi message");
                }
                lastLineAt = now;

                Console.WriteLine($"[{now:HH:mm:ss.fff}] Received from Pi: {line}");
                // Control/IMU messages (e.g. {"type":"imu",...} at ~10Hz) share
                // this same relay connection but aren't real GPS fixes - only
                // cache/log the real ones, or the mobile page's readout would
                // mostly be reading stale/incomplete IMU data instead.
                if (!line.Contains("\"type\":"))
                {
                    latestFix.Set(line);
                    lock (gpsLogLock)
                    {
                        File.AppendAllText(gpsLogPath, line + "\n");
                    }
                }
                BroadcastToUnity(line, unityClients, unityClientsLock);
            }
        }
        catch (IOException)
        {
            // Pi dropped the connection abruptly; fall through to cleanup below.
        }
    }
    Console.WriteLine("Pi disconnected.");
}

static void BroadcastToUnity(string message, List<TcpClient> unityClients, object unityClientsLock)
{
    byte[] data = Encoding.UTF8.GetBytes(message + "\n");
    lock (unityClientsLock)
    {
        for (int i = unityClients.Count - 1; i >= 0; i--)
        {
            var uc = unityClients[i];
            // These sockets have no send timeout configured, so a stalled
            // Unity client (GC pause, editor hiccup, half-dead connection)
            // could otherwise block this Write() indefinitely - and since
            // this runs under unityClientsLock, that would stall every other
            // message too (GPS fixes AND the 10Hz IMU stream). Timing every
            // write here tells us whether that's actually happening.
            var sw = Stopwatch.StartNew();
            try
            {
                uc.GetStream().Write(data, 0, data.Length);
                sw.Stop();
                if (sw.ElapsedMilliseconds > 50)
                    Console.WriteLine($"[{DateTime.UtcNow:HH:mm:ss.fff}] [PERF] Slow write to Unity client {uc.Client.RemoteEndPoint} took {sw.ElapsedMilliseconds}ms");
            }
            catch (Exception e)
            {
                Console.WriteLine($"[{DateTime.UtcNow:HH:mm:ss.fff}] [PERF] Unity client {uc.Client.RemoteEndPoint} write failed after {sw.ElapsedMilliseconds}ms ({e.Message}) - removing. {unityClients.Count - 1} client(s) left.");
                unityClients.RemoveAt(i);
            }
        }
    }
}

static async Task RunUnityListener(int port, List<TcpClient> unityClients, object unityClientsLock)
{
    var listener = new TcpListener(IPAddress.Any, port);
    listener.Start();
    Console.WriteLine($"Listening for Unity on port {port}...");

    while (true)
    {
        var client = await listener.AcceptTcpClientAsync();
        Console.WriteLine("Unity connected.");
        lock (unityClientsLock)
        {
            unityClients.Add(client);
        }
    }
}

static async Task RunWebServer(int port, string token, string indexHtml, List<TcpClient> unityClients, object unityClientsLock, SnapshotStore snapshots, TextStore latestFix, string loggedPositionsPath, object loggedPositionsLock)
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
        _ = Task.Run(() => HandleWebRequest(ctx, token, indexHtml, unityClients, unityClientsLock, snapshots, latestFix, loggedPositionsPath, loggedPositionsLock));
    }
}

static async Task HandleWebRequest(HttpListenerContext ctx, string token, string indexHtml, List<TcpClient> unityClients, object unityClientsLock, SnapshotStore snapshots, TextStore latestFix, string loggedPositionsPath, object loggedPositionsLock)
{
    try
    {
        var req = ctx.Request;
        var res = ctx.Response;

        switch (req.Url?.AbsolutePath)
        {
            case "/":
                await WriteResponse(res, 200, "text/html", Encoding.UTF8.GetBytes(indexHtml));
                break;

            case "/reset":
                if (req.HttpMethod != "POST" || req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                BroadcastToUnity("{\"type\":\"reset\"}", unityClients, unityClientsLock);
                Console.WriteLine("Reset requested from web UI.");
                await WriteResponse(res, 200, "text/plain", Encoding.UTF8.GetBytes("OK"));
                break;

            case "/place-marker":
                if (req.HttpMethod != "POST" || req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                BroadcastToUnity("{\"type\":\"place\"}", unityClients, unityClientsLock);
                Console.WriteLine("Place marker requested from web UI.");
                await WriteResponse(res, 200, "text/plain", Encoding.UTF8.GetBytes("OK"));
                break;

            case "/clear-markers":
                if (req.HttpMethod != "POST" || req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                BroadcastToUnity("{\"type\":\"clear\"}", unityClients, unityClientsLock);
                Console.WriteLine("Clear markers requested from web UI.");
                await WriteResponse(res, 200, "text/plain", Encoding.UTF8.GetBytes("OK"));
                break;

            case "/start-recording":
                if (req.HttpMethod != "POST" || req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                BroadcastToUnity("{\"type\":\"start-recording\"}", unityClients, unityClientsLock);
                Console.WriteLine("Start recording requested from web UI.");
                await WriteResponse(res, 200, "text/plain", Encoding.UTF8.GetBytes("OK"));
                break;

            case "/stop-recording":
                if (req.HttpMethod != "POST" || req.QueryString["token"] != token)
                {
                    await WriteResponse(res, 403, "text/plain", Encoding.UTF8.GetBytes("Forbidden"));
                    break;
                }
                BroadcastToUnity("{\"type\":\"stop-recording\"}", unityClients, unityClientsLock);
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
                BroadcastToUnity($"{{\"type\":\"grid-size\",\"cells\":{cells}}}", unityClients, unityClientsLock);
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
    public void Set(byte[] data) { lock (_lock) _data = data; }
    public byte[] Get() { lock (_lock) return _data; }
}

class TextStore
{
    private readonly object _lock = new();
    private string? _data;
    public void Set(string data) { lock (_lock) _data = data; }
    public string? Get() { lock (_lock) return _data; }
}
