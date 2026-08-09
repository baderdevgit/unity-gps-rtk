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

var unityClients = new List<TcpClient>();
var unityClientsLock = new object();
var snapshots = new SnapshotStore();
var latestFix = new TextStore();

_ = Task.Run(() => RunUnityListener(unityPort, unityClients, unityClientsLock));
_ = Task.Run(() => RunWebServer(webPort, authToken, indexHtml, unityClients, unityClientsLock, snapshots, latestFix));
await RunPiListener(piPort, unityClients, unityClientsLock, latestFix);

static async Task RunPiListener(int port, List<TcpClient> unityClients, object unityClientsLock, TextStore latestFix)
{
    var listener = new TcpListener(IPAddress.Any, port);
    listener.Start();
    Console.WriteLine($"Listening for Raspberry Pi on port {port}...");

    while (true)
    {
        var client = await listener.AcceptTcpClientAsync();
        Console.WriteLine($"Pi connected from {client.Client.RemoteEndPoint}");
        _ = Task.Run(() => HandlePiClient(client, unityClients, unityClientsLock, latestFix));
    }
}

static async Task HandlePiClient(TcpClient client, List<TcpClient> unityClients, object unityClientsLock, TextStore latestFix)
{
    using (client)
    using (var stream = client.GetStream())
    using (var reader = new StreamReader(stream, Encoding.UTF8))
    {
        try
        {
            string? line;
            while ((line = await reader.ReadLineAsync()) != null)
            {
                Console.WriteLine($"Received from Pi: {line}");
                latestFix.Set(line);
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
            try
            {
                uc.GetStream().Write(data, 0, data.Length);
            }
            catch
            {
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

static async Task RunWebServer(int port, string token, string indexHtml, List<TcpClient> unityClients, object unityClientsLock, SnapshotStore snapshots, TextStore latestFix)
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
        _ = Task.Run(() => HandleWebRequest(ctx, token, indexHtml, unityClients, unityClientsLock, snapshots, latestFix));
    }
}

static async Task HandleWebRequest(HttpListenerContext ctx, string token, string indexHtml, List<TcpClient> unityClients, object unityClientsLock, SnapshotStore snapshots, TextStore latestFix)
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
