using System.Net;
using System.Net.Sockets;
using System.Text;

const int piPort = 5002;
const int unityPort = 5001;

var unityClients = new List<TcpClient>();
var unityClientsLock = new object();

_ = Task.Run(() => RunUnityListener(unityPort, unityClients, unityClientsLock));
await RunPiListener(piPort, unityClients, unityClientsLock);

static async Task RunPiListener(int port, List<TcpClient> unityClients, object unityClientsLock)
{
    var listener = new TcpListener(IPAddress.Any, port);
    listener.Start();
    Console.WriteLine($"Listening for Raspberry Pi on port {port}...");

    while (true)
    {
        var client = await listener.AcceptTcpClientAsync();
        Console.WriteLine($"Pi connected from {client.Client.RemoteEndPoint}");
        _ = Task.Run(() => HandlePiClient(client, unityClients, unityClientsLock));
    }
}

static async Task HandlePiClient(TcpClient client, List<TcpClient> unityClients, object unityClientsLock)
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
