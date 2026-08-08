using System;
using System.Collections.Concurrent;
using System.IO;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

// Matches the JSON line shape sent by PiScript/gps.py's ServerRelay.
[Serializable]
public struct GpsFix
{
    public double lat;
    public double lon;
    public double alt;
    public int fixQuality;
    public string fixStatus;
    public double timestamp;
}

// Attach to any GameObject. Connects to the C# server running on the same PC
// and receives whatever data the server relayed from the Raspberry Pi.
public class RaspberryPiDataReceiver : MonoBehaviour
{
    [SerializeField] private string serverHost = "127.0.0.1";
    [SerializeField] private int serverPort = 5001;

    private TcpClient _client;
    private Thread _readThread;
    private volatile bool _running;
    private readonly ConcurrentQueue<string> _incoming = new ConcurrentQueue<string>();

    // Fired on the main thread with each raw line of data received.
    public event Action<string> OnDataReceived;

    // Fired on the main thread when a line successfully parses as a GpsFix.
    public event Action<GpsFix> OnGpsFixReceived;

    private void Start()
    {
        _running = true;
        _readThread = new Thread(ReadLoop) { IsBackground = true };
        _readThread.Start();
    }

    private void ReadLoop()
    {
        while (_running)
        {
            try
            {
                _client = new TcpClient();
                _client.Connect(serverHost, serverPort);

                using (var stream = _client.GetStream())
                using (var reader = new StreamReader(stream))
                {
                    string line;
                    while (_running && (line = reader.ReadLine()) != null)
                    {
                        _incoming.Enqueue(line);
                    }
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning($"Server connection lost/unavailable: {e.Message}. Retrying in 2s...");
            }

            if (_running)
                Thread.Sleep(2000); // retry connection
        }
    }

    private void Update()
    {
        while (_incoming.TryDequeue(out var line))
        {
            OnDataReceived?.Invoke(line);

            try
            {
                var fix = JsonUtility.FromJson<GpsFix>(line);
                OnGpsFixReceived?.Invoke(fix);
            }
            catch (ArgumentException)
            {
                // Not a GpsFix-shaped JSON line; subscribers to OnDataReceived
                // still got the raw text above.
            }
        }
    }

    private void OnDestroy()
    {
        _running = false;
        _client?.Close();
        _readThread?.Join(500);
    }
}
