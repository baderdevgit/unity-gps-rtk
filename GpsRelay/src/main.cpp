#include <WiFi.h>
#include "secrets.h"

const int RELAY_PINS[] = {26, 27};
const int RELAY_COUNT = sizeof(RELAY_PINS) / sizeof(RELAY_PINS[0]);

// Most relay boards switch on when the input is pulled LOW. If yours clicks on
// at the wrong times (on when it should be off), set this to false.
const bool RELAY_ACTIVE_LOW = true;

const unsigned long RELAY_TIMEOUT_MS = 1000;
const unsigned long RECONNECT_DELAY_MS = 500;

WiFiClient client;
bool relaysOn = false;
bool inBody = false;       // past the HTTP response headers
int headerMatch = 0;       // how much of the blank line ending the headers has been seen
bool sawR = false;         // the previous body byte was the 'R' of a state message
unsigned long lastStateAt = 0;
unsigned long lastConnectAt = 0;

// Logging only
const unsigned long LOG_INTERVAL_MS = 1000;
const unsigned long WIFI_LOG_INTERVAL_MS = 2000;
char statusLine[64];
int statusLength = 0;
bool statusLogged = false;
bool wifiWasUp = false;
bool serverWasUp = false;
unsigned long messagesSinceLog = 0;
unsigned long lastLogAt = 0;
unsigned long lastWifiLogAt = 0;

const char* wifiStatusName(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS: return "idle";
    case WL_NO_SSID_AVAIL: return "network not found (it must be 2.4GHz)";
    case WL_CONNECT_FAILED: return "connect failed (check the password)";
    case WL_CONNECTION_LOST: return "connection lost";
    case WL_DISCONNECTED: return "disconnected";
    default: return "other";
  }
}

void writeRelays(bool on) {
  int level = (on == RELAY_ACTIVE_LOW) ? LOW : HIGH;
  for (int i = 0; i < RELAY_COUNT; i++)
    digitalWrite(RELAY_PINS[i], level);
}

void setRelays(bool on) {
  if (on == relaysOn)
    return;
  relaysOn = on;
  writeRelays(on);
  Serial.printf("[%lu] >>> Relays %s\n", millis(), on ? "ON" : "OFF");
}

void connectToServer() {
  Serial.printf("Connecting to %s:%d...\n", SERVER_HOST, SERVER_PORT);
  if (!client.connect(SERVER_HOST, SERVER_PORT, 2000)) {
    Serial.println("Connection failed.");
    return;
  }

  client.printf("GET /relay.stream?token=%s HTTP/1.1\r\nHost: %s\r\nConnection: keep-alive\r\n\r\n",
                AUTH_TOKEN, SERVER_HOST);

  inBody = false;
  headerMatch = 0;
  sawR = false;
  statusLength = 0;
  statusLogged = false;
  serverWasUp = true;
  messagesSinceLog = 0;
  lastLogAt = millis();
  lastStateAt = millis();
  Serial.println("Connected, waiting for the server's response...");
}

// The first line of the response: "200 OK" is good, "403" means a wrong AUTH_TOKEN.
void logStatusLine(char c) {
  if (statusLogged)
    return;
  if (c == '\r' || c == '\n' || statusLength == sizeof(statusLine) - 1) {
    statusLine[statusLength] = '\0';
    Serial.printf("Server replied: %s\n", statusLine);
    statusLogged = true;
    return;
  }
  statusLine[statusLength++] = c;
}

void logStateOncePerSecond() {
  if (millis() - lastLogAt < LOG_INTERVAL_MS)
    return;
  lastLogAt = millis();
  Serial.printf("[%lu] Relays %s, %lu messages in the last second\n",
                millis(), relaysOn ? "ON" : "off", messagesSinceLog);
  messagesSinceLog = 0;
}

// Only the sequence "R0" or "R1" means anything in the body. The response is
// chunked, so chunk sizes arrive mixed in with the messages, but those are hex
// digits and can't contain an R.
void readByte(char c) {
  if (!inBody) {
    logStatusLine(c);
    const char* blankLine = "\r\n\r\n";
    headerMatch = (c == blankLine[headerMatch]) ? headerMatch + 1 : (c == '\r' ? 1 : 0);
    if (headerMatch == 4)
      inBody = true;
    return;
  }

  if (sawR && (c == '0' || c == '1')) {
    setRelays(c == '1');
    lastStateAt = millis();
    messagesSinceLog++;
  }
  sawR = (c == 'R');
}

void setup() {
  // Set the output level BEFORE making the pin an output, so the relays can't
  // click on for a moment during boot.
  relaysOn = false;
  writeRelays(false);
  for (int i = 0; i < RELAY_COUNT; i++)
    pinMode(RELAY_PINS[i], OUTPUT);

  Serial.begin(115200);
  Serial.printf("\nRelay board starting, relays on GPIO %d and %d\n", RELAY_PINS[0], RELAY_PINS[1]);
  Serial.printf("Joining WiFi network \"%s\"...\n", WIFI_SSID);

  WiFi.mode(WIFI_STA);
  // Modem sleep would otherwise delay every incoming message by up to ~100ms+.
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void loop() {
  wl_status_t wifiStatus = WiFi.status();
  if (wifiStatus != WL_CONNECTED) {
    if (wifiWasUp || millis() - lastWifiLogAt >= WIFI_LOG_INTERVAL_MS) {
      lastWifiLogAt = millis();
      Serial.printf("[%lu] No WiFi: %s\n", millis(), wifiStatusName(wifiStatus));
    }
    wifiWasUp = false;
    serverWasUp = false;
    setRelays(false);
    client.stop();
    delay(50);
    return;
  }

  if (!wifiWasUp) {
    wifiWasUp = true;
    Serial.printf("WiFi connected, IP %s, signal %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
  }

  if (!client.connected()) {
    if (serverWasUp) {
      Serial.printf("[%lu] Server closed the connection.\n", millis());
      serverWasUp = false;
    }
    setRelays(false);
    client.stop();
    if (millis() - lastConnectAt >= RECONNECT_DELAY_MS) {
      lastConnectAt = millis();
      connectToServer();
    }
    return;
  }

  while (client.available())
    readByte((char)client.read());

  logStateOncePerSecond();

  if (millis() - lastStateAt > RELAY_TIMEOUT_MS) {
    Serial.printf("[%lu] No state from server for %lums - relays off, reconnecting.\n",
                  millis(), RELAY_TIMEOUT_MS);
    serverWasUp = false;
    setRelays(false);
    client.stop();
  }
}
