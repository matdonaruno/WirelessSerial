/**
 * ESP32BothModes.ino - Serial Air WirelessSerial Example
 *
 * ESP32 AP+STA simultaneous mode: connects to your router AND
 * creates its own AP. Accessible from either network.
 *
 * NOTE: This example is ESP32-only (ESP8266 has limited AP+STA support).
 */

#ifdef ESP32
#include <WirelessSerial.h>

// ========== WiFi Credentials (STA) ==========
const char* staSSID = "YOUR_WIFI_SSID";
const char* staPassword = "YOUR_WIFI_PASSWORD";

// ========== AP Configuration ==========
const char* apSSID = "SerialAir-ESP32";
const char* apPassword = "serialair";

// ========== WirelessSerial ==========
WirelessSerial ws;
DualPrint* output;

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("=== Serial Air - ESP32 AP+STA Example ===");

    // Enable AP+STA mode
    WiFi.mode(WIFI_AP_STA);

    // Start AP
    WiFi.softAP(apSSID, apPassword);
    Serial.print("AP started: ");
    Serial.println(apSSID);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    // Connect to router
    WiFi.begin(staSSID, staPassword);
    Serial.print("Connecting to ");
    Serial.print(staSSID);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("STA IP: ");
    Serial.println(WiFi.localIP());

    // Start WirelessSerial
    ws.begin();
    output = ws.mirror(Serial);

    output->println("ESP32 AP+STA mode ready!");
    output->print("AP: ");
    output->print(WiFi.softAPIP());
    output->println(":23");
    output->print("STA: ");
    output->print(WiFi.localIP());
    output->println(":23");
}

void loop() {
    ws.handle();

    output->print("STA connected: ");
    output->println(WiFi.isConnected() ? "Yes" : "No");

    output->print("STA IP: ");
    output->println(WiFi.localIP());

    output->print("AP clients: ");
    output->println(WiFi.softAPgetStationNum());

    output->print("TCP clients: ");
    output->println(ws.clientCount());

    output->print("Free heap: ");
    output->print(ESP.getFreeHeap());
    output->println(" bytes");

    output->println("---");

    delay(2000);
}

#else
#error "This example requires ESP32. Use BasicMirror or APModeMirror for ESP8266."
#endif
