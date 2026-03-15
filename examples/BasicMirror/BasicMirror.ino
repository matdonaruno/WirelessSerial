/**
 * BasicMirror.ino - Serial Air WirelessSerial Example
 *
 * WiFi STA mode: connects to your router, mirrors Serial output to TCP.
 * The simplest usage of WirelessSerial.
 *
 * 1. Set your WiFi credentials below
 * 2. Upload to ESP8266 or ESP32
 * 3. Open Serial Air app on iPhone → device auto-discovered via mDNS
 */

#include <WirelessSerial.h>

// ========== WiFi Credentials ==========
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// ========== WirelessSerial ==========
WirelessSerial ws;
DualPrint* output;  // Sends to both Serial and TCP

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("=== Serial Air - BasicMirror Example ===");

    // Connect to WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("Connected! IP: ");
    Serial.println(WiFi.localIP());

    // Start WirelessSerial (TCP server + mDNS)
    ws.begin();

    // Enable mirror mode: output goes to both Serial and TCP
    output = ws.mirror(Serial);

    output->println("WirelessSerial started!");
    output->print("Clients can connect to: ");
    output->print(WiFi.localIP());
    output->println(":23");
}

void loop() {
    ws.handle();  // Must be called every loop iteration

    // All output via 'output' goes to both Serial USB and TCP clients
    output->print("Uptime: ");
    output->print(millis() / 1000);
    output->println("s");

    output->print("Free heap: ");
    output->print(ESP.getFreeHeap());
    output->println(" bytes");

    output->print("TCP clients: ");
    output->println(ws.clientCount());

    output->println("---");

    delay(2000);
}
