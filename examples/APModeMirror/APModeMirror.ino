/**
 * APModeMirror.ino - Serial Air WirelessSerial Example
 *
 * WiFi AP mode: ESP creates its own network, no router needed.
 * Connect your iPhone to the ESP's WiFi, then use Serial Air app.
 *
 * Useful for:
 * - Initial device setup (no WiFi configured yet)
 * - Field debugging without existing WiFi infrastructure
 * - Captive portal token verification
 */

#include <WirelessSerial.h>

// ========== AP Configuration ==========
const char* apSSID = "SerialAir-Device";
const char* apPassword = "serialair";  // Min 8 chars, or "" for open network

// ========== WirelessSerial ==========
WirelessSerial ws;
DualPrint* output;

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("=== Serial Air - AP Mode Example ===");

    // Start AP
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apSSID, apPassword);

    Serial.print("AP started: ");
    Serial.println(apSSID);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    // Start WirelessSerial
    ws.begin();
    output = ws.mirror(Serial);

    output->println("WirelessSerial AP mode ready!");
    output->print("Connect to WiFi: ");
    output->println(apSSID);
    output->print("Then open Serial Air or telnet to ");
    output->print(WiFi.softAPIP());
    output->println(":23");
}

void loop() {
    ws.handle();

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
