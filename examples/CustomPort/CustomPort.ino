/**
 * CustomPort.ino - Serial Air WirelessSerial Example
 *
 * Custom TCP port and mDNS name.
 * Useful when running multiple devices or avoiding port conflicts.
 */

#include <WirelessSerial.h>

// ========== WiFi Credentials ==========
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// ========== Custom Settings ==========
const uint16_t CUSTOM_PORT = 8023;
const char* CUSTOM_MDNS_NAME = "my-sensor";
const uint8_t MAX_CLIENTS = 2;

// ========== WirelessSerial ==========
WirelessSerial ws;
DualPrint* output;

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("=== Serial Air - Custom Port Example ===");

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

    // Configure before begin()
    ws.setMaxClients(MAX_CLIENTS);

    // Start with custom port and mDNS name
    ws.begin(CUSTOM_PORT, CUSTOM_MDNS_NAME);
    output = ws.mirror(Serial);

    output->println("WirelessSerial started with custom settings!");
    output->print("mDNS: ");
    output->print(CUSTOM_MDNS_NAME);
    output->println(".local");
    output->print("Port: ");
    output->println(CUSTOM_PORT);
    output->print("Max clients: ");
    output->println(MAX_CLIENTS);
}

void loop() {
    ws.handle();

    // Direct write to TCP only (not mirrored to Serial)
    ws.println("[TCP-only] This line only goes to TCP clients");

    // Mirrored output (goes to both Serial and TCP)
    output->print("Uptime: ");
    output->print(millis() / 1000);
    output->println("s");

    output->print("Clients: ");
    output->print(ws.clientCount());
    output->print("/");
    output->println(MAX_CLIENTS);

    output->println("---");

    delay(2000);
}
