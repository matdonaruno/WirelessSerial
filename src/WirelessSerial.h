#ifndef WIRELESS_SERIAL_H
#define WIRELESS_SERIAL_H

#include <Arduino.h>
#include <Print.h>

#ifdef ESP8266
  #include <ESP8266WiFi.h>
  #include <ESP8266mDNS.h>
#elif defined(ESP32)
  #include <WiFi.h>
  #include <ESPmDNS.h>
#else
  #error "WirelessSerial only supports ESP8266 and ESP32"
#endif

#include "DualPrint.h"

#define WIRELESS_SERIAL_VERSION "1.0.0"
#define WIRELESS_SERIAL_DEFAULT_PORT 23
#define WIRELESS_SERIAL_DEFAULT_MDNS "esp-serial"
#define WIRELESS_SERIAL_MAX_CLIENTS_LIMIT 5
#define WIRELESS_SERIAL_DEFAULT_MAX_CLIENTS 3

class WirelessSerial : public Print {
public:
    WirelessSerial();
    ~WirelessSerial();

    // ========== Lifecycle ==========

    /// Start TCP server and mDNS service.
    /// @param port TCP port number (default: 23)
    /// @param mdnsName mDNS service name (default: "esp-serial")
    void begin(uint16_t port = WIRELESS_SERIAL_DEFAULT_PORT,
               const char* mdnsName = WIRELESS_SERIAL_DEFAULT_MDNS);

    /// Stop server and disconnect all clients.
    void stop();

    /// Handle client connections. Call in loop().
    void handle();

    // ========== Mirror Mode ==========

    /// Mirror Serial output to TCP clients.
    /// After calling this, use the returned DualPrint* for output
    /// to send to both Serial and TCP simultaneously.
    /// @param serial The Stream to mirror (typically Serial)
    /// @return Pointer to DualPrint instance (owned by WirelessSerial)
    DualPrint* mirror(Print& serial);

    /// Disable mirroring.
    void unmirror();

    /// Check if mirroring is active.
    bool isMirroring() const;

    /// Get the DualPrint instance (nullptr if not mirroring).
    DualPrint* getDualPrint() const;

    // ========== Direct Output (Print interface) ==========

    size_t write(uint8_t byte) override;
    size_t write(const uint8_t* buffer, size_t size) override;

    // ========== Status ==========

    /// Number of connected clients.
    uint8_t clientCount() const;

    /// Whether the server is running.
    bool isRunning() const;

    // ========== Configuration ==========

    /// Set maximum number of clients (default: 3, max: 5).
    void setMaxClients(uint8_t maxClients);

private:
    WiFiServer* _server;
    WiFiClient _clients[WIRELESS_SERIAL_MAX_CLIENTS_LIMIT];
    uint8_t _maxClients;
    bool _running;
    uint16_t _port;

    DualPrint* _dualPrint;
    bool _mirroring;

    void _acceptNewClients();
    void _cleanupClients();
};

#endif // WIRELESS_SERIAL_H
