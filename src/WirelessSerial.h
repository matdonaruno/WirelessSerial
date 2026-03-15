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

#define WIRELESS_SERIAL_VERSION "1.1.0"
#define WIRELESS_SERIAL_DEFAULT_PORT 23
#define WIRELESS_SERIAL_DEFAULT_MDNS "esp-serial"
#define WIRELESS_SERIAL_MAX_CLIENTS_LIMIT 5
#define WIRELESS_SERIAL_DEFAULT_MAX_CLIENTS 3
#define WIRELESS_SERIAL_DEFAULT_BUFFER_SIZE 2048

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

    /// Enable ring buffer for offline data retention.
    /// When no clients are connected, write() stores data in the buffer.
    /// When a client connects, buffered data is flushed automatically.
    /// @param size Buffer size in bytes (default: 2048). 0 to disable.
    void enableBuffer(size_t size = WIRELESS_SERIAL_DEFAULT_BUFFER_SIZE);

    /// Disable and free the ring buffer.
    void disableBuffer();

    /// Get number of bytes currently in the buffer.
    size_t bufferedBytes() const;

private:
    WiFiServer* _server;
    WiFiClient _clients[WIRELESS_SERIAL_MAX_CLIENTS_LIMIT];
    uint8_t _maxClients;
    bool _running;
    uint16_t _port;

    DualPrint* _dualPrint;
    bool _mirroring;

    // Ring buffer
    uint8_t* _ringBuf;
    size_t _ringSize;
    size_t _ringHead;  // next write position
    size_t _ringCount; // bytes stored

    void _acceptNewClients();
    void _cleanupClients();
    void _bufferWrite(const uint8_t* data, size_t len);
    void _flushBufferTo(WiFiClient& client);
};

#endif // WIRELESS_SERIAL_H
