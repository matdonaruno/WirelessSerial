# WirelessSerial

Wireless serial monitor over WiFi TCP for ESP8266 and ESP32.

Part of the **Serial Air** system — pairs with the [Serial Air iOS app](https://github.com/matdonaruno/serial-air).

## Quick Start

```cpp
#include <WirelessSerial.h>

WirelessSerial ws;
DualPrint* output;

void setup() {
    Serial.begin(115200);
    WiFi.begin("SSID", "PASSWORD");
    while (WiFi.status() != WL_CONNECTED) delay(500);

    ws.begin();
    ws.enableBuffer();  // Buffer output when no client connected
    output = ws.mirror(Serial);
    output->println("Hello from Serial Air!");
}

void loop() {
    ws.handle();
    output->println("Hello!");
    delay(1000);
}
```

## Features

- TCP server on configurable port (default: 23)
- mDNS auto-discovery (compatible with Serial Air iOS app)
- Mirror mode: output to Serial USB and TCP simultaneously
- **Ring buffer**: retains output when no client is connected, flushes on connect
- Up to 5 concurrent TCP clients
- Minimal memory footprint (< 4KB RAM with default 2KB buffer)

## API

| Method | Description |
|--------|-------------|
| `begin(port, mdnsName)` | Start TCP server and mDNS |
| `stop()` | Stop server |
| `handle()` | Process connections (call in loop) |
| `mirror(serial)` | Enable mirror mode, returns DualPrint* |
| `unmirror()` | Disable mirror mode |
| `write(data)` | Send data to TCP clients (or buffer if none connected) |
| `clientCount()` | Number of connected clients |
| `isRunning()` | Server status |
| `setMaxClients(n)` | Set max clients (1-5) |
| `enableBuffer(size)` | Enable ring buffer (default: 2048 bytes) |
| `disableBuffer()` | Disable and free ring buffer |
| `bufferedBytes()` | Bytes currently in buffer |

## Ring Buffer

When enabled, output written while no clients are connected is stored in a ring buffer. When a client connects, buffered data is automatically sent.

```cpp
ws.enableBuffer();       // 2KB default
ws.enableBuffer(4096);   // custom size
ws.disableBuffer();      // disable and free memory
```

This is useful for capturing boot messages and early serial output before the app connects.

## Examples

- **BasicMirror** — STA mode, simplest usage
- **APModeMirror** — AP mode, no router needed
- **ESP32BothModes** — AP+STA simultaneous (ESP32 only)
- **CustomPort** — Custom port and mDNS name

## Installation

### Arduino IDE
1. Download this repository as ZIP
2. Arduino IDE → Sketch → Include Library → Add .ZIP Library
3. Select the downloaded ZIP file

### Manual
Copy this folder to your Arduino libraries directory:
- macOS: `~/Documents/Arduino/libraries/WirelessSerial/`
- Windows: `Documents\Arduino\libraries\WirelessSerial\`
- Linux: `~/Arduino/libraries/WirelessSerial/`

## Supported Platforms

- ESP8266 (Arduino Core)
- ESP32 (Arduino Core)

## License

MIT
