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
- Up to 5 concurrent TCP clients
- Minimal memory footprint (< 2KB RAM)

## API

| Method | Description |
|--------|-------------|
| `begin(port, mdnsName)` | Start TCP server and mDNS |
| `stop()` | Stop server |
| `handle()` | Process connections (call in loop) |
| `mirror(serial)` | Enable mirror mode, returns DualPrint* |
| `unmirror()` | Disable mirror mode |
| `write(data)` | Send data to TCP clients only |
| `clientCount()` | Number of connected clients |
| `isRunning()` | Server status |
| `setMaxClients(n)` | Set max clients (1-5) |

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
