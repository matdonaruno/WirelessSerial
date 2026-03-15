#include "WirelessSerial.h"

WirelessSerial::WirelessSerial()
    : _server(nullptr)
    , _maxClients(WIRELESS_SERIAL_DEFAULT_MAX_CLIENTS)
    , _running(false)
    , _port(WIRELESS_SERIAL_DEFAULT_PORT)
    , _dualPrint(nullptr)
    , _mirroring(false)
{
}

WirelessSerial::~WirelessSerial() {
    stop();
}

void WirelessSerial::begin(uint16_t port, const char* mdnsName) {
    if (_running) {
        stop();
    }

    _port = port;
    _server = new WiFiServer(port);
    _server->begin();

#ifdef ESP8266
    _server->setNoDelay(true);
#endif

    // Start mDNS service
    if (MDNS.begin(mdnsName)) {
        MDNS.addService("serial-air", "tcp", port);
        MDNS.addServiceTxt("serial-air", "tcp", "version", WIRELESS_SERIAL_VERSION);
#ifdef ESP8266
        MDNS.addServiceTxt("serial-air", "tcp", "device", "ESP8266");
#elif defined(ESP32)
        MDNS.addServiceTxt("serial-air", "tcp", "device", "ESP32");
#endif
    }

    _running = true;
}

void WirelessSerial::stop() {
    if (!_running) return;

    unmirror();

    // Disconnect all clients
    for (uint8_t i = 0; i < _maxClients; i++) {
        if (_clients[i].connected()) {
            _clients[i].stop();
        }
    }

    if (_server) {
        _server->close();
        delete _server;
        _server = nullptr;
    }

    _running = false;
}

void WirelessSerial::handle() {
    if (!_running) return;

#ifdef ESP8266
    MDNS.update();
#endif

    _acceptNewClients();
    _cleanupClients();
}

DualPrint* WirelessSerial::mirror(Print& serial) {
    unmirror();
    _dualPrint = new DualPrint(serial, *this);
    _mirroring = true;
    return _dualPrint;
}

void WirelessSerial::unmirror() {
    if (_dualPrint) {
        delete _dualPrint;
        _dualPrint = nullptr;
    }
    _mirroring = false;
}

bool WirelessSerial::isMirroring() const {
    return _mirroring;
}

DualPrint* WirelessSerial::getDualPrint() const {
    return _dualPrint;
}

size_t WirelessSerial::write(uint8_t byte) {
    return write(&byte, 1);
}

size_t WirelessSerial::write(const uint8_t* buffer, size_t size) {
    if (!_running) return 0;

    size_t written = 0;
    for (uint8_t i = 0; i < _maxClients; i++) {
        if (_clients[i].connected()) {
            written = _clients[i].write(buffer, size);
        }
    }
    return written;
}

uint8_t WirelessSerial::clientCount() const {
    uint8_t count = 0;
    for (uint8_t i = 0; i < _maxClients; i++) {
        if (_clients[i].connected()) {
            count++;
        }
    }
    return count;
}

bool WirelessSerial::isRunning() const {
    return _running;
}

void WirelessSerial::setMaxClients(uint8_t maxClients) {
    if (maxClients > WIRELESS_SERIAL_MAX_CLIENTS_LIMIT) {
        maxClients = WIRELESS_SERIAL_MAX_CLIENTS_LIMIT;
    }
    if (maxClients < 1) {
        maxClients = 1;
    }
    _maxClients = maxClients;
}

void WirelessSerial::_acceptNewClients() {
    if (!_server) return;

#ifdef ESP8266
    if (_server->hasClient()) {
        // Find a free slot
        for (uint8_t i = 0; i < _maxClients; i++) {
            if (!_clients[i].connected()) {
                _clients[i] = _server->accept();
                _clients[i].setNoDelay(true);
                return;
            }
        }
        // No free slot, reject
        WiFiClient rejected = _server->accept();
        rejected.stop();
    }
#else // ESP32
    WiFiClient newClient = _server->available();
    if (newClient) {
        for (uint8_t i = 0; i < _maxClients; i++) {
            if (!_clients[i].connected()) {
                _clients[i] = newClient;
                return;
            }
        }
        // No free slot, reject
        newClient.stop();
    }
#endif
}

void WirelessSerial::_cleanupClients() {
    for (uint8_t i = 0; i < _maxClients; i++) {
        if (_clients[i] && !_clients[i].connected()) {
            _clients[i].stop();
        }
    }
}
