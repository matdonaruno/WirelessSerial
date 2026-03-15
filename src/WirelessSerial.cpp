#include "WirelessSerial.h"

WirelessSerial::WirelessSerial()
    : _server(nullptr)
    , _maxClients(WIRELESS_SERIAL_DEFAULT_MAX_CLIENTS)
    , _running(false)
    , _port(WIRELESS_SERIAL_DEFAULT_PORT)
    , _dualPrint(nullptr)
    , _mirroring(false)
    , _ringBuf(nullptr)
    , _ringSize(0)
    , _ringHead(0)
    , _ringCount(0)
{
}

WirelessSerial::~WirelessSerial() {
    stop();
    disableBuffer();
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

    // Check if any client is connected
    bool hasClient = false;
    size_t written = 0;
    for (uint8_t i = 0; i < _maxClients; i++) {
        if (_clients[i].connected()) {
            hasClient = true;
            written = _clients[i].write(buffer, size);
        }
    }

    // No clients connected — store in ring buffer if enabled
    if (!hasClient && _ringBuf) {
        _bufferWrite(buffer, size);
        return size;
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

// ========== Buffer ==========

void WirelessSerial::enableBuffer(size_t size) {
    disableBuffer();
    if (size == 0) return;
    _ringBuf = new uint8_t[size];
    _ringSize = size;
    _ringHead = 0;
    _ringCount = 0;
}

void WirelessSerial::disableBuffer() {
    if (_ringBuf) {
        delete[] _ringBuf;
        _ringBuf = nullptr;
    }
    _ringSize = 0;
    _ringHead = 0;
    _ringCount = 0;
}

size_t WirelessSerial::bufferedBytes() const {
    return _ringCount;
}

void WirelessSerial::_bufferWrite(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        _ringBuf[_ringHead] = data[i];
        _ringHead = (_ringHead + 1) % _ringSize;
        if (_ringCount < _ringSize) {
            _ringCount++;
        }
        // When full, _ringHead overwrites oldest data (ring behavior)
    }
}

void WirelessSerial::_flushBufferTo(WiFiClient& client) {
    if (!_ringBuf || _ringCount == 0) return;

    // Calculate start position of oldest data
    size_t start;
    if (_ringCount < _ringSize) {
        start = 0;
    } else {
        start = _ringHead; // head has wrapped, oldest is at head
    }

    // Send in up to 2 chunks (wrap-around)
    if (start + _ringCount <= _ringSize) {
        // Contiguous
        client.write(_ringBuf + start, _ringCount);
    } else {
        // Wraps around
        size_t firstChunk = _ringSize - start;
        client.write(_ringBuf + start, firstChunk);
        client.write(_ringBuf, _ringCount - firstChunk);
    }

    // Clear buffer after flush
    _ringHead = 0;
    _ringCount = 0;
}

// ========== Private ==========

void WirelessSerial::_acceptNewClients() {
    if (!_server) return;

#ifdef ESP8266
    if (_server->hasClient()) {
        for (uint8_t i = 0; i < _maxClients; i++) {
            if (!_clients[i].connected()) {
                _clients[i] = _server->accept();
                _clients[i].setNoDelay(true);
                // Flush buffered data to new client
                _flushBufferTo(_clients[i]);
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
                // Flush buffered data to new client
                _flushBufferTo(_clients[i]);
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
