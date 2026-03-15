#ifndef DUAL_PRINT_H
#define DUAL_PRINT_H

#include <Arduino.h>

/// Print wrapper that outputs to two Print destinations simultaneously.
/// Used internally by WirelessSerial mirror mode and can also be used directly.
class DualPrint : public Print {
public:
    DualPrint(Print& primary, Print& secondary)
        : _primary(primary), _secondary(secondary) {}

    size_t write(uint8_t byte) override {
        _primary.write(byte);
        return _secondary.write(byte);
    }

    size_t write(const uint8_t* buffer, size_t size) override {
        _primary.write(buffer, size);
        return _secondary.write(buffer, size);
    }

private:
    Print& _primary;
    Print& _secondary;
};

#endif // DUAL_PRINT_H
