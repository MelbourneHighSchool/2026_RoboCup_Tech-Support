#pragma once

#include "linux_wire.h"

#include <array>
#include <cstdint>
#include <mutex>
#include <stdexcept>

namespace hardware {

// Synchronous PCB protocol client. The shared bus must outlive this object.
// Methods acquire bus.mutex internally: callers must not already hold it.
// Transport errors propagate without retries (a kick may already have fired).
class Pcb {
public:
    explicit Pcb(TwoWire& bus) : bus_(bus) {}

    // Raw 0–255 readings, starting at the front and proceeding clockwise.
    // Voltage = reading * 3.3 / 255; sensor bearing = index * 11.25 degrees.
    std::array<uint8_t, 32> read_sensors() {
        std::lock_guard<std::mutex> lock(bus_.mutex);
        // A plain read: sending a register byte would execute a PCB command.
        bus_.requestFrom(ADDRESS, 32, 1);
        std::array<uint8_t, 32> values{};
        for (auto& value : values)
            value = bus_.read();
        return values;
    }

    // 0 = off, 254 = full brightness. Validate before narrowing to a byte so
    // invalid input cannot wrap around to the reserved kick command (255).
    void set_brightness(int level) {
        if (level < 0 || level > 254)
            throw std::out_of_range("PCB brightness must be 0–254");
        send(static_cast<uint8_t>(level));
    }

    // Sends one request; pulse timing and cooldown belong to the PCB firmware.
    // A successful write confirms delivery, not that the PCB accepted the kick.
    void kick() { send(0xFF); }

private:
    static constexpr uint8_t ADDRESS = 0x37;
    TwoWire& bus_;

    void send(uint8_t command) {
        std::lock_guard<std::mutex> lock(bus_.mutex);
        bus_.beginTransmission(ADDRESS);
        bus_.write(command);
        bus_.endTransmission();
    }
};

} // namespace hardware
