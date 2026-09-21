// Offline protocol check:
// g++ -std=c++17 -Wall -Wextra -Werror -pthread -Ilib tests/pcb_native.cpp lib/linux_wire.cpp -o /tmp/pcb_native
// /tmp/pcb_native
#include "pcb.h"

#include <cassert>
#include <limits>
#include <system_error>
#include <vector>

class FakeWire final : public TwoWire {
public:
    int calls = 0;
    bool fail = false;
    std::vector<uint8_t> commands;

protected:
    void transfer(uint8_t address, bool reading, uint8_t* data, size_t size) override {
        ++calls;
        assert(address == 0x37);
        if (fail)
            throw std::system_error(std::make_error_code(std::errc::io_error));
        if (reading) {
            assert(size == 32);
            for (size_t i = 0; i < size; ++i)
                data[i] = static_cast<uint8_t>(255 - i);
        } else {
            assert(size == 1);
            commands.push_back(data[0]);
        }
    }
};

int main() {
    FakeWire bus;
    hardware::Pcb pcb(bus);
    assert(bus.calls == 0);
    const auto values = pcb.read_sensors();
    assert(bus.calls == 1); // No register write before the read.
    for (size_t i = 0; i < values.size(); ++i)
        assert(values[i] == 255 - i);

    pcb.set_brightness(0);
    pcb.set_brightness(127);
    pcb.set_brightness(254);
    pcb.kick();
    assert((bus.commands == std::vector<uint8_t>{0, 127, 254, 255}));

    const int before_invalid = bus.calls;
    for (int level : {-1, 255, 256, std::numeric_limits<int>::min(),
                      std::numeric_limits<int>::max()}) {
        bool rejected = false;
        try { pcb.set_brightness(level); }
        catch (const std::out_of_range&) { rejected = true; }
        assert(rejected);
    }
    assert(bus.calls == before_invalid);

    // All operations propagate failures exactly once and release the mutex.
    for (int operation = 0; operation < 3; ++operation) {
        bus.fail = true;
        const int before = bus.calls;
        bool failed = false;
        try {
            if (operation == 0) pcb.read_sensors();
            else if (operation == 1) pcb.set_brightness(100);
            else pcb.kick();
        } catch (const std::system_error&) { failed = true; }
        assert(failed);
        assert(bus.calls == before + 1);
        bus.fail = false;
        pcb.read_sensors();
        assert(bus.calls == before + 2);
    }
}
