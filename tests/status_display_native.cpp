#include "status_display.h"
#include <cassert>
#include <algorithm>
#include <atomic>
#include <vector>
#include <stdexcept>

using namespace hardware;
struct FakeDisplayWire : TwoWire {
    std::vector<std::vector<uint8_t>> packets;
    bool fail = false;
    void transfer(uint8_t address, bool reading, uint8_t* data, size_t size) override {
        assert(address == 0x3c && !reading && size <= 32);
        if (fail) throw std::runtime_error("OLED unplugged");
        packets.emplace_back(data, data + size);
    }
};
int main() {
    auto wire = std::make_shared<FakeDisplayWire>();
    StatusDisplay display("unused", 0x3c, wire, false);
    display.update("STRIKER", true, "RUNNING");
    assert(display.refresh(0));
    assert(wire->packets.front()[1] == 0xae);
    assert(wire->packets.front().back() == 0xaf);
    assert(wire->packets.size() == 129); // Init, then 64 address/data pairs.
    const auto count = wire->packets.size();
    assert(display.refresh(0));
    assert(wire->packets.size() == count); // Identical frame sends nothing.
    display.component("IMU", '!', "DISCONNECTED - STOPPED");
    assert(display.refresh(0));
    assert(wire->packets.size() > count && wire->packets.size() < count + 128);
    display.component("CAMERA", '!', "NO FRAMES");
    const auto before_busy = wire->packets.size();
    wire->mutex.lock();
    // try_lock a mutex from another thread, not its owning thread.
    std::thread busy([&] { assert(!display.refresh(0)); });
    busy.join();
    wire->mutex.unlock();
    assert(wire->packets.size() == before_busy);
    wire->fail = true;
    assert(!display.refresh(0));
    assert(!display.error().empty());
    wire->fail = false;
    assert(display.refresh(0));
    assert(display.error().empty());
    DisplayStatus status;
    status.mode = "DEFENCE"; status.run = true; status.state = "BLOCKED";
    status.errors = {{"CAMERA", "NO FRAMES"}, {"IMU", "DISCONNECTED"}};
    auto page1 = render_status(status, 0), page2 = render_status(status, 1);
    assert(page1 != page2);
    assert(std::equal(page1.begin(), page1.begin() + 4 * 128, page2.begin()));
    assert(page1 == render_status(status, 2));
    status.state = "RUNNING";
    status.native_blocked = true;
    assert(render_status(status, 0) == page1); // Native fault overrides a stalled Python label.
    assert(std::any_of(page1.begin(), page1.begin() + 2 * 128, [](auto pixel) { return pixel; }));
    auto data_start = wire->packets.size();
    display.stop();
    assert(wire->packets[data_start] == std::vector<uint8_t>({0, 0xae}));
    for (size_t i = data_start; i < wire->packets.size(); ++i)
        if (wire->packets[i][0] == 0x40)
            assert(std::all_of(wire->packets[i].begin() + 1, wire->packets[i].end(),
                               [](auto byte) { return byte == 0; }));
    auto stopped_count = wire->packets.size();
    display.stop();
    assert(wire->packets.size() == stopped_count);
}
