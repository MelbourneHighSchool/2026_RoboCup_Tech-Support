#pragma once
#include "linux_wire.h"
#include <array>
#include <condition_variable>
#include <map>
#include <memory>
#include <thread>

namespace hardware {
struct DisplayStatus {
    std::string mode = "SOCCER", state = "STARTING", detail;
    bool run = false, native_blocked = false;
    // '?' starting, '+' healthy, '!' unavailable.
    std::map<std::string, char> health;
    std::map<std::string, std::string> errors;
};
using DisplayFrame = std::array<uint8_t, 1024>;
DisplayFrame render_status(const DisplayStatus& status, size_t page);

// Uses the same transport and transaction mutex as the motor/IMU controller.
// Can outlive that controller and also render constructor failures.
class StatusDisplay {
public:
    explicit StatusDisplay(const std::string& device = "/dev/i2c-1", int address = 0x3c,
                           std::shared_ptr<TwoWire> transport = nullptr, bool start_worker = true);
    ~StatusDisplay();
    void update(const std::string& mode, bool run, const std::string& state,
                const std::string& detail = "");
    void component(const std::string& source, char health, const std::string& error = "");
    void set_native_blocked(bool blocked);
    std::string error() const;
    void stop();
    std::shared_ptr<TwoWire> bus() const { return wire_; }
    int address() const { return address_; }
    // Also used by deterministic offline protocol tests. Single rendering owner only.
    bool refresh(size_t page);
private:
    void worker() noexcept;
    void commands(std::initializer_list<uint8_t> bytes);
    std::shared_ptr<TwoWire> wire_;
    uint8_t address_;
    mutable std::mutex mutex_;
    std::mutex stop_mutex_;
    std::condition_variable wake_;
    DisplayStatus status_;
    DisplayFrame previous_{};
    std::array<bool, 64> valid_{};
    bool initialized_ = false, stopped_ = false;
    std::string error_;
    std::thread thread_;
};
}
