#pragma once

#include "../linux_wire.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include <array>
#include <chrono>
#include <exception>
#include <mutex>
#include <optional>
#include <deque>
#include <vector>
#include <utility>

namespace hardware {
// Owns the process-wide SH-2 session. The caller serializes initialize/service/close
// with motor I/O. Getters only lock the sample mutex and never perform I2C.
class LinuxBno08x {
public:
    using Clock = std::chrono::steady_clock;
    struct Snapshot {
        std::optional<std::array<double, 4>> quaternion;
        std::optional<double> raw_yaw, yaw, gyro_z;
        double last_yaw = 0; // Retained for field-to-body translation during outages.
        uint64_t update_count = 0, gyro_count = 0;
        double yaw_received_s = 0, gyro_received_s = 0;
    };
    LinuxBno08x(TwoWire& wire, int address = 0x4a, double report_interval_ms = 10);
    ~LinuxBno08x();
    LinuxBno08x(const LinuxBno08x&) = delete;
    LinuxBno08x& operator=(const LinuxBno08x&) = delete;
    void initialize();
    void service();
    void close() noexcept;
    void set_startup_yaw(double raw_yaw);
    Snapshot snapshot() const;
    struct History {
        uint64_t epoch;
        std::vector<std::pair<double,double>> yaw, gyro;
    };
    History history() const;
private:
    struct Hal : sh2_Hal_t { LinuxBno08x* owner; };
    static LinuxBno08x& owner(sh2_Hal_t* hal);
    static int hal_open(sh2_Hal_t* hal) noexcept;
    static void hal_close(sh2_Hal_t* hal) noexcept;
    static int hal_read(sh2_Hal_t* hal, uint8_t* data, unsigned size, uint32_t* timestamp) noexcept;
    static int hal_write(sh2_Hal_t* hal, uint8_t* data, unsigned size) noexcept;
    static uint32_t hal_time(sh2_Hal_t* hal) noexcept;
    static void on_event(void* cookie, sh2_AsyncEvent_t* event);
    static void on_sensor(void* cookie, sh2_SensorEvent_t* event);
    void enable_reports();
    void check_io_error();
    void read_bytes(uint8_t* data, size_t size);
    TwoWire& wire_;
    uint8_t address_;
    uint32_t interval_us_;
    Hal hal_{};
    bool owns_session_ = false, opened_ = false, reset_seen_ = false, reconfigure_ = false;
    bool received_ = false;
    std::exception_ptr io_error_;
    mutable std::mutex imu_mutex_;
    Snapshot sample_;
    std::optional<double> startup_yaw_;
    Clock::time_point yaw_time_{}, gyro_time_{};
    uint64_t history_epoch_ = 0;
    std::deque<std::pair<double,double>> yaw_history_, gyro_history_;
};
} // namespace hardware
