#include "linux_bno08x.h"
#include "sh2_err.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <thread>

namespace hardware {
namespace {
std::atomic<bool> session_owned{false};
constexpr double DEG = 180.0 / 3.14159265358979323846;
double wrap(double value) {
    value = std::fmod(value + 180.0, 360.0);
    return (value < 0 ? value + 360.0 : value) - 180.0;
}
}
LinuxBno08x::LinuxBno08x(TwoWire& wire, int address, double report_interval_ms)
    : wire_(wire) {
    if (address < 8 || address > 119 || !std::isfinite(report_interval_ms) || report_interval_ms < 1 || report_interval_ms > 1000)
        throw std::invalid_argument("Invalid IMU address or report interval (1..1000 ms)");
    bool expected = false;
    if (!session_owned.compare_exchange_strong(expected, true))
        throw std::runtime_error("Only one native BNO08x session is supported per process");
    owns_session_ = true;
    address_ = static_cast<uint8_t>(address);
    interval_us_ = static_cast<uint32_t>(report_interval_ms * 1000);
    hal_.owner = this;
    hal_.open = hal_open;
    hal_.close = hal_close;
    hal_.read = hal_read;
    hal_.write = hal_write;
    hal_.getTimeUs = hal_time;
}
LinuxBno08x::~LinuxBno08x() { close(); }
LinuxBno08x& LinuxBno08x::owner(sh2_Hal_t* hal) { return *static_cast<Hal*>(hal)->owner; }
void LinuxBno08x::check_io_error() {
    if (io_error_) {
        auto error = io_error_;
        io_error_ = nullptr;
        std::rethrow_exception(error);
    }
}
void LinuxBno08x::initialize() {
    if (!owns_session_ || opened_) throw std::runtime_error("Invalid BNO08x session state");
    // sh2_open ignores a HAL open failure and can return OK after reset timeout.
    // Check both the transport error and reset callback explicitly.
    const int result = sh2_open(&hal_, on_event, this);
    opened_ = result == SH2_OK;
    check_io_error();
    if (!opened_ || !reset_seen_) throw std::runtime_error("BNO08x reset/advertisement timed out");
    sh2_ProductIds_t ids{};
    const int product_result = sh2_getProdIds(&ids);
    check_io_error();
    if (product_result != SH2_OK || ids.numEntries == 0)
        throw std::runtime_error("BNO08x product identification failed");
    sh2_setSensorCallback(on_sensor, this);
    enable_reports();
}
void LinuxBno08x::enable_reports() {
    sh2_SensorConfig_t config{};
    config.reportInterval_us = interval_us_;
    for (auto sensor : {SH2_GAME_ROTATION_VECTOR, SH2_GYROSCOPE_CALIBRATED}) {
        const int result = sh2_setSensorConfig(sensor, &config);
        check_io_error();
        if (result != SH2_OK) throw std::runtime_error("BNO08x report configuration failed");
    }
    reconfigure_ = false;
}
void LinuxBno08x::service() {
    if (!opened_) return;
    // Drain a bounded batch: two 100 Hz reports may arrive in separate packets.
    for (int i = 0; i < 8; ++i) {
        received_ = false;
        sh2_service();
        check_io_error();
        if (reconfigure_) enable_reports(); // Never re-enter SH-2 from its callback.
        if (!received_) break;
    }
}
void LinuxBno08x::close() noexcept {
    if (opened_) sh2_close();
    opened_ = false;
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        yaw_time_ = gyro_time_ = {};
        ++history_epoch_;
        yaw_history_.clear(); gyro_history_.clear();
    }
    if (owns_session_) session_owned = false;
    owns_session_ = false;
}
int LinuxBno08x::hal_open(sh2_Hal_t* hal) noexcept {
    auto& self = owner(hal);
    uint8_t reset[] = {5, 0, 1, 0, 1};
    for (int i = 0; i < 5; ++i) {
        if (hal_write(hal, reset, sizeof(reset)) == sizeof(reset)) {
            self.io_error_ = nullptr;
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            return SH2_OK;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    return SH2_ERR;
}
void LinuxBno08x::hal_close(sh2_Hal_t*) noexcept {
    // The shared bus belongs to HardwareController; no reset GPIO is connected.
}
void LinuxBno08x::read_bytes(uint8_t* data, size_t size) {
    wire_.requestFrom(address_, static_cast<uint8_t>(size), 1);
    for (size_t i = 0; i < size; ++i) data[i] = wire_.read();
}
int LinuxBno08x::hal_read(sh2_Hal_t* hal, uint8_t* data, unsigned capacity,
                        uint32_t* timestamp) noexcept {
    auto& self = owner(hal);
    try {
        uint8_t header[4];
        *timestamp = hal_time(hal); // Poll arrival time; no interrupt pin is used.
        self.read_bytes(header, sizeof(header));
        const size_t size = (header[0] | (uint16_t(header[1]) << 8)) & 0x7fff;
        if (size == 0) return 0;
        if (size < 4) throw std::runtime_error("Invalid BNO08x SHTP length");
        if (size > capacity || size > SH2_HAL_MAX_TRANSFER_IN)
            throw std::runtime_error("BNO08x SHTP transfer exceeds receive buffer");
        // Each new I2C read repeats the SHTP header. Include it only once.
        // Bound work per service call; a corrupt length must not monopolize the bus.
        for (size_t cursor = 0; cursor < size;) {
            uint8_t chunk[32];
            const size_t skip = cursor == 0 ? 0 : 4;
            const size_t count = std::min(sizeof(chunk) - skip, size - cursor);
            self.read_bytes(chunk, count + skip);
            std::memcpy(data + cursor, chunk + skip, count);
            cursor += count;
        }
        self.received_ = true;
        return static_cast<int>(size);
    } catch (...) {
        self.io_error_ = std::current_exception();
        return 0; // No complete transfer; service() reports the error outside C.
    }
}
int LinuxBno08x::hal_write(sh2_Hal_t* hal, uint8_t* data, unsigned size) noexcept {
    auto& self = owner(hal);
    try {
        // All reset, product-ID and feature commands used here fit in 32 bytes.
        // Returning a partial count would silently drop bytes in this SH-2 core.
        if (size > 32) throw std::runtime_error("BNO08x command exceeds I2C chunk size");
        self.wire_.beginTransmission(self.address_);
        for (unsigned i = 0; i < size; ++i) self.wire_.write(data[i]);
        self.wire_.endTransmission();
        return static_cast<int>(size);
    } catch (...) {
        self.io_error_ = std::current_exception();
        return SH2_ERR; // Returning zero would cause an unbounded SHTP retry loop.
    }
}
uint32_t LinuxBno08x::hal_time(sh2_Hal_t*) noexcept {
    return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        Clock::now().time_since_epoch()).count());
}
void LinuxBno08x::on_event(void* cookie, sh2_AsyncEvent_t* event) {
    auto& self = *static_cast<LinuxBno08x*>(cookie);
    if (event->eventId == SH2_RESET) {
        self.reset_seen_ = self.reconfigure_ = true;
        std::lock_guard<std::mutex> lock(self.imu_mutex_);
        self.yaw_time_ = self.gyro_time_ = {};
        ++self.history_epoch_;
        self.yaw_history_.clear(); self.gyro_history_.clear();
    }
}
void LinuxBno08x::on_sensor(void* cookie, sh2_SensorEvent_t* event) {
    auto& self = *static_cast<LinuxBno08x*>(cookie);
    sh2_SensorValue_t value{};
    if (sh2_decodeSensorEvent(&value, event) != SH2_OK) return;
    std::lock_guard<std::mutex> lock(self.imu_mutex_);
    // SH-2 timestamps extend the HAL's low 32 monotonic microseconds, with
    // report-delay correction. Align that wrap domain to the host clock.
    const auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
        Clock::now().time_since_epoch()).count();
    const int32_t delta = static_cast<int32_t>(
        static_cast<uint32_t>(event->timestamp_uS) - static_cast<uint32_t>(now_us));
    const double timestamp = (now_us + delta) * 1e-6;
    const auto append = [&](auto& history, double reading) {
        // Invalid/late times remain usable for live control, but not deskew.
        if (delta > 0 || delta < -250000 ||
            (!history.empty() && timestamp <= history.back().first)) return;
        history.emplace_back(timestamp, reading);
        while (history.size() > 2 && history[1].first < timestamp-2) history.pop_front();
    };
    if (value.sensorId == SH2_GAME_ROTATION_VECTOR) {
        const auto& q = value.un.gameRotationVector;
        const double yaw = std::atan2(2.0 * (q.real * q.k + q.i * q.j),
                                     1.0 - 2.0 * (q.j * q.j + q.k * q.k)) * DEG;
        if (!std::isfinite(yaw) || q.i*q.i + q.j*q.j + q.k*q.k + q.real*q.real < 0.5) return;
        self.sample_.quaternion = {q.i, q.j, q.k, q.real};
        self.sample_.raw_yaw = yaw;
        if (self.startup_yaw_) {
            self.sample_.yaw = wrap(*self.startup_yaw_ - yaw);
            self.sample_.last_yaw = *self.sample_.yaw;
            append(self.yaw_history_, *self.sample_.yaw);
        }
        self.yaw_time_ = Clock::now();
        ++self.sample_.update_count;
        self.sample_.yaw_received_s = now_us * 1e-6;
    } else if (value.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        // Match startup_yaw - raw_yaw: the upside-down mounting requires
        // reversing sensor Z to report clockwise-positive angular velocity.
        const double z = -value.un.gyroscope.z * DEG;
        if (!std::isfinite(z)) return;
        self.sample_.gyro_z = z;
        ++self.sample_.gyro_count;
        self.sample_.gyro_received_s = now_us * 1e-6;
        append(self.gyro_history_, z);
        self.gyro_time_ = Clock::now();
    }
}
void LinuxBno08x::set_startup_yaw(double raw_yaw) {
    if (!std::isfinite(raw_yaw)) throw std::invalid_argument("Startup yaw must be finite");
    std::lock_guard<std::mutex> lock(imu_mutex_);
    startup_yaw_ = wrap(raw_yaw);
    ++history_epoch_;
    yaw_history_.clear(); gyro_history_.clear();
    if (sample_.raw_yaw) {
        sample_.yaw = wrap(*startup_yaw_ - *sample_.raw_yaw);
        sample_.last_yaw = *sample_.yaw;
    }
}
LinuxBno08x::Snapshot LinuxBno08x::snapshot() const {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    auto result = sample_;
    const auto now = Clock::now();
    if (now - yaw_time_ >= std::chrono::milliseconds(100)) {
        result.raw_yaw.reset();
        result.yaw.reset();
        result.quaternion.reset();
    }
    if (now - gyro_time_ >= std::chrono::milliseconds(100)) result.gyro_z.reset();
    return result;
}
LinuxBno08x::History LinuxBno08x::history() const {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return {history_epoch_, {yaw_history_.begin(), yaw_history_.end()},
            {gyro_history_.begin(), gyro_history_.end()}};
}
} // namespace hardware
