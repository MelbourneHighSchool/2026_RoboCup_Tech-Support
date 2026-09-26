#pragma once

#include "PowerfulBLDCdriver.h"
#include "imu/linux_bno08x.h"
#include "linux_kicker.h"
#include "pcb.h"
#include "status_display.h"
#include <array>
#include <atomic>
#include <condition_variable>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>
#include <functional>

namespace hardware {
constexpr double RPM_TO_MOTOR_SPEED = 275251.2;
struct MotorCalibration {
    int address;
    uint32_t elecangleoffset;
    int32_t sincoscentre;
};
struct DriveConfig {
    double diameter, max_yaw_rpm, max_rpm, yaw_correct_threshold;
    void validate() const;
};
struct Command {
    double direction = 0, speed = 0, rotation = 0, rotation_speed = 0, yaw = 0;
    int dribbler = 0;
    bool kick = false;
};
struct HardwareHealth {
    bool imu_healthy;
    std::string fault_source, error;
    int motor_address;
    uint64_t imu_recovery_generation;
};
struct PcbSnapshot {
    std::array<uint8_t, PCB_SENSOR_COUNT> readings{};
    // Read completion time in steady-clock seconds, absent before the first read.
    std::optional<double> timestamp_s;
    std::optional<double> age_s;
    bool valid = false; // Caller must also check age_s for its freshness limit.
};
class MotorCommunicationError : public std::runtime_error {
    using std::runtime_error::runtime_error;
};
// Pure kinematics shared by the drive loop and offline tests.
std::array<double, 4> calculate_drive_rpms(const Command& command, const DriveConfig& config);
std::pair<double, double> body_velocity(const std::array<double, 4>& rpms, double diameter);

// Owns motor I/O and its native drive thread. Future hardware belongs here.

class HardwareController {
public:
    HardwareController(const std::vector<MotorCalibration>& calibration, DriveConfig config,
                       const std::string& device = "/dev/i2c-1",
                       std::unique_ptr<TwoWire> transport = nullptr,
                       int imu_address = 0x4a, double imu_report_interval_ms = 10,
                       int kicker_pin = -1, const std::string& kicker_gpiochip = "",
                       std::unique_ptr<KickerOutput> kicker_output = nullptr,
                       double drive_motor_current_limit = 8.0,
                       double dribbler_motor_current_limit = 1.0,
                       double kick_pulse_length = 0.02, double kick_cooldown = 0.5,
                       std::shared_ptr<StatusDisplay> display = nullptr, bool use_pcb = false,
                       double motor_hz = 50, double pcb_hz = 50, double imu_poll_hz = 500);
    ~HardwareController();
    HardwareController(const HardwareController&) = delete;
    HardwareController& operator=(const HardwareController&) = delete;
    void move(double direction, double speed, double rotation, double rotation_speed,
              int dribbler = 0, bool kick = false);
    std::optional<double> get_raw_imu_yaw() const { return imu_->snapshot().raw_yaw; }
    std::optional<double> get_yaw() const { return imu_->snapshot().yaw; }
    std::optional<double> get_gyro_z_deg_s() const { return imu_->snapshot().gyro_z; }
    std::optional<std::array<double, 4>> get_latest_quaternion() const { return imu_->snapshot().quaternion; }
    uint64_t imu_update_count() const { return imu_->snapshot().update_count; }
    void set_startup_yaw(double raw_yaw) { imu_->set_startup_yaw(raw_yaw); }
    HardwareHealth health() const;
    PcbSnapshot get_pcb_snapshot() const;
    std::pair<double, double> get_measured_body_velocity_mm_s(double yaw_deg);
    struct LocalisationSample {
        double vx, vy, timestamp_s, read_span_s;
        LinuxBno08x::History imu;
    };
    LocalisationSample get_localisation_sample();
    double get_dribbler_rpm();
    void stop();
    void set_drive_current_limits(double constant_speed_amps, double acceleration_amps);
    uint64_t loop_count() const { return loop_count_.load(); }
    std::map<std::string, double> timing_diagnostics() const;
    double current_speed() const;
    double current_direction() const;
private:
    void drive_loop() noexcept;
    void imu_loop() noexcept;
    void kicker_loop() noexcept;
    void pcb_loop() noexcept;
    std::string disable_motors(); // Caller holds wire_->mutex; tries every write on every motor.
    void check_state() const; // Caller holds state_mutex_.
    void fail(const std::string& message, const std::string& source = "MOTOR", int address = -1);
    void check_imu_locked(); // state_mutex_; never acquires the bus.
    void motor_operation(size_t index, const std::function<void()>& operation);
    void record_timing(const std::string& name, double wait_s, double work_s);
    mutable std::mutex timing_mutex_;
    std::map<std::string, double> timing_;
    std::chrono::steady_clock::duration motor_period_, pcb_period_, imu_poll_period_;
    DriveConfig config_;
    std::shared_ptr<TwoWire> wire_;
    std::shared_ptr<StatusDisplay> display_;
    std::vector<int> addresses_;
    std::unique_ptr<LinuxBno08x> imu_;
    std::unique_ptr<KickerOutput> kicker_;
    std::unique_ptr<Pcb> pcb_;
    bool use_pcb_;
    PcbSnapshot pcb_snapshot_; // state_mutex_; getters return an owned copy
    std::vector<PowerfulBLDCdriver> motors_;
    int32_t drive_motor_current_limit_, dribbler_motor_current_limit_;
    int32_t constant_speed_current_limit_, acceleration_current_limit_; // state_mutex_
    std::chrono::steady_clock::duration kick_pulse_, kick_cooldown_;
    mutable std::mutex state_mutex_;
    std::mutex stop_mutex_;
    std::condition_variable wake_;
    Command target_;
    bool kicking_ = false; // Protected by state_mutex_, including the cooldown timestamp.
    std::chrono::steady_clock::time_point next_kick_time_{};
    double dx_ = 0, dy_ = 0;
    std::string error_, fault_source_;
    int fault_address_ = -1;
    bool imu_seen_ = false, imu_unavailable_ = false;
    uint64_t imu_recovery_generation_ = 0;
    bool last_imu_fresh_ = false;
    std::chrono::steady_clock::time_point imu_ready_deadline_{};
    std::atomic<bool> running_{false};
    std::atomic<uint64_t> loop_count_{0};
    std::thread thread_;
    std::thread imu_thread_;
    std::thread kicker_thread_;
    std::thread pcb_thread_;
};
} // namespace hardware
