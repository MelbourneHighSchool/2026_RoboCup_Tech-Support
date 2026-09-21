#include "hardware_controller.h"
#include "pcb.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <set>

namespace hardware {
namespace {
constexpr double PI = 3.14159265358979323846;
constexpr double RAD = PI / 180.0;
constexpr int32_t AMPS_PER_LSB = 1 << 16;
constexpr int32_t MOTOR_SPEED_LIMIT = 546133333;

double wrap(double angle) {
    double result = std::fmod(angle + 180.0, 360.0);
    if (result < 0) result += 360.0;
    return result - 180.0;
}
void finite(double value) {
    if (!std::isfinite(value)) throw std::invalid_argument("Motor parameters must be finite");
}
int32_t current_limit_lsb(double amps) {
    finite(amps);
    if (amps < 0 || amps > static_cast<double>(INT32_MAX) / AMPS_PER_LSB)
        throw std::invalid_argument("Motor current limits must be non-negative and fit in int32 LSB");
    return static_cast<int32_t>(amps * AMPS_PER_LSB);
}
std::chrono::steady_clock::duration seconds_duration(double seconds, const char* name) {
    if (!std::isfinite(seconds) || seconds < 0)
        throw std::invalid_argument(std::string(name) + " must be finite and non-negative");
    return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(seconds));
}
}
void DriveConfig::validate() const {
    for (double value : {diameter, max_yaw_rpm, max_rpm, yaw_correct_threshold}) finite(value);
    if (diameter <= 0 || max_rpm < 0 || max_yaw_rpm < 0 || yaw_correct_threshold < 0 ||
        max_rpm > MOTOR_SPEED_LIMIT / RPM_TO_MOTOR_SPEED ||
        max_yaw_rpm > MOTOR_SPEED_LIMIT / RPM_TO_MOTOR_SPEED)
        throw std::invalid_argument("Invalid wheel diameter, RPM limit or yaw threshold");
}
std::array<double, 4> calculate_drive_rpms(const Command& c, const DriveConfig& config) {
    const double error = wrap(c.rotation - c.yaw); // Yaw error
    double correction = 0;
    if (std::abs(error) > config.yaw_correct_threshold) {
        // Apply proportional correction
        correction = std::clamp(error / 60.0 * config.max_yaw_rpm *
                                std::clamp(c.rotation_speed, 0.0, 1.0),
                                -config.max_yaw_rpm, config.max_yaw_rpm);
    }
    // Convert global direction to local direction, offset by 45 degrees for easy maths.
    const double local = (wrap(c.yaw - c.direction) + 45.0) * RAD;
    // Convert translational speed to wheel rpm
    const double rpm = c.speed * 60.0 / (config.diameter * PI);
    // Calculate wheel rpms
    std::array<double, 4> result{-std::sin(local) * rpm, std::cos(local) * rpm,
                                std::sin(local) * rpm, -std::cos(local) * rpm};
    double peak = 0;
    for (double value : result) peak = std::max(peak, std::abs(value)); // Get the max rpm
    // Scale wheel rpms down to maintain direction and stay within max speed with room for yaw correction
    const double available = std::max(config.max_rpm - std::abs(correction), 0.0);
    const double scale = peak > available && peak > 0 ? available / peak : 1.0;
    // Apply yaw correction
    for (double& value : result)
        value = std::clamp(value * scale - correction, -config.max_rpm, config.max_rpm);
    return result;
}
std::pair<double, double> body_velocity(const std::array<double, 4>& rpms, double diameter) {
    // Opposite-wheel differences cancel rotation. Body y is LEFT, independent of yaw.
    const double factor = diameter * PI / 60.0;
    const double s = (rpms[2] - rpms[0]) * 0.5 * factor;
    const double c = (rpms[1] - rpms[3]) * 0.5 * factor;
    if (std::hypot(s, c) < 1e-3) return {0, 0};
    return {(s + c) / std::sqrt(2.0), (s - c) / std::sqrt(2.0)};
}
HardwareController::HardwareController(const std::vector<MotorCalibration>& calibration,
                                     DriveConfig config, const std::string& device,
                                     std::unique_ptr<TwoWire> transport,
                                     int imu_address, double imu_report_interval_ms,
                                     int kicker_pin, const std::string& kicker_gpiochip,
                                     std::unique_ptr<KickerOutput> kicker_output,
                                     double drive_motor_current_limit,
                                     double dribbler_motor_current_limit,
                                     double kick_pulse_length, double kick_cooldown,
                                     std::shared_ptr<StatusDisplay> display, bool use_pcb,
                                     double motor_hz, double pcb_hz, double imu_poll_hz) :
    config_(config),
    use_pcb_(use_pcb),
    drive_motor_current_limit_(current_limit_lsb(drive_motor_current_limit)),
    dribbler_motor_current_limit_(current_limit_lsb(dribbler_motor_current_limit)),
    constant_speed_current_limit_(drive_motor_current_limit_),
    acceleration_current_limit_(drive_motor_current_limit_),
    kick_pulse_(seconds_duration(kick_pulse_length, "Kick pulse length")),
    kick_cooldown_(seconds_duration(kick_cooldown, "Kick cooldown")) {
    const auto period = [](double hz, double maximum) {
        if (!std::isfinite(hz) || hz < 10 || hz > maximum)
            throw std::invalid_argument("Polling rate outside supported range");
        return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / hz));
    };
    motor_period_ = period(motor_hz, 200);
    pcb_period_ = period(pcb_hz, 200);
    imu_poll_period_ = period(imu_poll_hz, 1000);
    config_.validate();
    if (calibration.size() != 4 && calibration.size() != 5)
        throw std::invalid_argument("HardwareController requires four wheels and an optional dribbler");
    std::set<int> addresses;
    for (const auto& cal : calibration) {
        if (cal.address < 8 || cal.address > 119 || !addresses.insert(cal.address).second ||
            cal.sincoscentre < 0 || cal.sincoscentre > 4096)
            throw std::invalid_argument("Invalid or duplicate motor address, or invalid calibration centre");
    }
    if (imu_address < 8 || imu_address > 119 || addresses.count(imu_address) ||
        !std::isfinite(imu_report_interval_ms) || imu_report_interval_ms < 1 || imu_report_interval_ms > 1000)
        throw std::invalid_argument("Invalid IMU address or report interval (1..1000 ms)");
    if (kicker_pin < -1 || kicker_pin > 27)
        throw std::invalid_argument("Kicker pin must be -1 (disabled) or BCM GPIO 0..27");
    display_ = std::move(display);
    if (use_pcb_ && (addresses.count(0x37) || imu_address == 0x37 ||
                    (display_ && display_->address() == 0x37)))
        throw std::invalid_argument("PCB address 0x37 conflicts with another device");
    if (display_ && (transport || addresses.count(display_->address()) || imu_address == display_->address()))
        throw std::invalid_argument("Display transport/address conflicts with controller");
    wire_ = display_ ? display_->bus() : (transport ? std::shared_ptr<TwoWire>(std::move(transport)) :
                                                    std::make_shared<LinuxWire>(device));
    for (const auto& cal : calibration) addresses_.push_back(cal.address);
    // Claim the singleton SH-2 session before any motor writes.
    imu_ = std::make_unique<LinuxBno08x>(*wire_, imu_address, imu_report_interval_ms);
    if (!use_pcb_) kicker_ = std::move(kicker_output);
    if (!use_pcb_ && !kicker_ && kicker_pin >= 0)
        kicker_ = std::make_unique<LinuxKickerOutput>(kicker_pin, kicker_gpiochip);
    motors_.reserve(calibration.size());
    std::string init_source = "MOTOR";
    int init_address = -1;
    try {
        std::unique_lock<std::mutex> bus_lock(wire_->mutex);
        // Register all requested motors before any I/O so failure cleanup attempts all of them.
        for (const auto& cal : calibration) {
            motors_.emplace_back();
            motors_.back().begin(static_cast<uint8_t>(cal.address), wire_.get());
        }
        for (size_t i = 0; i < 4; ++i) {
            auto& motor = motors_[i];
            init_address = calibration[i].address;
            if (motor.getFirmwareVersion() != 3)
                throw MotorCommunicationError("Unsupported motor firmware at address " +
                                              std::to_string(calibration[i].address) + "; expected 3");
            motor.configureCommandMode(2);
            motor.configureOperatingModeAndSensor(3, 1);
            motor.setTorque(0);
            motor.setSpeed(0);
            motor.setCurrentLimitFOC(drive_motor_current_limit_);
            motor.setIdPidConstants(1500, 200);
            motor.setIqPidConstants(1500, 200);
            motor.setSpeedPidConstants(4e-2f, 4e-4f, 3e-2f);
            motor.setPositionPidConstants(275, 0, 0);
            motor.setPositionRegionBoundary(250000);
            motor.setSpeedLimit(MOTOR_SPEED_LIMIT);
            motor.setELECANGLEOFFSET(calibration[i].elecangleoffset);
            motor.setSINCOSCENTRE(calibration[i].sincoscentre);
            motor.configureCommandMode(12);
        }
        if (motors_.size() == 5) {
            auto& motor = motors_[4];
            init_address = calibration[4].address;
            if (motor.getFirmwareVersion() != 3)
                throw MotorCommunicationError("Unsupported motor firmware at address " +
                                              std::to_string(calibration[4].address) + "; expected 3");
            motor.configureCommandMode(2);
            motor.configureOperatingModeAndSensor(3, 1);
            motor.setTorque(0);
            motor.setCurrentLimitFOC(dribbler_motor_current_limit_);
            motor.setIdPidConstants(1500, 200);
            motor.setIqPidConstants(1500, 200);
            motor.setSpeedPidConstants(4e-2f, 4e-4f, 3e-2f);
            motor.setPositionPidConstants(275, 0, 0);
            motor.setPositionRegionBoundary(250000);
            motor.setSpeedLimit(MOTOR_SPEED_LIMIT);
            motor.setELECANGLEOFFSET(calibration[4].elecangleoffset);
            motor.setSINCOSCENTRE(calibration[4].sincoscentre);
            motor.configureCommandMode(2);
        }
        if (display_) display_->component("MOTOR", '+');
        init_source = "IMU";
        init_address = -1;
        imu_->initialize();
        bus_lock.unlock();
        imu_ready_deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        init_source = "OTHER";
        running_ = true;
        imu_thread_ = std::thread(&HardwareController::imu_loop, this);
        if (kicker_) kicker_thread_ = std::thread(&HardwareController::kicker_loop, this);
        if (use_pcb_) {
            pcb_ = std::make_unique<Pcb>(*wire_);
            pcb_thread_ = std::thread(&HardwareController::pcb_loop, this);
        }
        thread_ = std::thread(&HardwareController::drive_loop, this);
    } catch (const std::exception& exc) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            running_ = false;
        }
        wake_.notify_all();
        if (kicker_thread_.joinable()) kicker_thread_.join();
        if (pcb_thread_.joinable()) pcb_thread_.join();
        if (imu_thread_.joinable()) imu_thread_.join();
        std::lock_guard<std::mutex> bus_lock(wire_->mutex);
        imu_->close();
        auto cleanup = disable_motors();
        if (kicker_) {
            try { kicker_->close(); } catch (const std::exception& error) {
                cleanup += "; kicker shutdown: " + std::string(error.what());
            }
        }
        const auto message = init_source + (init_address >= 0 ? " " + std::to_string(init_address) : "") +
                             ": " + exc.what() + (cleanup.empty() ? "" : "; shutdown: " + cleanup);
        if (display_) display_->component(init_source, '!', message);
        if (init_source == "MOTOR") throw MotorCommunicationError(message);
        throw std::runtime_error(message);
    }
}
HardwareController::~HardwareController() {
    try { stop(); } catch (...) { /* Explicit stop reports failures; destructors cannot throw. */ }
}

void HardwareController::motor_operation(size_t index, const std::function<void()>& operation) {
    try { operation(); } catch (const std::exception& exc) {
        const auto message = "Motor " + std::to_string(addresses_[index]) + ": " + exc.what();
        fail(message, "MOTOR", addresses_[index]);
        throw MotorCommunicationError(message);
    }
}
void HardwareController::check_imu_locked() {
    const auto sample = imu_->snapshot();
    const bool fresh = sample.raw_yaw.has_value() && sample.gyro_z.has_value();
    if (fresh) imu_seen_ = true;
    const bool unavailable = !fresh && (imu_seen_ || std::chrono::steady_clock::now() >= imu_ready_deadline_);
    const bool new_outage = unavailable && !imu_unavailable_;
    if (new_outage) {
        ++imu_recovery_generation_;
        std::fprintf(stderr, "IMU unavailable\n");
    }
    if (display_ && (fresh != last_imu_fresh_ || new_outage))
        display_->component("IMU", fresh ? '+' : '!', unavailable ? "DISCONNECTED" : "");
    last_imu_fresh_ = fresh;
    imu_unavailable_ = unavailable;
}
PcbSnapshot HardwareController::get_pcb_snapshot() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto snapshot = pcb_snapshot_;
    if (snapshot.timestamp_s) {
        const double now = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        snapshot.age_s = now - *snapshot.timestamp_s;
    }
    return snapshot;
}
HardwareHealth HardwareController::health() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto sample = imu_->snapshot();
    return {sample.raw_yaw.has_value() && sample.gyro_z.has_value(),
            fault_source_, error_, fault_address_, imu_recovery_generation_};
}

void HardwareController::check_state() const {
    if (!error_.empty()) {
        if (fault_source_ == "MOTOR") throw MotorCommunicationError(error_);
        throw std::runtime_error(error_);
    }
    if (!running_) throw std::runtime_error("HardwareController is stopped; create a new controller");
}
void HardwareController::move(double direction, double speed, double rotation,
                              double rotation_speed, int dribbler, bool kick) {
    for (double value : {direction, speed, rotation, rotation_speed}) finite(value);
    if (std::abs(speed) > 1e6 || dribbler < -1 || dribbler > 1)
        throw std::invalid_argument("Speed out of range or dribbler not in -1..1");
    std::lock_guard<std::mutex> lock(state_mutex_);
    check_state();
    if (kick && !kicker_ && !use_pcb_) throw std::invalid_argument("Kick requested without a configured kicker_pin or PCB");
    // Like kicker.py, ignore requests during an active pulse or the 0.5 s cooldown.
    // Do not queue an old request to fire when the cooldown expires.
    const bool accepted_kick = kick && !kicking_ && std::chrono::steady_clock::now() >= next_kick_time_;
    target_ = {wrap(direction), speed, wrap(rotation), std::clamp(rotation_speed, 0.0, 1.0),
               0, dribbler, accepted_kick};
    if (accepted_kick) wake_.notify_all();
}
void HardwareController::fail(const std::string& message, const std::string& source, int address) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    pcb_snapshot_.valid = false;
    if (error_.find(message) == std::string::npos) {
        if (!error_.empty()) error_ += "; ";
        error_ += message;
    }
    if (fault_source_.empty()) { fault_source_ = source; fault_address_ = address; }
    if (display_) {
        display_->component(source, '!', message);
        display_->set_native_blocked(true);
    }
    running_ = false;
    wake_.notify_all();
}
void HardwareController::set_drive_current_limits(double constant_speed_amps, double acceleration_amps) {
    const auto steady = current_limit_lsb(constant_speed_amps);
    const auto accelerating = current_limit_lsb(acceleration_amps);
    std::lock_guard<std::mutex> lock(state_mutex_);
    check_state();
    constant_speed_current_limit_ = steady;
    acceleration_current_limit_ = accelerating;
}
std::pair<double, double> HardwareController::get_measured_body_velocity_mm_s(double yaw_deg) {
    finite(yaw_deg); // Kept for compatibility; body-frame inversion does not need yaw.
    const auto sample = get_localisation_sample();
    return {sample.vx, sample.vy};
}
HardwareController::LocalisationSample HardwareController::get_localisation_sample() {
    const auto requested = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> bus_lock(wire_->mutex);
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        check_state();
    }
    try {
        const auto started = std::chrono::steady_clock::now();
        std::array<double, 4> rpms;
        for (size_t i = 0; i < rpms.size(); ++i) {
            motor_operation(i, [&] { motors_[i].updateQuickDataReadout(); });
            rpms[i] = motors_[i].getSpeedQDR() / RPM_TO_MOTOR_SPEED;
        }
        const auto ended = std::chrono::steady_clock::now();
        const auto velocity = body_velocity(rpms, config_.diameter);
        const double span = std::chrono::duration<double>(ended-started).count();
        record_timing("odometry", std::chrono::duration<double>(started-requested).count(), span);
        const double timestamp = std::chrono::duration<double>(started.time_since_epoch()).count()+span/2;
        return {velocity.first, velocity.second, timestamp, span, imu_->history()};
    } catch (const std::exception& exc) {
        fail(exc.what());
        throw MotorCommunicationError(exc.what());
    }
}
double HardwareController::get_dribbler_rpm() {
    std::lock_guard<std::mutex> bus_lock(wire_->mutex);
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        check_state();
    }
    if (motors_.size() < 5)
        throw std::runtime_error("No dribbler motor is configured");
    try {
        motor_operation(4, [&] { motors_[4].updateQuickDataReadout(); });
        return motors_[4].getSpeedQDR() / RPM_TO_MOTOR_SPEED;
    } catch (const std::exception& exc) {
        fail(exc.what());
        throw MotorCommunicationError(exc.what());
    }
}
std::string HardwareController::disable_motors() {
    std::string errors;
    for (size_t i = 0; i < motors_.size(); ++i) {
        auto attempt = [&](auto operation) {
            try { operation(); } catch (const std::exception& exc) {
                if (!errors.empty()) errors += "; ";
                errors += "motor " + std::to_string(addresses_[i]) + ": " + exc.what();
            }
        };
        auto& motor = motors_[i];
        attempt([&] { motor.setSpeed(0); });
        attempt([&] { motor.setTorque(0); });
        attempt([&] { motor.configureCommandMode(2); });
        attempt([&] { motor.configureOperatingModeAndSensor(3, 1); });
        attempt([&] { motor.setTorque(0); });
    }
    return errors;
}
void HardwareController::stop() {
    std::lock_guard<std::mutex> stop_lock(stop_mutex_);
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        running_ = false;
        pcb_snapshot_.valid = false;
    }
    wake_.notify_all();
    std::string kicker_errors;
    if (use_pcb_) {
        if (pcb_thread_.joinable()) pcb_thread_.join();
    } else {
        // Join the GPIO worker first: pulse termination must not wait for I2C shutdown.
        if (kicker_thread_.joinable()) kicker_thread_.join();
        if (kicker_) {
            try { kicker_->close(); } catch (const std::exception& exc) { kicker_errors = exc.what(); }
        }
    }
    if (thread_.joinable()) thread_.join();
    if (imu_thread_.joinable()) imu_thread_.join();
    std::lock_guard<std::mutex> bus_lock(wire_->mutex);
    imu_->close();
    const auto errors = disable_motors();
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        dx_ = dy_ = 0;
        target_ = {};
    }
    if (!errors.empty()) fail("Motor shutdown failed: " + errors, "MOTOR");
    if (!kicker_errors.empty()) fail("Kicker shutdown failed: " + kicker_errors, "OTHER");
    if (!errors.empty()) throw MotorCommunicationError("Hardware shutdown failed: " + errors);
    if (!kicker_errors.empty()) throw std::runtime_error("Kicker shutdown failed: " + kicker_errors);
}
double HardwareController::current_speed() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return std::hypot(dx_, dy_);
}
double HardwareController::current_direction() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return std::atan2(dy_, dx_) / RAD;
}
void HardwareController::record_timing(const std::string& name, double wait_s, double work_s) {
    std::lock_guard<std::mutex> lock(timing_mutex_);
    ++timing_[name + "_count"];
    timing_[name + "_wait_s"] += wait_s;
    timing_[name + "_work_s"] += work_s;
    auto& maximum = timing_[name + "_max_wait_s"];
    maximum = std::max(maximum, wait_s);
    auto& work_max = timing_[name + "_max_work_s"];
    work_max = std::max(work_max, work_s);
}
std::map<std::string, double> HardwareController::timing_diagnostics() const {
    const auto imu = imu_->snapshot();
    const double now = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    std::lock_guard<std::mutex> lock(timing_mutex_);
    auto result = timing_;
    result["yaw_count"] = imu.update_count;
    result["gyro_count"] = imu.gyro_count;
    result["yaw_age_s"] = imu.yaw_received_s ? now - imu.yaw_received_s : -1;
    result["gyro_age_s"] = imu.gyro_received_s ? now - imu.gyro_received_s : -1;
    return result;
}
void HardwareController::imu_loop() noexcept {
    while (running_) {
        const auto requested = std::chrono::steady_clock::now();
        try {
            std::lock_guard<std::mutex> bus_lock(wire_->mutex);
            if (!running_) break;
            const auto acquired = std::chrono::steady_clock::now();
            imu_->service();
            record_timing("imu_poll", std::chrono::duration<double>(acquired - requested).count(),
                          std::chrono::duration<double>(std::chrono::steady_clock::now() - acquired).count());
        } catch (const std::exception&) {
            // Report freshness to main.py; the game loop owns the pause policy.
        }
        std::unique_lock<std::mutex> lock(state_mutex_);
        check_imu_locked();
        wake_.wait_for(lock, imu_poll_period_, [&] { return !running_; });
    }
}
void HardwareController::kicker_loop() noexcept {
    try {
        while (running_) {
            std::unique_lock<std::mutex> lock(state_mutex_);
            wake_.wait(lock, [&] { return !running_ || target_.kick; });
            if (!running_) break;
            target_.kick = false; // Consume once; a stale move target must not fire repeatedly.
            kicking_ = true;
            kicker_->high();
            const auto started = std::chrono::steady_clock::now();
            next_kick_time_ = started + kick_cooldown_;
            // wait_until releases state_mutex_; motors, IMU, and move() can proceed.
            // Shutdown interrupts the pulse early, and ordinary move updates do not extend it.
            wake_.wait_until(lock, started + kick_pulse_, [&] { return !running_; });
            lock.unlock();
            kicker_->idle();
            lock.lock();
            kicking_ = false;
        }
    } catch (const std::exception& exc) {
        fail("Kicker GPIO failed: " + std::string(exc.what()), "OTHER");
    }
    // Also attempted after high()/idle() failures and on shutdown during a pulse.
    try { kicker_->idle(); } catch (const std::exception& exc) {
        fail("Kicker shutdown failed: " + std::string(exc.what()), "OTHER");
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    kicking_ = false;
    target_.kick = false;
}
void HardwareController::pcb_loop() noexcept {
    using Clock = std::chrono::steady_clock;
    const auto period = pcb_period_;
    auto next = Clock::now() + period;
    try {
        while (running_) {
            bool kick;
            {
                std::unique_lock<std::mutex> lock(state_mutex_);
                wake_.wait_until(lock, next, [&] { return !running_; });
                if (!running_) break;
                kick = target_.kick;
                target_.kick = false; // Consume each request once.
                if (kick) next_kick_time_ = Clock::now() + kick_cooldown_;
            }
            // Pcb takes the bus mutex; never hold state_mutex_ during I/O.
            if (kick) pcb_->kick();
            const auto requested = Clock::now();
            const auto scan = pcb_->read_sensors();
            record_timing("pcb", 0, std::chrono::duration<double>(Clock::now() - requested).count());
            const double timestamp = std::chrono::duration<double>(
                Clock::now().time_since_epoch()).count();
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                // A stop/fault may have occurred while the read was in flight.
                if (!running_) break;
                pcb_snapshot_ = {scan, timestamp, std::nullopt, true};
            }
            next += period;
            const auto now = Clock::now();
            if (next <= now) next = now + period; // Skip missed polls after a stall.
        }
    } catch (const std::exception& exc) {
        fail("PCB communication failed: " + std::string(exc.what()), "OTHER");
    }
}
void HardwareController::drive_loop() noexcept {
    using Clock = std::chrono::steady_clock; // Measures elapsed time
    const auto period = motor_period_;
    auto last = Clock::now(); // Previous loop time
    auto next = last + period; // Target next loop time
    std::array<double, 4> previous_rpms{};
    int32_t applied_current_limit = drive_motor_current_limit_;
    try {
        while (running_) {
            Command command;
            Clock::time_point deadline;
            int32_t steady_current, accelerating_current;
            {
                // Obtains a lock to avoid this loop and move() racing
                std::unique_lock<std::mutex> lock(state_mutex_);
                // Releases the lock and waits for the next scheduled loop time.
                // If a stop command is called during a wait, the wait will instantly end and the loop will break.
                wake_.wait_until(lock, next, [&] { return !running_; });
                if (!running_) break;

                // Update time variables
                const auto now = Clock::now();
                // Cap dt at 0.04 to avoid large jumps after a stutter
                const double dt = std::clamp(std::chrono::duration<double>(now - last).count(), 0.0, 0.04);
                last = now;
                deadline = next + period;
                next += period;
                if (next <= now) next = now + period; // Account for missed loops

                command = target_; // target_ is the command from move(). Saves it to command so it can be unlocked
                steady_current = constant_speed_current_limit_;
                accelerating_current = acceleration_current_limit_;
                // Calculate required change in velocity vectors (delta refers to change in velocity)
                double delta_x = std::cos(command.direction * RAD) * command.speed - dx_;
                double delta_y = std::sin(command.direction * RAD) * command.speed - dy_;
                const double magnitude = std::hypot(delta_x, delta_y);
                const double step = 8000.0 * dt; // Max acceleration of 8000 mm/s^2
                // Cap acceleration
                if (magnitude > step && magnitude > 0) {
                    delta_x *= step / magnitude;
                    delta_y *= step / magnitude;
                }
                // Update current global velocity
                dx_ += delta_x;
                dy_ += delta_y;
                // Calculate global direction and speed
                command.direction = std::atan2(dy_, dx_) / RAD;
                command.speed = std::hypot(dx_, dy_);
            }
            const auto requested = Clock::now();
            // Acquire lock on i2c bus
            std::lock_guard<std::mutex> bus_lock(wire_->mutex);
            if (!running_) break;
            const auto acquired = Clock::now();
            // Read after acquiring the bus: an IMU operation may have delayed this tick.
            const auto imu = imu_->snapshot();
            command.yaw = imu.last_yaw;
            if (!imu.yaw) command.rotation_speed = 0;
            const auto rpms = calculate_drive_rpms(command, config_);
            // Wheel target changes include translation ramps, braking and yaw corrections.
            bool accelerating = false;
            for (size_t i = 0; i < rpms.size(); ++i)
                accelerating |= std::abs(rpms[i] - previous_rpms[i]) > 0.1;
            const auto current_limit = accelerating ? accelerating_current : steady_current;
            if (current_limit != applied_current_limit) {
                for (size_t i = 0; i < rpms.size(); ++i)
                    motor_operation(i, [&] { motors_[i].setCurrentLimitFOC(current_limit); });
                applied_current_limit = current_limit;
            }
            previous_rpms = rpms;
            // Send motor commands
            for (size_t i = 0; i < rpms.size(); ++i)
                motor_operation(i, [&] { motors_[i].setSpeed(static_cast<int32_t>(rpms[i] * RPM_TO_MOTOR_SPEED)); });
            // Spin the dribbler, if configured
            if (motors_.size() > 4)
                motor_operation(4, [&] { motors_[4].setTorque(command.dribbler * dribbler_motor_current_limit_); });
            ++loop_count_;
            const auto completed = Clock::now();
            record_timing("motor", std::chrono::duration<double>(acquired - requested).count(),
                          std::chrono::duration<double>(completed - acquired).count());
            std::lock_guard<std::mutex> timing_lock(timing_mutex_);
            if (completed > deadline) ++timing_["motor_overruns"];

        }
    } catch (const std::exception& exc) {
        fail(exc.what());
    }
    // Disable motors after shutdown
    std::lock_guard<std::mutex> bus_lock(wire_->mutex);
    const auto errors = disable_motors();
    if (!errors.empty()) fail("Motor shutdown failed: " + errors);
}
} // namespace hardware
