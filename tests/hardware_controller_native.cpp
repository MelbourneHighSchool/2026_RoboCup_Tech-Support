// Offline motor protocol, kinematics and lifecycle tests; no Linux device is opened.
#include "hardware_controller.h"
#include "fake_bno08x.h"
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <map>

using namespace hardware;
namespace {
struct Packet { int address; std::vector<uint8_t> bytes; };
struct State {
    State() { imu.auto_reports = true; }
    FakeBno08x imu;
    std::mutex mutex;
    std::vector<Packet> packets;
    std::map<int, int32_t> speed;
    int delay_ms = 0;
    int fail_address = -1, fail_opcode = -1, bad_firmware_address = -1;
    std::atomic<bool> in_transfer{false};
    std::atomic<bool> block_motor{false}, motor_blocked{false};
};
class FakeWire : public TwoWire {
public:
    explicit FakeWire(std::shared_ptr<State> state) : state_(std::move(state)) {}
    void transfer(uint8_t address, bool reading, uint8_t* data, size_t size) override {
        assert(!state_->in_transfer.exchange(true));
        struct EndTransfer {
            std::atomic<bool>& active;
            ~EndTransfer() { active = false; }
        } end{state_->in_transfer};
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (address == 0x4a) {
            state_->imu.transfer(reading, data, size);
            return;
        }
        // Once an IMU header is read, all chunks must finish before any other I/O.
        assert(!state_->imu.peeked);
        if (address == 0x3c) {
            state_->packets.push_back({address, {data, data + size}});
            if (state_->fail_address == address) throw std::runtime_error("OLED unavailable");
            return;
        }
        if (state_->block_motor) {
            state_->motor_blocked = true;
            while (state_->block_motor)
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (reading) {
            std::fill(data, data + size, 0);
            if (size == 4) data[0] = address == state_->bad_firmware_address ? 2 : 3;
            else {
                assert(size == 10);
                uint32_t value = static_cast<uint32_t>(state_->speed[address]);
                for (int i = 0; i < 4; ++i) data[4 + i] = (value >> (8 * i)) & 255;
            }
            return;
        }
        state_->packets.push_back({address, {data, data + size}});
        if (address == state_->fail_address && data[0] == state_->fail_opcode)
            throw std::runtime_error("injected I2C failure");
        if (data[0] == 0x12) {
            assert(size == 5);
            uint32_t value = 0;
            for (int i = 0; i < 4; ++i) value |= uint32_t(data[i + 1]) << (8 * i);
            std::memcpy(&state_->speed[address], &value, 4);
            if (address == 25 && value != 0 && state_->delay_ms)
                std::this_thread::sleep_for(std::chrono::milliseconds(state_->delay_ms));
        }
    }
private:
    std::shared_ptr<State> state_;
};
const DriveConfig config{50, 100, 1000, 3};
std::vector<MotorCalibration> calibration(int count) {
    std::vector<MotorCalibration> result;
    for (int i = 0; i < count; ++i) result.push_back({25 + i, 0xfedcba98u, 2048});
    return result;
}
bool has_packet(const State& state, int address, std::vector<uint8_t> bytes) {
    for (const auto& packet : state.packets)
        if (packet.address == address && packet.bytes == bytes) return true;
    return false;
}
void wait_ticks(HardwareController& controller, uint64_t ticks) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (controller.loop_count() < ticks && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    assert(controller.loop_count() >= ticks);
}
void close(double a, double b, double tolerance = 1e-6) { assert(std::abs(a - b) < tolerance); }
template<class F> void throws(F operation) {
    bool failed = false;
    try { operation(); } catch (const std::exception&) { failed = true; }
    assert(failed);
}
template<class F> void await_condition(F condition) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!condition() && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    assert(condition());
}
void imu_protocol() {
    auto state = std::make_shared<State>();
    state->imu.auto_reports = false;
    FakeWire wire(state);
    LinuxBno08x imu(wire);
    throws([&] { LinuxBno08x duplicate(wire); });
    imu.initialize();
    assert(state->imu.writes.front() == FakeBno08x::Bytes({5, 0, 1, 0, 1}));
    assert(state->imu.feature_count(0x08) == 1);
    assert(state->imu.feature_count(0x02) == 1);
    assert(std::count(state->imu.reads.begin(), state->imu.reads.end(), 32) >= 3);
    assert(!imu.snapshot().raw_yaw);
    imu.set_startup_yaw(10);
    state->imu.yaw_report(-80);
    // Clockwise turn: raw yaw and sensor gyro Z decrease, while the
    // startup-relative heading and the exported angular velocity increase.
    state->imu.gyro_report(-1);
    imu.service();
    auto sample = imu.snapshot();
    close(*sample.raw_yaw, -80, 0.01);
    close(*sample.yaw, 90, 0.01);
    close(*sample.gyro_z, 180 / std::acos(-1));
    close((*sample.quaternion)[2], std::sin(-40 * std::acos(-1) / 180), 1.0/16384);
    assert(sample.update_count == 1);
    // A failed payload read must never publish a partial quaternion.
    state->imu.yaw_report(50);
    state->imu.fail_payload = true;
    throws([&] { imu.service(); });
    assert(imu.snapshot().update_count == 1);
    state->imu.yaw_report(-179);
    imu.service();
    imu.set_startup_yaw(179);
    close(*imu.snapshot().yaw, -2, 0.01);

    std::this_thread::sleep_for(std::chrono::milliseconds(110));
    // Anticlockwise sensor rate must yield negative project angular velocity.
    state->imu.gyro_report(1);
    imu.service();
    sample = imu.snapshot();
    assert(!sample.yaw && !sample.raw_yaw && !sample.quaternion);
    close(*sample.gyro_z, -180 / std::acos(-1)); // Gyro freshness is independent.
    close(sample.last_yaw, -2, 0.01);

    state->imu.reset();
    imu.service();
    sample = imu.snapshot();
    assert(!sample.yaw && !sample.gyro_z);
    assert(state->imu.feature_count(0x08) == 2 && state->imu.feature_count(0x02) == 2);
    state->imu.yaw_report(179);
    imu.service();
    close(*imu.snapshot().yaw, 0, 0.01);

    // Reject oversized SHTP transfers before reading a potentially huge payload.
    state->imu.queue(3, FakeBno08x::Bytes(400, 0));
    const auto reads = state->imu.reads.size();
    throws([&] { imu.service(); });
    assert(state->imu.reads.size() == reads + 1);
    state->imu.packets.clear();
    state->imu.peeked = false;
    state->imu.yaw_report(179);
    imu.service();
    assert(imu.snapshot().yaw);
    imu.close();
    imu.close();
    assert(!imu.snapshot().yaw);
    LinuxBno08x replacement(wire, 0x4a, 20);
    replacement.initialize();
    // Clear earlier 10 ms commands before checking the new report interval.
    for (size_t i = state->imu.writes.size() - 2; i < state->imu.writes.size(); ++i) {
        const auto& command = state->imu.writes[i];
        assert(command[9] == 0x20 && command[10] == 0x4e); // 20000 us
    }
}
void native_yaw_control() {
    auto state = std::make_shared<State>();
    state->imu.auto_reports = true;
    HardwareController controller(calibration(5), config, "unused", std::make_unique<FakeWire>(state));
    controller.set_startup_yaw(0);
    await_condition([&] { return controller.health().imu_healthy; });
    controller.set_startup_yaw(0);
    await_condition([&] { return controller.get_yaw().has_value(); });
    // One movement request: subsequent yaw changes must affect the motors without move().
    controller.move(0, 500, 90, 1, 1);
    auto common_rpm = [&] {
        std::lock_guard<std::mutex> lock(state->mutex);
        double sum = 0;
        for (int address = 25; address <= 28; ++address) sum += state->speed[address];
        return sum / (4 * RPM_TO_MOTOR_SPEED);
    };
    await_condition([&] { return std::abs(common_rpm() + 100) < 0.01; });
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->imu.yaw = -60;
    }
    await_condition([&] { return std::abs(common_rpm() + 50) < 0.05 &&
                                 controller.get_yaw().value_or(0) > 59; });
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->imu.fail_read = true;
    }
    await_condition([&] { return !controller.get_yaw(); });
    const auto updates = controller.imu_update_count();
    wait_ticks(controller, controller.loop_count() + 2);
    assert(!controller.get_raw_imu_yaw() && !controller.get_latest_quaternion());
    assert(controller.imu_update_count() == updates);
    close(common_rpm(), 0, 0.01);
    close(controller.current_speed(), 500); // Health reporting alone does not replace commands.
    assert(!controller.health().imu_healthy);
    assert(controller.health().imu_recovery_generation > 0);
    controller.move(0, 0, 0, 0, 0); // Same command main.py sends while paused.
    wait_ticks(controller, controller.loop_count() + 5);
    close(controller.current_speed(), 0);
    const auto velocity = controller.get_measured_body_velocity_mm_s(0);
    close(velocity.first, 0); close(velocity.second, 0);
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        assert(has_packet(*state, 29, {0x11, 0, 0, 0, 0}));
        state->imu.fail_read = false;
        state->imu.yaw = 0;
    }
    await_condition([&] { return controller.health().imu_healthy; });
    wait_ticks(controller, controller.loop_count() + 2);
    close(controller.current_speed(), 0); // Last target remains the normal paused command.
    controller.move(0, 500, 90, 1, 1);
    await_condition([&] { return std::abs(common_rpm() + 100) < 0.01; });
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->imu.reset();
    }
    await_condition([&] {
        std::lock_guard<std::mutex> lock(state->mutex);
        return state->imu.feature_count(0x08) == 2 && state->imu.feature_count(0x02) == 2;
    });
    await_condition([&] { return controller.get_yaw().has_value(); });
    controller.stop();
    assert(!controller.get_yaw());
}
void imu_initialization_failures() {
    for (int failure = 0; failure < 3; ++failure) {
        auto state = std::make_shared<State>();
        state->imu.omit_product = failure == 0;
        state->imu.fail_feature = failure == 1;
        state->imu.omit_reset = failure == 2;
        throws([&] {
            HardwareController controller(calibration(5), config, "unused", std::make_unique<FakeWire>(state));
        });
        for (int i = 0; i < 5; ++i) {
            assert(state->speed[25+i] == 0);
            assert(has_packet(*state, 25+i, {0x11, 0, 0, 0, 0}));
        }
    }
}
void kinematics() {
    for (double yaw : {-170., 0., 90., 179.}) {
        for (double direction : {-180., -90., 0., 45., 90.}) {
            auto rpms = calculate_drive_rpms({direction, 500, yaw, 1, yaw, 0}, config);
            auto velocity = body_velocity(rpms, 50);
            const double angle = (direction - yaw) * std::acos(-1) / 180;
            close(velocity.first, 500 * std::cos(angle));
            close(velocity.second, -500 * std::sin(angle));
            // A common rotation component must not change translation odometry.
            for (auto& rpm : rpms) rpm -= 80;
            const auto rotated = body_velocity(rpms, 50);
            close(rotated.first, velocity.first);
            close(rotated.second, velocity.second);
        }
    }
    const auto clockwise = calculate_drive_rpms({0, 0, 90, 1, 0, 0}, config);
    for (double rpm : clockwise) close(rpm, -100);
    const auto wrap = calculate_drive_rpms({0, 0, -179, 1, 179, 0}, {50,100,1000,0});
    for (double rpm : wrap) close(rpm, -100.0 / 30);
    const auto saturated = calculate_drive_rpms({0, 10000, 90, 1, 0, 0}, config);
    for (double rpm : saturated) assert(std::abs(rpm) <= 1000);
    close(saturated[0], -1000);
    close(saturated[1], 800);
}
void lifecycle(int count) {
    auto state = std::make_shared<State>();
    HardwareController controller(calibration(count), config, "unused", std::make_unique<FakeWire>(state),
                                  0x4a, 10, -1, "", nullptr, 2.5, 0.75);
    controller.set_startup_yaw(0);
    await_condition([&] { return controller.health().imu_healthy; });
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        for (int i = 0; i < count; ++i) {
            assert(has_packet(*state, 25+i, {0x30, 0x98, 0xba, 0xdc, 0xfe}));
            assert(has_packet(*state, 25+i, {0x32, 0, 8, 0, 0}));
            assert(has_packet(*state, 25+i, {0x21, uint8_t(i < 4 ? 12 : 2)}));
            if (i < 4)
                assert(has_packet(*state, 25+i, {0x33, 0, 0x80, 2, 0}));
            else
                assert(has_packet(*state, 25+i, {0x33, 0, 0xc0, 0, 0}));
        }
        // Float PID values must be serialized as IEEE bits, not converted to integers.
        assert(has_packet(*state, 25, {0x43, 0, 0x80, 0x89, 0x43, 0, 0, 0, 0, 0, 0, 0, 0}));
    }
    controller.move(90, 500, 0, 0, 1);
    wait_ticks(controller, 6); // Native loop continues without further move() calls.
    close(controller.current_speed(), 500);
    auto velocity = controller.get_measured_body_velocity_mm_s(137);
    close(velocity.first, 0, 1e-4);
    close(velocity.second, -500, 1e-4); // Negative QDR register decoded correctly.
    if (count == 5) {
        std::lock_guard<std::mutex> lock(state->mutex);
        assert(has_packet(*state, 29, {0x11, 0, 0xc0, 0, 0}));
    }
    throws([&] { controller.move(0, std::numeric_limits<double>::quiet_NaN(), 0, 0, 0); });
    throws([&] { controller.move(0, 0, 0, 0, 0, true); });
    controller.stop();
    const auto ticks = controller.loop_count();
    controller.stop();
    close(controller.current_speed(), 0);
    throws([&] { controller.move(0, 500, 0, 0, 0); });
    throws([&] { controller.get_measured_body_velocity_mm_s(0); });
    assert(controller.loop_count() == ticks);
    std::lock_guard<std::mutex> lock(state->mutex);
    for (int i = 0; i < count; ++i) {
        assert(state->speed[25+i] == 0);
        assert(has_packet(*state, 25+i, {0x11, 0, 0, 0, 0}));
    }
}
struct KickState {
    std::mutex mutex;
    std::condition_variable changed;
    std::vector<std::chrono::steady_clock::time_point> starts, ends;
    bool active = false, closed = false, fail_high = false;
};
class FakeKicker : public KickerOutput {
public:
    explicit FakeKicker(std::shared_ptr<KickState> state) : state_(state) {}
    void high() override {
        std::lock_guard<std::mutex> lock(state_->mutex);
        assert(!state_->active && !state_->closed);
        state_->active = true;
        state_->starts.push_back(std::chrono::steady_clock::now());
        state_->changed.notify_all();
        if (state_->fail_high) throw std::runtime_error("injected GPIO failure");
    }
    void idle() override {
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (state_->active) state_->ends.push_back(std::chrono::steady_clock::now());
        state_->active = false;
        state_->changed.notify_all();
    }
    void close() override {
        idle();
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->closed = true;
    }
private:
    std::shared_ptr<KickState> state_;
};
void kicker_control() {
    auto state = std::make_shared<State>();
    auto gpio = std::make_shared<KickState>();
    HardwareController controller(calibration(4), config, "unused",
        std::make_unique<FakeWire>(state), 0x4a, 10, -1, "",
        std::make_unique<FakeKicker>(gpio), 8.0, 1.0, 0.01, 0.1);
    controller.set_startup_yaw(0);
    await_condition([&] { return controller.health().imu_healthy; });
    auto wait = [&](size_t count, bool ended) {
        std::unique_lock<std::mutex> lock(gpio->mutex);
        assert(gpio->changed.wait_for(lock, std::chrono::seconds(2), [&] {
            return (ended ? gpio->ends.size() : gpio->starts.size()) >= count;
        }));
    };
    state->block_motor = true;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!state->motor_blocked && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    assert(state->motor_blocked);
    controller.move(0, 500, 0, 0, 0, true);
    wait(1, false);
    controller.move(0, 500, 0, 0, 0, true); // During pulse: ignored.
    wait(1, true);
    state->block_motor = false; // Pulse completed while the I2C transaction was blocked.
    controller.move(0, 500, 0, 0, 0, true); // During cooldown: ignored.
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    {
        std::lock_guard<std::mutex> lock(gpio->mutex);
        assert(gpio->starts.size() == 1); // No queued or stale-target repeat.
        assert(gpio->ends[0] - gpio->starts[0] >= std::chrono::milliseconds(10));
    }
    assert(controller.loop_count() > 10); // Driving continues independently.
    controller.move(0, 0, 0, 0, 0, true);
    wait(2, false);
    controller.stop(); // Interrupt the active pulse and release GPIO.
    controller.stop();
    {
        std::lock_guard<std::mutex> lock(gpio->mutex);
        assert(!gpio->active && gpio->closed && gpio->ends.size() == 2);
    }
}
void kicker_failure() {
    auto state = std::make_shared<State>();
    auto gpio = std::make_shared<KickState>();
    gpio->fail_high = true;
    HardwareController controller(calibration(4), config, "unused",
        std::make_unique<FakeWire>(state), 0x4a, 10, -1, "",
        std::make_unique<FakeKicker>(gpio));
    controller.set_startup_yaw(0);
    await_condition([&] { return controller.health().imu_healthy; });
    controller.move(0, 0, 0, 0, 0, true);
    {
        std::unique_lock<std::mutex> lock(gpio->mutex);
        assert(gpio->changed.wait_for(lock, std::chrono::seconds(2), [&] {
            return !gpio->ends.empty();
        }));
    }
    throws([&] { controller.move(0, 0, 0, 0); });
    controller.stop();
    assert(gpio->closed && !gpio->active);
}
void kicker_initialization_cleanup() {
    auto state = std::make_shared<State>();
    state->bad_firmware_address = 26;
    auto gpio = std::make_shared<KickState>();
    throws([&] {
        HardwareController controller(calibration(4), config, "unused",
            std::make_unique<FakeWire>(state), 0x4a, 10, -1, "",
            std::make_unique<FakeKicker>(gpio));
    });
    assert(gpio->closed && !gpio->active && gpio->starts.empty());
}
void capped_acceleration() {
    auto state = std::make_shared<State>();
    state->delay_ms = 100; // Simulate slow I2C; the ramp must still cap dt at 40 ms.
    HardwareController controller(calibration(4), config, "unused", std::make_unique<FakeWire>(state));
    controller.move(0, 1800, 0, 0, 0);
    wait_ticks(controller, 8);
    controller.stop();
    double previous = 0;
    bool reached_target = false;
    for (const auto& packet : state->packets) {
        if (packet.address != 25 || packet.bytes[0] != 0x12) continue;
        uint32_t raw = 0;
        for (int i = 0; i < 4; ++i) raw |= uint32_t(packet.bytes[i+1]) << (8*i);
        int32_t units;
        std::memcpy(&units, &raw, 4);
        const double speed = -units / RPM_TO_MOTOR_SPEED * 50 * std::acos(-1) / 60 * std::sqrt(2.0);
        if (speed == 0) continue;
        assert(speed - previous <= 320.001);
        previous = speed;
        if (std::abs(speed - 1800) < 0.001) reached_target = true;
    }
    assert(reached_target);
}
void failures() {
    auto state = std::make_shared<State>();
    state->bad_firmware_address = 26;
    throws([&] {
        HardwareController controller(calibration(5), config, "unused", std::make_unique<FakeWire>(state));
    });
    for (int i = 0; i < 5; ++i) assert(has_packet(*state, 25+i, {0x21, 2}));
    state = std::make_shared<State>();
    HardwareController controller(calibration(5), config, "unused", std::make_unique<FakeWire>(state));
    controller.set_startup_yaw(0);
    await_condition([&] { return controller.health().imu_healthy; });
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->fail_address = 25;
        state->fail_opcode = 0x12;
    }
    // Wait until the background thread latches the error.
    bool failed = false;
    for (int i = 0; i < 100 && !failed; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        try { controller.move(0, 500, 0, 0, 0); }
        catch (const MotorCommunicationError&) { failed = true; }
    }
    assert(failed);
    assert(controller.health().fault_source == "MOTOR");
    assert(controller.health().motor_address == 25);
    throws([&] { controller.move(0, 0, 0, 0, 0); }); // Fault remains latched.
    throws([&] { controller.stop(); }); // Shutdown failures are reported.
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        assert(has_packet(*state, 29, {0x11, 0, 0, 0, 0})); // Failure didn't skip other motors.
        state->fail_address = -1;
    }
    controller.stop(); // Retrying shutdown after a transient bus failure works.
    auto invalid = calibration(4);
    invalid[1].address = invalid[0].address;
    throws([&] { HardwareController bad(invalid, config, "/no/hardware"); });
}
}
void dynamic_current_limits() {
    auto state = std::make_shared<State>();
    HardwareController controller(calibration(5), config, "unused", std::make_unique<FakeWire>(state));
    controller.set_startup_yaw(0);
    await_condition([&] { return controller.health().imu_healthy; });
    controller.set_drive_current_limits(0.75, 2.5);
    throws([&] { controller.set_drive_current_limits(-1, 2); });
    wait_ticks(controller, 2);
    controller.move(0, 500, 0, 0);
    wait_ticks(controller, 12);
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        for (int address = 25; address < 29; ++address) {
            std::vector<std::vector<uint8_t>> limits;
            for (const auto& packet : state->packets)
                if (packet.address == address && packet.bytes[0] == 0x33)
                    limits.push_back(packet.bytes);
            assert(limits.size() == 4); // Initial, stationary, accelerating, cruising; no repeated writes.
            assert(limits[1] == std::vector<uint8_t>({0x33, 0, 0xc0, 0, 0}));
            assert(limits[2] == std::vector<uint8_t>({0x33, 0, 0x80, 2, 0}));
            assert(limits[3] == limits[1]);
        }
        assert(!has_packet(*state, 29, {0x33, 0, 0x80, 2, 0}));
    }
    controller.stop();
    throws([&] { controller.set_drive_current_limits(1, 2); });
}
void shared_display_lifecycle() {
    auto state = std::make_shared<State>();
    auto wire = std::make_shared<FakeWire>(state);
    auto display = std::make_shared<StatusDisplay>("unused", 0x3c, wire);
    HardwareController controller(calibration(4), config, "unused", nullptr,
                                  0x4a, 10, -1, "", nullptr, 8, 1, 0.02, 0.5, display);
    controller.set_startup_yaw(0);
    await_condition([&] { return controller.health().imu_healthy; });
    display->update("STRIKER", true, "RUNNING");
    controller.move(0, 500, 0, 0);
    wait_ticks(controller, controller.loop_count() + 5);
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->fail_address = 0x3c;
    }
    display->component("OTHER", '!', "DISPLAY TEST");
    await_condition([&] { return !display->error().empty(); });
    assert(controller.health().error.empty());
    assert(controller.health().imu_healthy);
    controller.stop();
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->fail_address = -1;
    }
    display->update("STRIKER", false, "PAUSED"); // Still usable after controller stop.
    display->stop();
    std::lock_guard<std::mutex> lock(state->mutex);
    assert(has_packet(*state, 0x3c, {0, 0xae}));
}
void imu_pause_finishes_kicker() {
    auto state = std::make_shared<State>();
    auto gpio = std::make_shared<KickState>();
    HardwareController controller(calibration(5), config, "unused",
        std::make_unique<FakeWire>(state), 0x4a, 10, -1, "",
        std::make_unique<FakeKicker>(gpio), 8, 1, 0.4, 0.1);
    controller.set_startup_yaw(0);
    await_condition([&] { return controller.health().imu_healthy; });
    controller.move(0, 500, 0, 0, 1, true);
    await_condition([&] { std::lock_guard<std::mutex> lock(gpio->mutex); return gpio->active; });
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->imu.gyro_enabled = false; // Report loss even if yaw still arrives.
    }
    await_condition([&] { return !controller.health().imu_healthy; });
    controller.move(0, 0, 0, 0, 0); // Ordinary pause; leave the active pulse alone.
    {
        std::lock_guard<std::mutex> lock(gpio->mutex);
        assert(gpio->active);
    }
    await_condition([&] { std::lock_guard<std::mutex> lock(gpio->mutex); return !gpio->active; });
    {
        std::lock_guard<std::mutex> lock(gpio->mutex);
        assert(gpio->ends[0] - gpio->starts[0] >= std::chrono::milliseconds(400));
    }
    wait_ticks(controller, controller.loop_count() + 2);
    close(controller.current_speed(), 0);
    assert(controller.get_yaw());
    assert(!controller.get_gyro_z_deg_s());
    controller.stop();
}
int main() {
    shared_display_lifecycle();
    imu_pause_finishes_kicker();
    dynamic_current_limits();
    imu_protocol();
    kinematics();
    lifecycle(4);
    lifecycle(5);
    capped_acceleration();
    failures();
    kicker_control();
    kicker_failure();
    kicker_initialization_cleanup();
    native_yaw_control();
    imu_initialization_failures();
    std::cout << "Native hardware controller tests passed\n";
}
