#include "status_display.h"
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <stdexcept>

namespace hardware {
namespace {
// Five columns, seven pixels high. Uppercase keeps the small OLED legible.
const std::map<char, std::array<uint8_t, 5>> font = {
    {'A',{126,9,9,9,126}}, {'B',{127,73,73,73,54}}, {'C',{62,65,65,65,34}},
    {'D',{127,65,65,34,28}}, {'E',{127,73,73,73,65}}, {'F',{127,9,9,9,1}},
    {'G',{62,65,73,73,122}}, {'H',{127,8,8,8,127}}, {'I',{65,65,127,65,65}},
    {'J',{32,64,65,63,1}}, {'K',{127,8,20,34,65}}, {'L',{127,64,64,64,64}},
    {'M',{127,2,12,2,127}}, {'N',{127,4,8,16,127}}, {'O',{62,65,65,65,62}},
    {'P',{127,9,9,9,6}}, {'Q',{62,65,81,33,94}}, {'R',{127,9,25,41,70}},
    {'S',{38,73,73,73,50}}, {'T',{1,1,127,1,1}}, {'U',{63,64,64,64,63}},
    {'V',{31,32,64,32,31}}, {'W',{63,64,56,64,63}}, {'X',{99,20,8,20,99}},
    {'Y',{3,4,120,4,3}}, {'Z',{97,81,73,69,67}},
    {'0',{62,81,73,69,62}}, {'1',{0,66,127,64,0}}, {'2',{98,81,73,73,70}},
    {'3',{34,65,73,73,54}}, {'4',{24,20,18,127,16}}, {'5',{39,69,69,69,57}},
    {'6',{60,74,73,73,48}}, {'7',{1,113,9,5,3}}, {'8',{54,73,73,73,54}},
    {'9',{6,73,73,41,30}}, {' ',{0,0,0,0,0}}, {'!',{0,0,95,0,0}},
    {'?',{2,1,81,9,6}}, {'+',{8,8,62,8,8}}, {'-',{8,8,8,8,8}},
    {':',{0,0,36,0,0}}, {'/',{32,16,8,4,2}}, {'.',{0,96,96,0,0}},
    {'=',{20,20,20,20,20}}, {'_',{64,64,64,64,64}}
};
void text(DisplayFrame& frame, int x, int y, const std::string& value, int scale = 1) {
    for (unsigned char ch : value) {
        auto glyph = font.find(static_cast<char>(std::toupper(ch)));
        if (glyph == font.end()) glyph = font.find('?');
        if (x + 5 * scale > 128) break;
        for (int c = 0; c < 5; ++c)
            for (int r = 0; r < 7; ++r)
                if (glyph->second[c] & (1 << r))
                    for (int dx = 0; dx < scale; ++dx)
                        for (int dy = 0; dy < scale; ++dy) {
                            const int px = x + c * scale + dx, py = y + r * scale + dy;
                            if (py < 64) frame[(py / 8) * 128 + px] |= 1 << (py % 8);
                        }
        x += 6 * scale;
    }
}
}
DisplayFrame render_status(const DisplayStatus& s, size_t page) {
    DisplayFrame frame{};
    text(frame, 0, 0, s.mode, 2);
    text(frame, 0, 17, std::string("RUN ") + (s.run ? "ON " : "OFF ") +
         (s.run && s.native_blocked ? "BLOCKED" : s.state));
    std::string health;
    for (const auto& source : {"LIDAR", "CAMERA", "IMU", "MOTOR"}) {
        auto entry = s.health.find(source);
        health += std::string(1, source[0]) + ":" +
                  (entry == s.health.end() ? '?' : entry->second) + " ";
    }
    text(frame, 0, 27, health);
    if (s.errors.empty()) {
        const bool ready = std::all_of(s.health.begin(), s.health.end(),
            [](const auto& item) { return item.second == '+'; }) && s.health.size() >= 4;
        text(frame, 0, 39, s.state == "STARTING" ? "INITIALIZING" :
             (ready ? "ALL SYSTEMS OK" : "WAITING FOR DATA"));
        text(frame, 0, 51, s.detail);
    } else {
        auto item = s.errors.begin();
        const size_t index = page % s.errors.size();
        std::advance(item, index);
        text(frame, 0, 37, "ERR " + std::to_string(index + 1) + "/" +
             std::to_string(s.errors.size()) + " " + item->first);
        // Two 21-character rows; long details scroll in overlapping windows.
        const size_t offset = item->second.size() > 42 ?
            ((page / s.errors.size()) * 21) % item->second.size() : 0;
        const auto detail = item->second.substr(offset, 42);
        text(frame, 0, 47, detail.substr(0, 21));
        if (detail.size() > 21) text(frame, 0, 56, detail.substr(21));
    }
    return frame;
}
StatusDisplay::StatusDisplay(const std::string& device, int address,
                             std::shared_ptr<TwoWire> transport, bool start_worker) {
    if (address < 8 || address > 119) throw std::invalid_argument("Invalid display address");
    address_ = address;
    wire_ = transport ? std::move(transport) : std::make_shared<LinuxWire>(device);
    if (start_worker) thread_ = std::thread(&StatusDisplay::worker, this);
}
StatusDisplay::~StatusDisplay() { stop(); }
void StatusDisplay::update(const std::string& mode, bool run, const std::string& state,
                           const std::string& detail) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.mode = mode; status_.run = run; status_.state = state; status_.detail = detail;
}
void StatusDisplay::component(const std::string& source, char health, const std::string& error) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.health[source] = health;
    if (error.empty()) status_.errors.erase(source);
    else status_.errors[source] = error;
}
void StatusDisplay::set_native_blocked(bool blocked) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.native_blocked = blocked;
}
std::string StatusDisplay::error() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return error_;
}
void StatusDisplay::commands(std::initializer_list<uint8_t> bytes) {
    wire_->beginTransmission(address_); wire_->write(0x00);
    for (auto byte : bytes) wire_->write(byte);
    wire_->endTransmission();
}
bool StatusDisplay::refresh(size_t page) {
    DisplayStatus snapshot;
    { std::lock_guard<std::mutex> lock(mutex_); snapshot = status_; }
    auto frame = render_status(snapshot, page);
    try {
        if (!initialized_) {
            std::unique_lock<std::mutex> lock(wire_->mutex, std::try_to_lock);
            if (!lock) return false;
            // SSD1306 128x64, internal charge pump, horizontal addressing.
            // Solomon Systech datasheet: https://www.sunrom.com/download/SSD1306.pdf
            commands({0xae,0xd5,0x80,0xa8,0x3f,0xd3,0x00,0x40,0x8d,0x14,
                      0x20,0x00,0xa1,0xc8,0xda,0x12,0x81,0x7f,0xd9,0xf1,
                      0xdb,0x40,0xa4,0xa6,0x2e,0xaf});
            initialized_ = true;
            valid_.fill(false);
        }
        for (size_t chunk = 0; chunk < 64; ++chunk) {
            const size_t offset = chunk * 16;
            if (valid_[chunk] && std::equal(frame.begin() + offset,
                    frame.begin() + offset + 16, previous_.begin() + offset)) continue;
            std::unique_lock<std::mutex> lock(wire_->mutex, std::try_to_lock);
            if (!lock) return false;
            const uint8_t col = offset % 128, row = offset / 128;
            commands({0x21,col,static_cast<uint8_t>(col+15),0x22,row,row});
            wire_->beginTransmission(address_); wire_->write(0x40);
            for (size_t i = offset; i < offset + 16; ++i) wire_->write(frame[i]);
            wire_->endTransmission();
            std::copy_n(frame.begin() + offset, 16, previous_.begin() + offset);
            valid_[chunk] = true;
            lock.unlock();
            // Give waiting motor/IMU workers a scheduling opportunity before
            // the next chunk, including on a 100 kHz bus.
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        std::lock_guard<std::mutex> lock(mutex_); error_.clear();
        return true;
    } catch (const std::exception& exc) {
        initialized_ = false;
        std::lock_guard<std::mutex> lock(mutex_);
        if (error_ != exc.what()) std::fprintf(stderr, "OLED unavailable: %s\n", exc.what());
        error_ = exc.what();
        return false;
    }
}
void StatusDisplay::worker() noexcept {
    const auto start = std::chrono::steady_clock::now();
    while (true) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopped_) return;
        }
        const auto elapsed = std::chrono::steady_clock::now() - start;
        refresh(std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() / 2);
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait_for(lock, error_.empty() ? std::chrono::milliseconds(200) :
                       std::chrono::milliseconds(2000), [&] { return stopped_; });
    }
}
void StatusDisplay::stop() {
    std::lock_guard<std::mutex> stop_lock(stop_mutex_);
    { std::lock_guard<std::mutex> lock(mutex_); if (stopped_) return; stopped_ = true; }
    wake_.notify_all();
    if (thread_.joinable()) thread_.join();
    try {
        std::lock_guard<std::mutex> lock(wire_->mutex);
        commands({0xae});
        for (uint8_t row = 0; row < 8; ++row) {
            commands({0x21,0,127,0x22,row,row});
            for (int chunk = 0; chunk < 8; ++chunk) {
                wire_->beginTransmission(address_); wire_->write(0x40);
                for (int i = 0; i < 16; ++i) wire_->write(0);
                wire_->endTransmission();
            }
        }
        commands({0x8d,0x10});
    } catch (const std::exception& exc) {
        std::fprintf(stderr, "OLED shutdown failed: %s\n", exc.what());
    }
}
}
