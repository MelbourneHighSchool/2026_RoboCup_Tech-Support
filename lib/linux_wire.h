#pragma once

#include <cstdint>
#include <string>
#include <mutex>
#include <vector>

// Minimal Wire interface used by PowerfulBLDCdriver. The owner serializes access.
// Each Linux I2C_RDWR message carries its own address; no shared I2C_SLAVE state.
class TwoWire {
public:
    virtual ~TwoWire() = default;
    std::mutex mutex; // Serialize complete register transactions across native owners.
    void beginTransmission(uint8_t address);
    void write(uint8_t value);
    void endTransmission();
    void requestFrom(uint8_t address, uint8_t count, uint8_t stop);
    uint8_t read();
protected:
    virtual void transfer(uint8_t address, bool reading, uint8_t* data, size_t size) = 0;
private:
    uint8_t address_ = 0;
    std::vector<uint8_t> tx_, rx_;
    size_t cursor_ = 0;
};

class LinuxWire final : public TwoWire {
public:
    explicit LinuxWire(const std::string& device);
    ~LinuxWire() override;
    LinuxWire(const LinuxWire&) = delete;
    LinuxWire& operator=(const LinuxWire&) = delete;
protected:
    void transfer(uint8_t address, bool reading, uint8_t* data, size_t size) override;
private:
    int fd_;
};
