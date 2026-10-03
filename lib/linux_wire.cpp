#include "linux_wire.h"

#include <cerrno>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <system_error>
#include <unistd.h>

void TwoWire::beginTransmission(uint8_t address) {
    address_ = address;
    tx_.clear();
}
void TwoWire::write(uint8_t value) { tx_.push_back(value); }
void TwoWire::endTransmission() {
    transfer(address_, false, tx_.data(), tx_.size());
}
void TwoWire::requestFrom(uint8_t address, uint8_t count, uint8_t) {
    rx_.assign(count, 0);
    cursor_ = 0;
    transfer(address, true, rx_.data(), rx_.size());
}
uint8_t TwoWire::read() {
    if (cursor_ >= rx_.size()) throw std::runtime_error("Short motor I2C read");
    return rx_[cursor_++];
}
LinuxWire::LinuxWire(const std::string& device) : fd_(open(device.c_str(), O_RDWR | O_CLOEXEC)) {
    if (fd_ < 0) throw std::system_error(errno, std::generic_category(), "Open " + device);
}
LinuxWire::~LinuxWire() { close(fd_); }
void LinuxWire::transfer(uint8_t address, bool reading, uint8_t* data, size_t size) {
    i2c_msg message{};
    message.addr = address;
    message.flags = reading ? I2C_M_RD : 0;
    message.len = static_cast<__u16>(size);
    message.buf = data;
    i2c_rdwr_ioctl_data transaction{&message, 1};
    const int result = ioctl(fd_, I2C_RDWR, &transaction);
    if (result != 1) {
        throw std::system_error(result < 0 ? errno : EIO, std::generic_category(),
                                "I2C address " + std::to_string(address) +
                                (reading ? " read" : " write"));
    }
}
