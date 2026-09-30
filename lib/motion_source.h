#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

namespace hardware {
struct MotionHistory {
    std::uint64_t epoch = 0;
    std::vector<std::pair<double, double>> yaw, gyro;
};
struct MotionSample {
    double vx = 0, vy = 0, timestamp_s = 0, read_span_s = 0;
    MotionHistory imu;
    std::uint64_t sequence = 0;
};
class MotionSource {
public:
    struct Batch {
        std::deque<MotionSample> samples;
        bool discontinuity = false, closed = false;
    };
    void publish(MotionSample sample) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_) return;
        sample.sequence = ++count_;
        latest_ = sample;
        if (!claimed_) return;
        if (queue_.size() == 64) {
            queue_.pop_front();
            ++dropped_;
            discontinuity_ = true;
        }
        queue_.push_back(std::move(sample));
    }
    MotionSample latest() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return latest_;
    }
    void claim() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_) throw std::runtime_error("Motion source is closed");
        if (claimed_) throw std::runtime_error("Motion source already has a localisation consumer");
        claimed_ = true;
        queue_.clear();
        discontinuity_ = false;
        if (latest_.timestamp_s > 0) queue_.push_back(latest_);
    }
    void release() {
        std::lock_guard<std::mutex> lock(mutex_);
        claimed_ = false;
        queue_.clear();
    }
    Batch drain() {
        std::lock_guard<std::mutex> lock(mutex_);
        Batch result;
        result.samples.swap(queue_);
        result.discontinuity = discontinuity_;
        result.closed = closed_;
        discontinuity_ = false;
        return result;
    }
    void close() {
        std::lock_guard<std::mutex> lock(mutex_);
        closed_ = true;
    }
    std::uint64_t count() const { std::lock_guard<std::mutex> lock(mutex_); return count_; }
    std::uint64_t dropped() const { std::lock_guard<std::mutex> lock(mutex_); return dropped_; }
    size_t depth() const { std::lock_guard<std::mutex> lock(mutex_); return queue_.size(); }
private:
    mutable std::mutex mutex_;
    MotionSample latest_;
    std::deque<MotionSample> queue_;
    bool claimed_ = false, closed_ = false, discontinuity_ = false;
    std::uint64_t count_ = 0, dropped_ = 0;
};
} // namespace hardware
