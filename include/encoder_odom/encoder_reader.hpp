#ifndef ENCODER_READER_HPP_
#define ENCODER_READER_HPP_

#include <memory>
#include <pigpiod_if2.h>
#include "rclcpp/rclcpp.hpp"

class EncoderReader {
public:
    EncoderReader(int pi, int pinA, int pinB, double ticksPerRevolution);
    ~EncoderReader();
    int getCount() const;
    void reset();
    static void callback(int pi, unsigned int gpio, unsigned int level, uint32_t tick, void *user);

private:
    int pi_;
    int pinA_;
    int pinB_;
    double ticksPerRevolution_;
    volatile int count_;
    static constexpr int DEBOUNCE_US = 100;  // デバウンス時間(μs)
};

#endif  // ENCODER_READER_HPP_