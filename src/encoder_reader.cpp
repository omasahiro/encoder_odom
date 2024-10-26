#include "encoder_odom/encoder_reader.hpp"

EncoderReader::EncoderReader(int pi, int pinA, int pinB, double ticksPerRevolution)
    : pi_(pi), pinA_(pinA), pinB_(pinB), ticksPerRevolution_(ticksPerRevolution), count_(0) {
    
    set_mode(pi_, pinA_, PI_INPUT);
    set_mode(pi_, pinB_, PI_INPUT);
    set_pull_up_down(pi_, pinA_, PI_PUD_UP);
    set_pull_up_down(pi_, pinB_, PI_PUD_UP);
    
    callback_ex(pi_, pinA_, EITHER_EDGE, callback, this);
}

EncoderReader::~EncoderReader() {
    set_mode(pi_, pinA_, PI_INPUT);
    set_mode(pi_, pinB_, PI_INPUT);
}

void EncoderReader::callback(int pi, unsigned int gpio, unsigned int level, uint32_t tick, void *user) {
    EncoderReader* encoder = static_cast<EncoderReader*>(user);
    static uint32_t last_tick = 0;
    
    if (tick - last_tick < encoder->DEBOUNCE_US) {
        return;
    }
    last_tick = tick;

    int stateA = gpio_read(pi, encoder->pinA_);
    int stateB = gpio_read(pi, encoder->pinB_);

    if (stateA == stateB) {
        encoder->count_++;
    } else {
        encoder->count_--;
    }
}

int EncoderReader::getCount() const {
    return count_;
}

void EncoderReader::reset() {
    count_ = 0;
}