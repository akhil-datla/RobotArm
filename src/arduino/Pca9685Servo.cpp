#ifdef ARDUINO

#include "Pca9685Servo.h"

namespace roboarm {

Pca9685Servo::Pca9685Servo() : m_board(nullptr), m_channel(0) {}

Pca9685Servo::Pca9685Servo(ServoDriver& board, uint8_t channel)
    : m_board(&board), m_channel(channel) {}

void Pca9685Servo::attach(ServoDriver& board, uint8_t channel) {
    m_board = &board;
    m_channel = channel;
}

void Pca9685Servo::writeMicroseconds(float microseconds) {
    if (m_board != nullptr) {
        m_board->writeChannelMicroseconds(m_channel, microseconds);
    }
}

}  // namespace roboarm

#endif  // ARDUINO
