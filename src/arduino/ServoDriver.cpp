#ifdef ARDUINO

#include "ServoDriver.h"

namespace roboarm {

ServoDriver::ServoDriver(uint8_t i2cAddress, float pwmFreqHz)
    : m_driver(i2cAddress), m_address(i2cAddress), m_freqHz(pwmFreqHz) {}

void ServoDriver::configure(uint8_t i2cAddress, float pwmFreqHz) {
    m_address = i2cAddress;
    m_freqHz = pwmFreqHz;
}

void ServoDriver::begin() {
    Wire.begin();
    // (Re)create the driver at the configured address, then bring it up.
    m_driver = Adafruit_PWMServoDriver(m_address);
    m_driver.begin();
    m_driver.setOscillatorFrequency(kPca9685OscHz);
    m_driver.setPWMFreq(m_freqHz);
}

void ServoDriver::writeChannelMicroseconds(uint8_t channel, float microseconds) {
    // Preferred path: the library's calibrated microsecond writer.
    m_driver.writeMicroseconds(channel, static_cast<uint16_t>(lroundf(microseconds)));
    // Fallback for older library versions lacking writeMicroseconds():
    //   uint16_t ticks = lroundf(microseconds * 4096.0f * m_freqHz / 1.0e6f);
    //   m_driver.setPWM(channel, 0, ticks);
}

}  // namespace roboarm

#endif  // ARDUINO
