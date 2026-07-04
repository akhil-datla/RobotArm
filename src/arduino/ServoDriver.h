// ServoDriver.h — student-facing wrapper around the PCA9685 16-channel PWM board
// (via Adafruit_PWMServoDriver over I2C). This is the one place hardware shows up
// in the component path. Joints bind to it by channel.
//
// ARDUINO ONLY: this header pulls in Adafruit_PWMServoDriver.h / Wire.h, so the
// whole file is guarded by #ifdef ARDUINO. It only ever builds inside the Arduino
// toolchain. See CLAUDE.md sections 4.1, 10 (repo layout), 14.
#pragma once

#ifdef ARDUINO

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <math.h>
#include <stdint.h>

namespace roboarm {

// The PCA9685's nominal oscillator is 25 MHz but often measures ~26-27 MHz. This
// default gets pulse widths close; calibrate per board for best accuracy.
constexpr uint32_t kPca9685OscHz = 27000000UL;
constexpr uint8_t kDefaultPca9685Address = 0x40;
constexpr float kDefaultServoFreqHz = 50.0f;

class ServoDriver {
public:
    // address is the I2C address (0x40 default; set by the board's jumpers).
    // pwmFreqHz is the servo refresh rate (50 Hz for the RDS3225).
    ServoDriver(uint8_t i2cAddress = kDefaultPca9685Address,
                float pwmFreqHz = kDefaultServoFreqHz);

    // Change the address / frequency before begin().
    void configure(uint8_t i2cAddress, float pwmFreqHz);

    // Start I2C and the PCA9685: Wire.begin(), driver.begin(), set the
    // oscillator frequency, then the PWM frequency.
    void begin();

    // Command one channel to a pulse width in microseconds. Used by Pca9685Servo.
    void writeChannelMicroseconds(uint8_t channel, float microseconds);

    // Escape hatch to the underlying Adafruit driver.
    Adafruit_PWMServoDriver& raw() { return m_driver; }

    uint8_t address() const { return m_address; }
    float freqHz() const { return m_freqHz; }

private:
    Adafruit_PWMServoDriver m_driver;
    uint8_t m_address;
    float m_freqHz;
};

}  // namespace roboarm

#endif  // ARDUINO
