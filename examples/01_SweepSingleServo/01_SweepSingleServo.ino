/*
  01_SweepSingleServo — what a "Joint" is.

  A Joint is a servo channel + its calibration + soft limits. You give it an angle
  in degrees and it works out the pulse the servo needs. This sketch builds one
  Joint on a PCA9685 channel and sweeps it back and forth.

  Wiring (see README): PCA9685 SDA/SCL to the Arduino I2C pins, V+ from a separate
  5-6 V servo supply (NOT the Arduino 5 V pin), all grounds tied together, and the
  servo plugged into channel 0.
*/
#include <RobotArm.h>
using namespace roboarm;

// The PCA9685 board at I2C address 0x40, refreshing servos at 50 Hz (RDS3225).
ServoDriver board(0x40, 50);

// One Joint on channel 0.
Joint joint(board, 0);

void setup() {
  Serial.begin(115200);
  board.begin();

  // RDS3225 (180-degree): the full 500-2500 us pulse maps to 0-180 degrees.
  joint.setPulseRange(500, 2500);
  joint.setTravel(180);
  joint.setLimits(0, 180);   // never command outside this range
}

void loop() {
  // Sweep up 0 -> 180 degrees...
  for (int deg = 0; deg <= 180; deg += 2) {
    joint.setAngle(deg);   // set the target
    joint.update();        // push it out to the servo
    Serial.print(F("angle = "));
    Serial.print(deg);
    Serial.print(F(" deg -> pulse = "));
    Serial.print(joint.commandedPulseUs());
    Serial.println(F(" us"));
    delay(15);
  }
  // ...and back down 180 -> 0 degrees.
  for (int deg = 180; deg >= 0; deg -= 2) {
    joint.setAngle(deg);
    joint.update();
    delay(15);
  }
}
