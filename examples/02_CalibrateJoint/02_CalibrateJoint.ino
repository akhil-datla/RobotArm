/*
  02_CalibrateJoint — the pulse-to-angle mapping (calibration).

  Real servos rarely hit their stops at exactly 500 and 2500 us. Just inside those
  they buzz and stall. Calibration finds each unit's TRUE endpoints and teaches the
  Joint the straight-line map between angle (degrees) and pulse (microseconds).

  How to use this sketch:
    1. Open the Serial Monitor at 115200 baud.
    2. It slowly sweeps the raw pulse from 500 to 2500 us. WATCH the servo. Note the
       pulse where it just reaches each mechanical end without buzzing/straining
       (often ~550 and ~2450).
    3. Put those two numbers into setPulseRange() below and re-upload.
    4. The sketch then commands 90 degrees and prints the pulse — it should be about
       the midpoint (~1500 us) if your endpoints are symmetric.
*/
#include <RobotArm.h>
using namespace roboarm;

ServoDriver board(0x40, 50);
Joint joint(board, 0);

// >>> After watching the sweep, replace these with YOUR servo's true endpoints. <<<
const int kTrueMinUs = 500;
const int kTrueMaxUs = 2500;

void setup() {
  Serial.begin(115200);
  board.begin();

  Serial.println(F("Raw pulse sweep 500 -> 2500 us. Note the true mechanical ends."));
  for (int us = 500; us <= 2500; us += 10) {
    board.writeChannelMicroseconds(0, us);   // command the raw pulse directly
    Serial.print(F("pulse = "));
    Serial.print(us);
    Serial.println(F(" us"));
    delay(40);
  }

  // Teach the Joint the calibrated endpoints and the mounting direction.
  joint.setPulseRange(kTrueMinUs, kTrueMaxUs);
  joint.setTravel(180);
  joint.setLimits(0, 180);
  joint.setDirection(+1);   // flip to -1 if the arm moves the wrong way

  // Confirm the center: 90 degrees should land near the midpoint pulse.
  joint.setAngle(90);
  joint.update();
  Serial.print(F("90 deg -> pulse = "));
  Serial.print(joint.commandedPulseUs());
  Serial.println(F(" us  (expect ~1500 if endpoints are symmetric)"));
}

void loop() {
  // Nudge between 45 and 135 degrees so you can eyeball the calibration.
  joint.setAngle(45);  joint.update();  delay(1000);
  joint.setAngle(135); joint.update();  delay(1000);
}
