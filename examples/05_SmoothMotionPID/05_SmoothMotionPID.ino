/*
  05_SmoothMotionPID — motion shaping and the control loop, in your own hands.

  Part 1: drive a Joint straight to setpoints — it snaps there.
  Part 2: add a SlewRateLimiter so the same setpoints are approached smoothly.
  Part 3: run a PIDController YOURSELF — call pid.calculate(setpoint, measurement,
          dt) and see the effort it produces. With a feedback pot on an analog pin
          you get a true closed loop; without one we simulate the measurement so
          you can still watch the numbers converge.
*/
#include <RobotArm.h>
using namespace roboarm;

ServoDriver board(0x40, 50);
Joint joint(board, 0);

// Motion shaping: at most 90 deg/second of change.
SlewRateLimiter smoothing(90);

// A PID you call yourself. Gains are a gentle starting point — tune for your arm.
PIDController pid(2.0, 0.5, 0.05);

uint32_t lastMicros = 0;

void setup() {
  Serial.begin(115200);
  board.begin();
  joint.setPulseRange(500, 2500);
  joint.setTravel(180);
  joint.setLimits(0, 180);

  pid.setOutputLimits(0, 180);      // command stays a valid angle
  pid.setTolerance(1.0);            // "close enough" is within 1 degree
  lastMicros = micros();

  // Part 1: snap to setpoints (no smoothing yet).
  Serial.println(F("Part 1: snapping"));
  joint.setAngle(30);  joint.update(); delay(500);
  joint.setAngle(150); joint.update(); delay(500);

  // Part 2: turn on smoothing — now setAngle() is approached at 90 deg/s.
  Serial.println(F("Part 2: smoothed"));
  joint.useSmoothing(smoothing);
}

void loop() {
  // Compute a real dt in seconds from the clock.
  uint32_t now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;

  // Part 2 in action: alternate targets; update() ramps toward them smoothly.
  static float target = 30;
  static uint32_t lastSwap = 0;
  if (now - lastSwap > 2000000UL) { target = (target == 30) ? 150 : 30; lastSwap = now; }
  joint.setAngle(target);
  joint.update();

  // Part 3: run YOUR OWN PID loop alongside it.
  // Real closed loop: float measurement = map(analogRead(A0), 0, 1023, 0, 180);
  // Here we simulate the measurement lagging toward the joint's commanded angle.
  static float measurement = 30;
  measurement += (joint.currentDeg() - measurement) * 0.1f;
  float effort = pid.calculate(target, measurement, dt);

  Serial.print(F("target ")); Serial.print(target);
  Serial.print(F("  measured ")); Serial.print(measurement);
  Serial.print(F("  pid effort ")); Serial.print(effort);
  Serial.print(F("  atSetpoint ")); Serial.println(pid.atSetpoint() ? F("yes") : F("no"));
  delay(20);
}
