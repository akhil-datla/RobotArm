/*
  03_WristAngle — forward kinematics and the approach angle.

  Forward kinematics answers: "given the joint angles, WHERE is the hand and which
  way does it point?" This sketch sets the shoulder/elbow/wrist by hand and prints
  the tip (x, y) and the approach angle phi = shoulder + elbow + wrist.

  Then it sweeps phi while keeping the wrist POINT fixed, so you can watch the hand
  tilt from level to pointing straight down without the arm's wrist moving in space.
*/
#include <RobotArm.h>
using namespace roboarm;

ServoDriver board(0x40, 50);
Joint shoulder(board, 0), elbow(board, 1), wrist(board, 2);

// Link lengths in mm: upper arm, forearm, hand.
ArmKinematics kin(100, 100, 60);

// Give a joint the RDS3225 (180-degree) defaults.
void configJoint(Joint& j) {
  j.setPulseRange(500, 2500);
  j.setTravel(180);
  j.setLimits(0, 180);
}

void setup() {
  Serial.begin(115200);
  board.begin();
  configJoint(shoulder);
  configJoint(elbow);
  configJoint(wrist);
  // Pointing the hand down/level (phi <= 0) needs a large NEGATIVE wrist angle, so
  // calibrate the wrist centered: offset 180 maps theta3 = -180..0 onto the servo's
  // 0..180 range (otherwise the wrist would silently clamp to 0). See example 02.
  wrist.setOffset(180);
  wrist.setLimits(-180, 0);

  // Set a pose by hand and see where the hand ends up.
  shoulder.setAngle(30); elbow.setAngle(60); wrist.setAngle(0);
  shoulder.update(); elbow.update(); wrist.update();

  ToolPose pose = kin.forward(30, 60, 0);
  Serial.print(F("tip x = ")); Serial.print(pose.x);
  Serial.print(F(" mm, y = ")); Serial.print(pose.y);
  Serial.print(F(" mm, approach = ")); Serial.print(pose.approachDeg);
  Serial.println(F(" deg"));
}

void loop() {
  // Hold the WRIST point fixed at (140, 80) mm and sweep the approach angle phi.
  // The tip rides on an arc of radius L3 (=60 mm) around that fixed wrist point,
  // so the wrist stays put in space while only the hand tilts. For each phi we
  // work out the tip that keeps the wrist there, then solve IK for it.
  const float xw = 140.0f, yw = 80.0f;
  for (int phi = 0; phi >= -90; phi -= 5) {
    float tipX = xw + 60.0f * cosf(degToRad((float)phi));
    float tipY = yw + 60.0f * sinf(degToRad((float)phi));
    JointAngles a = kin.solve(tipX, tipY, phi);
    if (a.reachable) {
      shoulder.setAngle(a.shoulder); elbow.setAngle(a.elbow); wrist.setAngle(a.wrist);
      shoulder.update(); elbow.update(); wrist.update();
      Serial.print(F("phi = ")); Serial.print(phi);
      Serial.print(F(" -> shoulder ")); Serial.print(a.shoulder);
      Serial.print(F(", elbow ")); Serial.print(a.elbow);
      Serial.print(F(", wrist ")); Serial.println(a.wrist);
    } else {
      Serial.print(F("phi = ")); Serial.print(phi); Serial.println(F(" unreachable"));
    }
    delay(400);
  }
}
