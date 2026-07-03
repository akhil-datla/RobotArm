/*
  04_MoveToXY — the core lesson: reach to a point with inverse kinematics.

  This shows the component way FIRST, so you see inverse kinematics as a visible
  step: solve for the joint angles, read them back, then command the joints. A
  Gripper (a servo driven to two angles) picks and places. At the very end, the
  SAME motion is shown in three lines with the RobotArm convenience — proof that
  RobotArm is just these parts wired together.
*/
#include <RobotArm.h>
using namespace roboarm;

// ---------- The component way (assemble the parts you understand) ----------
ServoDriver board(0x40, 50);
Joint shoulder(board, 0), elbow(board, 1), wrist(board, 2);
Gripper gripper(board, 3, /*openDeg=*/30, /*closeDeg=*/120);
ArmKinematics kin(100, 100, 60);

void reachTo(float x, float y, float approachDeg) {
  // STEP 1 - inverse kinematics: what joint angles put the hand at (x, y, phi)?
  JointAngles a = kin.solve(x, y, approachDeg);
  if (!a.reachable) {
    Serial.println(F("  (target unreachable — clamped to nearest reachable pose)"));
  }
  // STEP 2 - command each joint. The Joint turns degrees into a pulse for you.
  shoulder.setAngle(a.shoulder);
  elbow.setAngle(a.elbow);
  wrist.setAngle(a.wrist);
  // STEP 3 - push the targets out to the servos.
  shoulder.update(); elbow.update(); wrist.update();
}

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
}

void loop() {
  // Reach out and pick from above (gripper pointing straight down, phi = -90).
  Serial.println(F("pick at (150, 40)"));
  reachTo(150, 40, -90);
  gripper.open();  delay(600);
  gripper.close(); delay(600);   // grab

  // Move to a place point, hand level (phi = 0), and release.
  Serial.println(F("place at (60, 150)"));
  reachTo(60, 150, 0);
  gripper.open();  delay(600);   // release

  delay(1000);

  // ---------- The short way: RobotArm wires the EXACT same parts together ----
  // (Uncomment to try — it composes Joints + ArmKinematics + Gripper internally.)
  //
  //   RobotArm arm;
  //   arm.setLinkLengths(100, 100, 60);
  //   arm.addShoulder(0); arm.addElbow(1); arm.addWrist(2); arm.addGripper(3, 30, 120);
  //   arm.begin();
  //   arm.moveTo(150, 40, -90);   // = solve IK, command the joints, update — for you
  //   arm.closeGripper();
  //   arm.update();
}
