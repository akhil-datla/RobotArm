/*
  06_MoveToXYZ — reach into 3D with a rotating BASE joint.

  This is example 04 grown a dimension. A base joint that yaws about the vertical
  axis turns the flat 3-joint arm into a 4-DOF SPATIAL arm. The idea splits into
  two easy steps you already understand:

    1. AIM  — the base turns to the target's compass direction (its azimuth),
    2. REACH — the same planar shoulder/elbow/wrist arm reaches the target's
               horizontal distance + height in that now-rotated vertical plane.

  So the only new math is one atan2 for the base; everything else is the 2D arm
  you already have. We show the component way FIRST (solve 3D IK, read the four
  angles, command the joints), then the SAME motion in a few lines with the
  RobotArm3D convenience — proof it is just these parts wired together.

  FRAME (see README):  +X forward, +Y up, +Z sideways (to the arm's left).
    The base angle is 0 when the arm faces +X; +90 points it toward +Z.

  WIRING: base servo on channel 0; shoulder/elbow/wrist on 1/2/3; gripper on 4.
  Power the servos from a separate 5-6 V supply — see the README.
*/
#include <RobotArm.h>
using namespace roboarm;

// ---------- The component way (assemble the parts you understand) ----------
ServoDriver board(0x40, 50);
Joint base(board, 0);                         // NEW: the base yaw joint
Joint shoulder(board, 1), elbow(board, 2), wrist(board, 3);
Gripper gripper(board, 4, /*openDeg=*/30, /*closeDeg=*/120);
ArmKinematics3D kin(100, 100, 60);            // same 3 link lengths; the base adds yaw

void reachTo(float x, float y, float z, float approachDeg) {
  // STEP 1 - inverse kinematics: what base + joint angles put the hand at
  // (x, y, z) with the gripper pitched at approachDeg?
  JointAngles3D a = kin.solve(x, y, z, approachDeg);
  if (!a.reachable) {
    Serial.println(F("  (target unreachable — clamped to nearest reachable pose)"));
  }
  // STEP 2 - command each joint. The base aims; the planar joints reach.
  base.setAngle(a.base);
  shoulder.setAngle(a.shoulder);
  elbow.setAngle(a.elbow);
  wrist.setAngle(a.wrist);
  // STEP 3 - push the targets out to the servos.
  base.update(); shoulder.update(); elbow.update(); wrist.update();
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

  // The BASE is centered on "forward": mount/calibrate it so the servo's middle
  // (90 deg / 1500 us) points the arm along +X. An offset of 90 maps the azimuth
  // range -90..+90 onto the servo's 0..180, so the arm can swing to either side.
  base.setOffset(90);
  base.setLimits(-90, 90);

  // The wrist bends back past its own zero for a downward/level hand, so its
  // kinematics angle is large and NEGATIVE — center it exactly as in example 04.
  wrist.setOffset(180);
  wrist.setLimits(-180, 0);
}

void loop() {
  // Pick from a spot out to the LEFT (+Z), gripper pointing straight down.
  Serial.println(F("pick at (110, 40, 70)  [forward, up, left]"));
  reachTo(110, 40, 70, -90);
  gripper.open();  delay(600);
  gripper.close(); delay(600);   // grab

  // Place to a spot on the RIGHT (-Z), hand level, and release.
  Serial.println(F("place at (110, 120, -70)"));
  reachTo(110, 120, -70, 0);
  gripper.open();  delay(600);   // release

  delay(1000);

  // ---------- The short way: RobotArm3D wires the EXACT same parts together ----
  // (Uncomment to try — it composes a base Joint + the planar RobotArm + the 3D
  //  kinematics internally, and centers the base on forward for you.)
  //
  //   RobotArm3D arm;
  //   arm.setLinkLengths(100, 100, 60);
  //   arm.addBase(0);
  //   arm.addShoulder(1); arm.addElbow(2); arm.addWrist(3); arm.addGripper(4, 30, 120);
  //   arm.joint(2).setOffset(180); arm.joint(2).setLimits(-180, 0);  // wrist, see above
  //   arm.begin();
  //   arm.moveTo(110, 40, 70, -90);   // = solve 3D IK, aim the base, command, update
  //   arm.closeGripper();
  //   arm.update();
}
