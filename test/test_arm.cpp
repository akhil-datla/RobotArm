// Phase 9 — RobotArm, the convenience layer that composes the public Joint,
// ArmKinematics, and Gripper components. Everything is host-tested with fakes:
// no hardware, deterministic FakeClock.
#include <cmath>
#include <type_traits>
#include "doctest.h"
#include "TestHelpers.h"
#include "Arm.h"
#include "FakeServoOutput.h"
#include "FakeClock.h"

using namespace roboarm;

// Self-referential internal pointers make these types unsafe to copy — enforce it.
static_assert(!std::is_copy_constructible<RobotArm>::value, "RobotArm must be non-copyable");
static_assert(!std::is_copy_assignable<RobotArm>::value, "RobotArm must be non-copy-assignable");
static_assert(!std::is_copy_constructible<Joint>::value, "Joint must be non-copyable");

// Expected pulse (us) for an angle under the default RDS3225 map (0->500, 180->2500).
static double pulseFor(double deg) { return 500.0 + deg / 180.0 * 2000.0; }

TEST_CASE("moveTo drives each fake output to the expected calibrated command") {
    // Asymmetric arm; Appendix A: tip (152.763, 160.324) phi=90 -> (20, 40, 30),
    // all within the default [0,180] joint range.
    FakeClock clk;
    RobotArm arm(clk);
    arm.setLinkLengths(120.0f, 80.0f, 50.0f);
    FakeServoOutput sh, el, wr;
    CHECK(arm.addShoulder(sh));
    CHECK(arm.addElbow(el));
    CHECK(arm.addWrist(wr));

    bool ok = arm.moveTo(152.763f, 160.324f, 90.0f);
    CHECK(ok);
    CHECK(sh.lastMicroseconds() == tst::approx(pulseFor(20.0)));
    CHECK(el.lastMicroseconds() == tst::approx(pulseFor(40.0)));
    CHECK(wr.lastMicroseconds() == tst::approx(pulseFor(30.0)));
    // The joints the arm holds report those same angles.
    CHECK(arm.joint(0).currentDeg() == tst::approxDeg(20.0));
    CHECK(arm.joint(1).currentDeg() == tst::approxDeg(40.0));
    CHECK(arm.joint(2).currentDeg() == tst::approxDeg(30.0));
}

TEST_CASE("moveTo reports reachability and never commands out of range") {
    FakeClock clk;
    RobotArm arm(clk);
    arm.setLinkLengths(100.0f, 100.0f, 60.0f);
    FakeServoOutput sh, el, wr;
    arm.addShoulder(sh);
    arm.addElbow(el);
    arm.addWrist(wr);

    SUBCASE("far out of reach returns false and clamps to the nearest pose") {
        bool ok = arm.moveTo(500.0f, 0.0f, 0.0f);  // way past 260 mm reach
        CHECK_FALSE(ok);
        // Commands are still within each joint's [0,180] limit (no NaN/overrange).
        FakeServoOutput* outs[3] = {&sh, &el, &wr};
        for (FakeServoOutput* o : outs) {
            CHECK(o->lastMicroseconds() >= 500.0f - 1e-3f);
            CHECK(o->lastMicroseconds() <= 2500.0f + 1e-3f);
        }
        // The nearest reachable pose for (500,0,phi=0) is the arm fully extended
        // along +X: tip at (L1+L2+L3) = 260 mm, hand level.
        ToolPose p = arm.currentPose();
        CHECK(p.x == tst::approx(260.0, 0.5));
        CHECK(p.y == tst::approx(0.0, 0.5));
        CHECK(p.approachDeg == tst::approxDeg(0.0));
    }
    SUBCASE("reachable at one approach angle, not another") {
        // Widen the joint ranges so this isolates KINEMATIC reachability: the
        // elbow-down solution for (240,0,0) needs a slightly negative shoulder.
        for (int i = 0; i < 3; ++i) arm.joint(i).setLimits(-90.0f, 180.0f);
        CHECK(arm.moveTo(240.0f, 0.0f, 0.0f));         // wrist point (180,0): ok
        CHECK_FALSE(arm.moveTo(240.0f, 0.0f, 180.0f)); // wrist point (300,0): out
    }
}

TEST_CASE("reachable() query matches moveTo without commanding") {
    FakeClock clk;
    RobotArm arm(clk);
    arm.setLinkLengths(100.0f, 100.0f, 60.0f);
    CHECK(arm.reachable(150.0f, 60.0f, -90.0f));
    CHECK_FALSE(arm.reachable(500.0f, 0.0f, 0.0f));
}

TEST_CASE("setMaxSpeed makes update() ramp joints over multiple ticks") {
    FakeClock clk;
    RobotArm arm(clk);
    arm.setLinkLengths(120.0f, 80.0f, 50.0f);
    FakeServoOutput sh, el, wr;
    arm.addShoulder(sh);
    arm.addElbow(el);
    arm.addWrist(wr);
    arm.setMaxSpeed(30.0f);  // 30 deg/sec

    // Joints start at the 90 deg default. Command shoulder toward 20 deg.
    arm.setJointAngles(20.0f, 40.0f, 30.0f);  // first (internal) update: dt=0 baseline
    CHECK(arm.joint(0).currentDeg() == tst::approxDeg(90.0));  // has not jumped

    clk.advanceSeconds(0.1f);
    arm.update();  // 30*0.1 = 3 deg step, from 90 toward 20
    CHECK(arm.joint(0).currentDeg() == tst::approxDeg(87.0));
    CHECK(arm.joint(0).currentDeg() > 20.0f);  // still ramping, not jumped

    // Enough ticks later it arrives.
    for (int i = 0; i < 200; ++i) {
        clk.advanceSeconds(0.1f);
        arm.update();
    }
    CHECK(arm.joint(0).currentDeg() == tst::approxDeg(20.0));
    CHECK(arm.joint(1).currentDeg() == tst::approxDeg(40.0));
    CHECK(arm.joint(2).currentDeg() == tst::approxDeg(30.0));
}

TEST_CASE("downward grasp reaches when the wrist is calibrated for negative angles") {
    // Regression for the example/quickstart config: a downward approach (phi=-90)
    // needs a large NEGATIVE wrist angle. With the wrist centered (offset 180,
    // limits [-180,0]) the pose is genuinely reached instead of silently clamping.
    FakeClock clk;
    RobotArm arm(clk);
    arm.setLinkLengths(100.0f, 100.0f, 60.0f);
    FakeServoOutput sh, el, wr;
    arm.addShoulder(sh);
    arm.addElbow(el);
    arm.addWrist(wr);
    arm.joint(2).setOffset(180.0f);
    arm.joint(2).setLimits(-180.0f, 0.0f);

    bool ok = arm.moveTo(150.0f, 40.0f, -90.0f);  // grab from above
    CHECK(ok);                                     // reached, not clamped
    const JointAngles a = arm.lastSolution();
    CHECK(a.wrist < -90.0f);                       // genuinely a large negative angle
    // The commanded wrist angle equals the IK solution (no soft-limit clamp)...
    CHECK(arm.joint(2).currentDeg() == tst::approx(a.wrist));
    // ...and the pulse is a valid, in-range command (never garbage).
    CHECK(wr.lastMicroseconds() >= 500.0f - 1e-3f);
    CHECK(wr.lastMicroseconds() <= 2500.0f + 1e-3f);
}

TEST_CASE("moveTo returns false when a joint soft limit clamps the solution") {
    FakeClock clk;
    RobotArm arm(clk);
    arm.setLinkLengths(120.0f, 80.0f, 50.0f);
    FakeServoOutput sh, el, wr;
    arm.addShoulder(sh);
    arm.addElbow(el);
    arm.addWrist(wr);
    // Constrain the elbow so the IK solution (40 deg) is outside its soft range.
    arm.joint(1).setLimits(0.0f, 30.0f);

    bool ok = arm.moveTo(152.763f, 160.324f, 90.0f);  // needs elbow = 40 deg
    CHECK_FALSE(ok);  // kinematically reachable, but the elbow limit blocks it
    // The elbow command is safely clamped to its limit (never out of range)...
    CHECK(arm.joint(1).currentDeg() == tst::approxDeg(30.0));
    CHECK(el.lastMicroseconds() == tst::approx(pulseFor(30.0)));
    // ...while the in-limit joints still went exactly where the IK asked.
    CHECK(arm.joint(0).currentDeg() == tst::approxDeg(20.0));
    CHECK(arm.joint(2).currentDeg() == tst::approxDeg(30.0));
}

TEST_CASE("setServoTravel propagates travel to every joint (and later ones)") {
    FakeClock clk;
    RobotArm arm(clk);
    FakeServoOutput sh, el, wr;
    arm.addShoulder(sh);
    arm.addElbow(el);
    arm.addWrist(wr);
    arm.setServoTravel(270.0f);  // switch to a 270-degree servo mapping

    // With 270-deg travel the 500..2500 us span covers 270 deg, so the pulse for a
    // given angle changes: 135 -> 1500, 90 -> 500+90/270*2000, 45 -> 500+45/270*2000.
    arm.setJointAngles(135.0f, 90.0f, 45.0f);  // all within the default [0,180] limits
    CHECK(sh.lastMicroseconds() == tst::approx(1500.0));
    CHECK(el.lastMicroseconds() == tst::approx(500.0 + 90.0 / 270.0 * 2000.0));
    CHECK(wr.lastMicroseconds() == tst::approx(500.0 + 45.0 / 270.0 * 2000.0));

    // A joint added AFTER setServoTravel also gets the 270-deg mapping.
    FakeServoOutput extra;
    CHECK(arm.addJoint(extra));
    arm.joint(3).setAngle(135.0f);
    arm.joint(3).update();
    CHECK(extra.lastMicroseconds() == tst::approx(1500.0));
}

TEST_CASE("setMaxSpeed(0) turns smoothing back off (instant moves)") {
    FakeClock clk;
    RobotArm arm(clk);
    FakeServoOutput sh, el, wr;
    arm.addShoulder(sh);
    arm.addElbow(el);
    arm.addWrist(wr);
    arm.setMaxSpeed(30.0f);  // smoothing on
    arm.setMaxSpeed(0.0f);   // ...and back off
    arm.setJointAngles(20.0f, 40.0f, 30.0f);
    // No ramp: the joints jump straight to the targets on the first update.
    CHECK(arm.joint(0).currentDeg() == tst::approxDeg(20.0));
    CHECK(arm.joint(1).currentDeg() == tst::approxDeg(40.0));
    CHECK(arm.joint(2).currentDeg() == tst::approxDeg(30.0));
}

TEST_CASE("setJointAngles bypasses IK and clamps to limits") {
    FakeClock clk;
    RobotArm arm(clk);
    FakeServoOutput sh, el, wr;
    arm.addShoulder(sh);
    arm.addElbow(el);
    arm.addWrist(wr);

    arm.setJointAngles(10.0f, 200.0f, -30.0f);  // elbow/ wrist out of [0,180]
    CHECK(arm.joint(0).currentDeg() == tst::approxDeg(10.0));
    CHECK(arm.joint(1).currentDeg() == tst::approxDeg(180.0));  // clamped high
    CHECK(arm.joint(2).currentDeg() == tst::approxDeg(0.0));    // clamped low
    CHECK(sh.lastMicroseconds() == tst::approx(pulseFor(10.0)));
}

TEST_CASE("gripper open/close command the configured endpoints") {
    FakeClock clk;
    RobotArm arm(clk);
    FakeServoOutput grip;
    arm.addGripper(grip, /*openDeg=*/30.0f, /*closeDeg=*/120.0f);

    arm.openGripper();
    CHECK(grip.lastMicroseconds() == tst::approx(pulseFor(30.0)));
    arm.closeGripper();
    CHECK(grip.lastMicroseconds() == tst::approx(pulseFor(120.0)));
    arm.setGripper(0.5f);
    CHECK(grip.lastMicroseconds() == tst::approx(pulseFor(75.0)));
}

TEST_CASE("the convenience layer is transparent: accessors expose the real parts") {
    FakeClock clk;
    RobotArm arm(clk);
    arm.setLinkLengths(120.0f, 80.0f, 50.0f);
    FakeServoOutput sh, el, wr;
    arm.addShoulder(sh);
    arm.addElbow(el);
    arm.addWrist(wr);

    // kinematics() is the same object the arm solves with.
    CHECK(arm.kinematics().l1() == tst::approx(120.0));
    CHECK(arm.kinematics().l2() == tst::approx(80.0));
    CHECK(arm.kinematics().l3() == tst::approx(50.0));

    // joint(i) is the same object moveTo commands: reconfigure it and see the
    // effect flow through.
    arm.joint(0).setDirection(-1);  // reverse the shoulder
    arm.moveTo(152.763f, 160.324f, 90.0f);      // shoulder solves to 20 deg
    CHECK(sh.lastMicroseconds() == tst::approx(pulseFor(180.0 - 20.0)));  // reversed
}

TEST_CASE("forward pose accessor reflects the commanded joint angles") {
    FakeClock clk;
    RobotArm arm(clk);
    arm.setLinkLengths(120.0f, 80.0f, 50.0f);
    FakeServoOutput sh, el, wr;
    arm.addShoulder(sh);
    arm.addElbow(el);
    arm.addWrist(wr);
    arm.setJointAngles(20.0f, 40.0f, 30.0f);
    ToolPose p = arm.currentPose();
    CHECK(p.x == tst::approx(152.763));
    CHECK(p.y == tst::approx(160.324));
    CHECK(p.approachDeg == tst::approxDeg(90.0));
}

TEST_CASE("joint(index) clamps out-of-range indices (never out of bounds)") {
    FakeClock clk;
    RobotArm arm(clk);
    FakeServoOutput sh;
    arm.addShoulder(sh);
    // Out-of-range indices return a valid Joint reference (clamped), no UB/crash.
    Joint& lo = arm.joint(-5);
    Joint& hi = arm.joint(999);
    CHECK(&lo == &arm.joint(0));
    CHECK(&hi == &arm.joint(kMaxJoints - 1));
    // The clamped joints are safe to touch.
    hi.setAngle(45.0f);
    CHECK(hi.targetDeg() == tst::approxDeg(45.0));
}

TEST_CASE("adding more than kMaxJoints is rejected gracefully") {
    FakeClock clk;
    RobotArm arm(clk);
    FakeServoOutput outs[kMaxJoints + 3];
    int accepted = 0;
    for (int i = 0; i < kMaxJoints + 3; ++i) {
        if (arm.addJoint(outs[i])) accepted++;
    }
    CHECK(accepted == kMaxJoints);
    CHECK(arm.jointCount() == kMaxJoints);
}

TEST_CASE("default-constructed arm has no clock until setClock; adds are rejected") {
    RobotArm arm;  // default ctor: no clock (host)
    FakeServoOutput out, grip;
    // Without a clock, registration is rejected gracefully (no crash, no UB).
    CHECK_FALSE(arm.addShoulder(out));
    CHECK_FALSE(arm.addJoint(out));
    CHECK_FALSE(arm.addGripper(grip, 30.0f, 120.0f));
    CHECK(arm.jointCount() == 0);
    CHECK_FALSE(arm.hasGripper());

    // After setClock, registration works.
    FakeClock clk;
    arm.setClock(clk);
    CHECK(arm.addShoulder(out));
    CHECK(arm.jointCount() == 1);
    arm.begin();  // host no-op, but exercise it
}

TEST_CASE("setServoTravel also retunes the gripper; update() refreshes it") {
    FakeClock clk;
    RobotArm arm(clk);
    FakeServoOutput sh, grip;
    arm.addShoulder(sh);
    arm.addGripper(grip, 30.0f, 120.0f);
    arm.setServoTravel(270.0f);  // must propagate to the gripper too

    arm.openGripper();  // 30 deg under 270 travel -> 500 + 30/270*2000
    CHECK(grip.lastMicroseconds() == tst::approx(500.0 + 30.0 / 270.0 * 2000.0));

    // arm.update() refreshes the gripper output (the m_hasGripper branch).
    const int before = grip.writeCount();
    arm.update();
    CHECK(grip.writeCount() == before + 1);
}

TEST_CASE("a joint added AFTER setMaxSpeed gets smoothing immediately") {
    FakeClock clk;
    RobotArm arm(clk);
    arm.setMaxSpeed(30.0f);  // enable smoothing first
    FakeServoOutput sh;
    arm.addShoulder(sh);     // attachJointAt must wire smoothing since maxSpeed>0
    arm.joint(0).setAngle(0.0f);
    arm.update();            // baseline
    CHECK(arm.joint(0).currentDeg() == tst::approxDeg(90.0));  // starts at default, no jump
    arm.setJointAngles(0.0f, 0.0f, 0.0f);  // target shoulder 0
    clk.advanceSeconds(0.1f);
    arm.update();
    // Smoothing is active: it ramps (90 -> 87), it does not jump straight to 0.
    CHECK(arm.joint(0).currentDeg() == tst::approxDeg(87.0));
}

TEST_CASE("2-arg moveTo uses the default approach angle") {
    FakeClock clk;
    RobotArm arm(clk);
    arm.setLinkLengths(120.0f, 80.0f, 50.0f);
    FakeServoOutput sh, el, wr;
    arm.addShoulder(sh);
    arm.addElbow(el);
    arm.addWrist(wr);
    arm.setDefaultApproachAngle(90.0f);
    CHECK(arm.moveTo(152.763f, 160.324f));  // phi defaults to 90
    CHECK(sh.lastMicroseconds() == tst::approx(pulseFor(20.0)));
    CHECK(wr.lastMicroseconds() == tst::approx(pulseFor(30.0)));
}
