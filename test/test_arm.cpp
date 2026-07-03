// Phase 9 — RobotArm, the convenience layer that composes the public Joint,
// ArmKinematics, and Gripper components. Everything is host-tested with fakes:
// no hardware, deterministic FakeClock.
#include <cmath>
#include "doctest.h"
#include "TestHelpers.h"
#include "Arm.h"
#include "FakeServoOutput.h"
#include "FakeClock.h"

using namespace roboarm;

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

    SUBCASE("far out of reach returns false and clamps") {
        bool ok = arm.moveTo(500.0f, 0.0f, 0.0f);  // way past 260 mm reach
        CHECK_FALSE(ok);
        // Commands are still within each joint's [0,180] limit (no NaN/overrange).
        for (FakeServoOutput* o : {&sh, &el, &wr}) {
            CHECK(o->lastMicroseconds() >= 500.0f - 1e-3f);
            CHECK(o->lastMicroseconds() <= 2500.0f + 1e-3f);
        }
    }
    SUBCASE("reachable at one approach angle, not another") {
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
