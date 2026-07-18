// RobotArm3D — the 4-DOF convenience layer: a rotating base joint on top of the
// planar RobotArm. It composes the SAME public parts (a base Joint + the planar
// RobotArm + ArmKinematics3D) and exposes them, so this is glass-box, not a
// parallel implementation. Everything is host-tested with fakes + a FakeClock.
#include <cmath>
#include <type_traits>
#include "doctest.h"
#include "TestHelpers.h"
#include "Arm3D.h"
#include "MathUtils.h"
#include "FakeServoOutput.h"
#include "FakeClock.h"

using namespace roboarm;

// Self-referential internal pointers (the planar arm's slews, the base slew) make
// RobotArm3D unsafe to copy — enforce it at compile time.
static_assert(!std::is_copy_constructible<RobotArm3D>::value, "RobotArm3D must be non-copyable");
static_assert(!std::is_copy_assignable<RobotArm3D>::value, "RobotArm3D must be non-copy-assignable");

// Planar joints: default RDS3225 map (0->500, 180->2500), offset 0, limits [0,180].
static double pulseFor(double deg) { return 500.0 + deg / 180.0 * 2000.0; }
// The base joint is centered on "forward": offset = travel/2 (90), limits +/-90,
// so azimuth 0 maps to the servo's 1500 us center and +/-90 to the endpoints.
static double basePulseFor(double deg) { return 500.0 + (deg + 90.0) / 180.0 * 2000.0; }

TEST_CASE("a fresh injected 3D arm has sane defaults") {
    FakeClock clk;
    RobotArm3D arm(clk);
    CHECK_FALSE(arm.hasBase());
    CHECK(arm.jointCount() == 0);        // planar joints, base counted separately
    CHECK(arm.lastReachable());
    CHECK_FALSE(arm.lastSolution().clamped);
}

TEST_CASE("moveTo drives the base and the three planar joints to expected pulses") {
    // Asymmetric arm; Appendix A planar pose (20,40,30) -> tip (152.763,160.324),
    // phi=90 — all planar angles inside the default [0,180] range. Yaw the whole
    // thing to base=30 deg and feed the resulting 3D point back in.
    FakeClock clk;
    RobotArm3D arm(clk);
    arm.setLinkLengths(120.0f, 80.0f, 50.0f);
    FakeServoOutput base, sh, el, wr;
    CHECK(arm.addBase(base));
    CHECK(arm.addShoulder(sh));
    CHECK(arm.addElbow(el));
    CHECK(arm.addWrist(wr));

    ToolPose3D fk = arm.kinematics().forward(30.0f, 20.0f, 40.0f, 30.0f);
    bool ok = arm.moveTo(fk.x, fk.y, fk.z, fk.approachDeg);
    CHECK(ok);
    CHECK(base.lastMicroseconds() == tst::approx(basePulseFor(30.0)));
    CHECK(sh.lastMicroseconds() == tst::approx(pulseFor(20.0)));
    CHECK(el.lastMicroseconds() == tst::approx(pulseFor(40.0)));
    CHECK(wr.lastMicroseconds() == tst::approx(pulseFor(30.0)));
    // The parts report those same angles.
    CHECK(arm.base().currentDeg() == tst::approxDeg(30.0));
    CHECK(arm.joint(0).currentDeg() == tst::approxDeg(20.0));
    CHECK(arm.joint(1).currentDeg() == tst::approxDeg(40.0));
    CHECK(arm.joint(2).currentDeg() == tst::approxDeg(30.0));
}

TEST_CASE("the z=0 slice matches the plain 2D arm, base centered") {
    FakeClock clk;
    RobotArm3D a3(clk);
    RobotArm a2(clk);
    a3.setLinkLengths(120.0f, 80.0f, 50.0f);
    a2.setLinkLengths(120.0f, 80.0f, 50.0f);
    FakeServoOutput base, sh3, el3, wr3, sh2, el2, wr2;
    a3.addBase(base);
    a3.addShoulder(sh3); a3.addElbow(el3); a3.addWrist(wr3);
    a2.addShoulder(sh2); a2.addElbow(el2); a2.addWrist(wr2);

    CHECK(a3.moveTo(152.763f, 160.324f, 0.0f, 90.0f));
    CHECK(a2.moveTo(152.763f, 160.324f, 90.0f));
    // Base sits at the forward center (azimuth 0 -> 1500 us).
    CHECK(base.lastMicroseconds() == tst::approx(basePulseFor(0.0)));
    CHECK(base.lastMicroseconds() == tst::approx(1500.0));
    // The planar joints match the 2D arm exactly.
    CHECK(sh3.lastMicroseconds() == tst::approx(sh2.lastMicroseconds()));
    CHECK(el3.lastMicroseconds() == tst::approx(el2.lastMicroseconds()));
    CHECK(wr3.lastMicroseconds() == tst::approx(wr2.lastMicroseconds()));
}

TEST_CASE("the base canonicalizes azimuth: +Z yaws one way, -Z the other") {
    FakeClock clk;
    RobotArm3D arm(clk);
    arm.setLinkLengths(120.0f, 80.0f, 50.0f);
    FakeServoOutput base, sh, el, wr;
    arm.addBase(base);
    arm.addShoulder(sh); arm.addElbow(el); arm.addWrist(wr);

    SUBCASE("target straight out +Z -> base +90") {
        CHECK(arm.moveTo(0.0f, 160.324f, 152.763f, 90.0f));
        CHECK(arm.lastSolution().base == tst::approxDeg(90.0));
        CHECK(base.lastMicroseconds() == tst::approx(basePulseFor(90.0)));
    }
    SUBCASE("target straight out -Z -> base -90") {
        CHECK(arm.moveTo(0.0f, 160.324f, -152.763f, 90.0f));
        CHECK(arm.lastSolution().base == tst::approxDeg(-90.0));
        CHECK(base.lastMicroseconds() == tst::approx(basePulseFor(-90.0)));
    }
    SUBCASE("the same planar solution rides along at any azimuth") {
        arm.moveTo(0.0f, 160.324f, 152.763f, 90.0f);
        CHECK(arm.lastSolution().shoulder == tst::approxDeg(20.0));
        CHECK(arm.lastSolution().elbow == tst::approxDeg(40.0));
        CHECK(arm.lastSolution().wrist == tst::approxDeg(30.0));
    }
}

TEST_CASE("3D moveTo reports reachability and never commands out of range") {
    FakeClock clk;
    RobotArm3D arm(clk);
    arm.setLinkLengths(100.0f, 100.0f, 60.0f);
    FakeServoOutput base, sh, el, wr;
    arm.addBase(base);
    arm.addShoulder(sh); arm.addElbow(el); arm.addWrist(wr);

    // Horizontal distance sqrt(500^2 + 300^2) = 583 mm, way past the 260 mm reach.
    bool ok = arm.moveTo(500.0f, 0.0f, 300.0f, 0.0f);
    CHECK_FALSE(ok);
    FakeServoOutput* outs[4] = {&base, &sh, &el, &wr};
    for (FakeServoOutput* o : outs) {
        CHECK(o->lastMicroseconds() >= 500.0f - 1e-3f);
        CHECK(o->lastMicroseconds() <= 2500.0f + 1e-3f);
    }
    // The base still turned to the correct azimuth even though the reach clamped.
    CHECK(arm.lastSolution().base == tst::approxDeg(radToDeg(std::atan2(300.0f, 500.0f))));
    CHECK(std::isfinite(arm.lastSolution().shoulder));
}

TEST_CASE("reachable() query matches moveTo without commanding anything") {
    FakeClock clk;
    RobotArm3D arm(clk);
    arm.setLinkLengths(100.0f, 100.0f, 60.0f);
    // 150 mm forward + 60 up, azimuth 45 -> well inside reach.
    CHECK(arm.reachable(106.066f, 60.0f, 106.066f, -90.0f));
    CHECK_FALSE(arm.reachable(500.0f, 0.0f, 300.0f, 0.0f));
}

TEST_CASE("setMaxSpeed ramps the base joint over multiple ticks, not a jump") {
    FakeClock clk;
    RobotArm3D arm(clk);
    arm.setLinkLengths(120.0f, 80.0f, 50.0f);
    FakeServoOutput base, sh, el, wr;
    arm.addBase(base);
    arm.addShoulder(sh); arm.addElbow(el); arm.addWrist(wr);
    arm.setMaxSpeed(30.0f);  // 30 deg/sec on every joint including the base

    // Command a pose at base = 60 deg. The base starts from the 90 deg default.
    ToolPose3D fk = arm.kinematics().forward(60.0f, 20.0f, 40.0f, 30.0f);
    arm.moveTo(fk.x, fk.y, fk.z, fk.approachDeg);  // internal update: dt=0 baseline
    CHECK(arm.base().currentDeg() == tst::approxDeg(90.0));  // has not jumped

    clk.advanceSeconds(0.1f);
    arm.update();  // 30 * 0.1 = 3 deg step, from 90 toward 60
    CHECK(arm.base().currentDeg() == tst::approxDeg(87.0));
    CHECK(arm.base().currentDeg() > 60.0f);  // still ramping

    for (int i = 0; i < 200; ++i) { clk.advanceSeconds(0.1f); arm.update(); }
    CHECK(arm.base().currentDeg() == tst::approxDeg(60.0));
    CHECK(arm.joint(0).currentDeg() == tst::approxDeg(20.0));
}

TEST_CASE("setJointAngles bypasses IK and clamps every joint to its limits") {
    FakeClock clk;
    RobotArm3D arm(clk);
    FakeServoOutput base, sh, el, wr;
    arm.addBase(base);
    arm.addShoulder(sh); arm.addElbow(el); arm.addWrist(wr);

    // base 45 (in [-90,90]); shoulder 10; elbow past 180 -> clamps; wrist below 0.
    arm.setJointAngles(45.0f, 10.0f, 200.0f, -30.0f);
    CHECK(arm.base().currentDeg() == tst::approxDeg(45.0));
    CHECK(arm.joint(0).currentDeg() == tst::approxDeg(10.0));
    CHECK(arm.joint(1).currentDeg() == tst::approxDeg(180.0));  // clamped high
    CHECK(arm.joint(2).currentDeg() == tst::approxDeg(0.0));    // clamped low
    CHECK(base.lastMicroseconds() == tst::approx(basePulseFor(45.0)));

    SUBCASE("the base limit clamps an over-range azimuth command") {
        arm.setJointAngles(200.0f, 10.0f, 20.0f, 30.0f);   // 200 -> clamps to +90
        CHECK(arm.base().currentDeg() == tst::approxDeg(90.0));
        arm.setJointAngles(-200.0f, 10.0f, 20.0f, 30.0f);  // -200 -> clamps to -90
        CHECK(arm.base().currentDeg() == tst::approxDeg(-90.0));
    }
}

TEST_CASE("3-arg moveTo uses the default approach angle") {
    FakeClock clk;
    RobotArm3D arm(clk);
    arm.setLinkLengths(120.0f, 80.0f, 50.0f);
    FakeServoOutput base, sh, el, wr;
    arm.addBase(base);
    arm.addShoulder(sh); arm.addElbow(el); arm.addWrist(wr);
    arm.setDefaultApproachAngle(90.0f);

    ToolPose3D fk = arm.kinematics().forward(30.0f, 20.0f, 40.0f, 30.0f);  // phi=90
    CHECK(arm.moveTo(fk.x, fk.y, fk.z));  // 3-arg -> default approach 90
    CHECK(sh.lastMicroseconds() == tst::approx(pulseFor(20.0)));
    CHECK(wr.lastMicroseconds() == tst::approx(pulseFor(30.0)));
    CHECK(base.lastMicroseconds() == tst::approx(basePulseFor(30.0)));
}

TEST_CASE("currentPose reflects the commanded base + planar angles") {
    FakeClock clk;
    RobotArm3D arm(clk);
    arm.setLinkLengths(120.0f, 80.0f, 50.0f);
    FakeServoOutput base, sh, el, wr;
    arm.addBase(base);
    arm.addShoulder(sh); arm.addElbow(el); arm.addWrist(wr);

    arm.setJointAngles(30.0f, 20.0f, 40.0f, 30.0f);
    ToolPose3D p = arm.currentPose();
    CHECK(p.x == tst::approx(132.2972));   // 152.763 * cos30
    CHECK(p.y == tst::approx(160.324));
    CHECK(p.z == tst::approx(76.3815));    // 152.763 * sin30
    CHECK(p.approachDeg == tst::approxDeg(90.0));
}

TEST_CASE("setServoTravel propagates to the base and the planar joints") {
    FakeClock clk;
    RobotArm3D arm(clk);
    FakeServoOutput base, sh, el, wr;
    arm.addBase(base);
    arm.addShoulder(sh); arm.addElbow(el); arm.addWrist(wr);
    arm.setServoTravel(270.0f);  // switch to a 270-degree servo mapping

    // Base re-centers on the new travel: offset/limits become +/-135, so azimuth 0
    // is still the 1500 us center, and the planar joints use the 270 map.
    arm.setJointAngles(0.0f, 135.0f, 90.0f, 45.0f);
    CHECK(base.lastMicroseconds() == tst::approx(1500.0));  // azimuth 0 stays centered
    CHECK(sh.lastMicroseconds() == tst::approx(1500.0));    // 135 of 270 travel
    CHECK(el.lastMicroseconds() == tst::approx(500.0 + 90.0 / 270.0 * 2000.0));
    CHECK(wr.lastMicroseconds() == tst::approx(500.0 + 45.0 / 270.0 * 2000.0));

    SUBCASE("the widened base range now reaches +120 azimuth") {
        arm.setJointAngles(120.0f, 90.0f, 90.0f, 0.0f);  // 120 is inside +/-135 now
        CHECK(arm.base().currentDeg() == tst::approxDeg(120.0));
    }
}

TEST_CASE("the gripper works through the 3D arm") {
    FakeClock clk;
    RobotArm3D arm(clk);
    FakeServoOutput base, grip;
    arm.addBase(base);
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
    RobotArm3D arm(clk);
    arm.setLinkLengths(120.0f, 80.0f, 50.0f);
    FakeServoOutput base, sh, el, wr;
    arm.addBase(base);
    arm.addShoulder(sh); arm.addElbow(el); arm.addWrist(wr);

    // kinematics() is the ArmKinematics3D the arm solves with.
    CHECK(arm.kinematics().l1() == tst::approx(120.0));
    CHECK(arm.kinematics().l2() == tst::approx(80.0));
    CHECK(arm.kinematics().l3() == tst::approx(50.0));

    // arm() is the same planar RobotArm the 3D layer drives; base() is the base
    // Joint; joint(i) forwards to the planar arm. Reconfigure a part, see it flow.
    CHECK(&arm.joint(0) == &arm.arm().joint(0));
    arm.base().setDirection(-1);  // reverse the base
    arm.moveTo(0.0f, 160.324f, 152.763f, 90.0f);  // base solves to +90
    CHECK(base.lastMicroseconds() == tst::approx(basePulseFor(-90.0)));  // reversed
}

TEST_CASE("default-constructed 3D arm has no clock until setClock; adds are rejected") {
    RobotArm3D arm;  // default ctor: no clock on host
    FakeServoOutput base, sh, grip;
    CHECK_FALSE(arm.addBase(base));
    CHECK_FALSE(arm.addShoulder(sh));
    CHECK_FALSE(arm.addGripper(grip, 30.0f, 120.0f));
    CHECK_FALSE(arm.hasBase());
    CHECK(arm.jointCount() == 0);

    FakeClock clk;
    arm.setClock(clk);
    CHECK(arm.addBase(base));
    CHECK(arm.addShoulder(sh));
    CHECK(arm.hasBase());
    arm.begin();  // host no-op, but exercise it
}

TEST_CASE("moveTo before a base is registered still drives the planar joints") {
    // The base is optional: a 3D arm with only the planar joints behaves like the
    // 2D arm for z=0 targets and simply ignores the (absent) base.
    FakeClock clk;
    RobotArm3D arm(clk);
    arm.setLinkLengths(120.0f, 80.0f, 50.0f);
    FakeServoOutput sh, el, wr;
    arm.addShoulder(sh); arm.addElbow(el); arm.addWrist(wr);
    CHECK_FALSE(arm.hasBase());
    CHECK(arm.moveTo(152.763f, 160.324f, 0.0f, 90.0f));
    CHECK(sh.lastMicroseconds() == tst::approx(pulseFor(20.0)));
}
