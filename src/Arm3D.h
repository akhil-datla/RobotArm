// Arm3D.h — RobotArm3D, the optional one-call convenience layer for a 4-DOF
// SPATIAL arm: a rotating BASE joint on top of the planar RobotArm.
//
// Like RobotArm, this is NOT a black box. It is built *out of* the same public
// parts a student can use directly — a base Joint, the planar RobotArm, and
// ArmKinematics3D — and it exposes every one of them (kinematics(), arm(),
// base(), joint(i), gripper()) so you can drop from the short path down to the
// parts at any time. moveTo(x, y, z, approach) is literally "solve 3D IK -> turn
// the base to the azimuth -> command the planar joints -> update", the same steps
// you would write by hand.
//
// THE IDEA: the base yaws about the vertical +Y axis, aiming the whole planar arm
// at the target's compass direction; the planar arm then reaches the target's
// horizontal distance + height in that rotated vertical plane. See Kinematics3D.h.
//
// The core methods here (injecting IServoOutput / IClock) are always available
// and fully host-tested. The channel-based convenience overloads that build a
// PCA9685-backed output for you are added under #ifdef ARDUINO. See CLAUDE.md
// sections 5, 9, 11.
#pragma once

#include "Arm.h"
#include "Joint.h"
#include "core/Kinematics3D.h"
#include "core/SlewRateLimiter.h"
#include "hal/IClock.h"
#include "hal/IServoOutput.h"

namespace roboarm {

class RobotArm3D {
public:
    // Host/advanced: inject the clock (tests pass a FakeClock). The planar arm
    // shares the same clock.
    explicit RobotArm3D(IClock& clock);

    // Arduino/default: the clock is wired up in begin() (shared ArduinoClock).
    RobotArm3D();

    // Non-copyable / non-movable: the arm holds internal self-references (the
    // planar arm's per-joint smoothing limiters and, on Arduino, its owned PCA9685
    // outputs, plus the base's slew limiter), so a copy would dangle them. Declare
    // one arm and use it.
    RobotArm3D(const RobotArm3D&) = delete;
    RobotArm3D& operator=(const RobotArm3D&) = delete;

    // Bind (or replace) the clock used for motion timing (base + planar joints).
    void setClock(IClock& clock);

#ifdef ARDUINO
    // Select the PCA9685 I2C address and servo PWM frequency (defaults 0x40 @
    // 50 Hz). Optional — only call it if your board differs from the defaults.
    void usePCA9685(uint8_t i2cAddress = kDefaultPca9685Address,
                    float pwmFreqHz = kDefaultServoFreqHz);

    // Channel-based joint registration: builds a PCA9685-backed output for you,
    // all on the one shared board.
    bool addBase(uint8_t channel);
    bool addShoulder(uint8_t channel);
    bool addElbow(uint8_t channel);
    bool addWrist(uint8_t channel);
    bool addJoint(uint8_t channel);
    bool addGripper(uint8_t channel, float openDeg, float closeDeg);

    // Access the shared PCA9685 board wrapper (the planar arm owns it).
    ServoDriver& board() { return m_arm.board(); }
#endif

    // --- configuration ---
    // Servo mechanical travel in degrees (default 180 for the RDS3225). Also
    // re-centers the base's soft limits to +/- travel/2 about "forward".
    void setServoTravel(float degrees);
    // The three link lengths in millimeters (kept in sync across both kinematics).
    void setLinkLengths(float l1Mm, float l2Mm, float l3Mm);
    // Approach angle used by the 3-argument moveTo().
    void setDefaultApproachAngle(float approachDeg);
    // Cap joint speed (deg/sec) on every joint including the base; 0 or less turns
    // smoothing back off (joints move instantly).
    void setMaxSpeed(float degPerSec);
    // Hook for hardware start-up. On host this is a harmless no-op.
    void begin();

    // --- joint registration (inject an output; channel overloads under ARDUINO) ---
    // Register the base yaw joint. It is centered on "forward": azimuth 0 maps to
    // the servo's mechanical center, and the soft limits span +/- travel/2. Returns
    // false if no clock is set yet.
    bool addBase(IServoOutput& output);
    // Register the shoulder / elbow / wrist (the planar IK joints). Forwarded to
    // the planar arm.
    bool addShoulder(IServoOutput& output);
    bool addElbow(IServoOutput& output);
    bool addWrist(IServoOutput& output);
    // Register an extra (non-IK) planar joint; returns false past kMaxJoints.
    bool addJoint(IServoOutput& output);
    // Register the gripper with its open/closed angles. Returns false with no clock.
    bool addGripper(IServoOutput& output, float openDeg, float closeDeg);

    // --- motion ---
    // Solve 3D IK for (x, y, z, approachDeg), turn the base to the azimuth, command
    // the planar joints, and push it all out. Returns true only if the pose was
    // actually reached: false if the target is kinematically unreachable OR if a
    // joint's soft limit clamped the solution (the arm still moves to the nearest
    // safe pose). A false return always means "the hand is not exactly where you
    // asked."
    bool moveTo(float xMm, float yMm, float zMm, float approachDeg);
    // 3-arg form uses the default approach angle.
    bool moveTo(float xMm, float yMm, float zMm);
    // Command the four joint angles directly, bypassing IK (still limit-clamped).
    void setJointAngles(float baseDeg, float shoulderDeg, float elbowDeg,
                        float wristDeg);
    // Advance motion shaping and refresh every servo. Call each loop().
    void update();

    // --- gripper (forwarded to the planar arm) ---
    void openGripper();
    void closeGripper();
    void setGripper(float fraction);

    // --- queries ---
    // Is (x, y, z, approachDeg) reachable? (Does not command anything.)
    bool reachable(float xMm, float yMm, float zMm, float approachDeg) const;
    // Forward kinematics of the current base + planar joint angles.
    ToolPose3D currentPose() const;
    // The most recent IK result (base + three angles, reachable/clamped flags).
    JointAngles3D lastSolution() const { return m_lastSolution; }
    // Whether the most recent moveTo target was kinematically reachable.
    bool lastReachable() const { return m_lastSolution.reachable; }

    // --- component access (the convenience layer is transparent) ---
    // The ArmKinematics3D the arm solves with (same object, not a copy).
    ArmKinematics3D& kinematics() { return m_kin; }
    const ArmKinematics3D& kinematics() const { return m_kin; }
    // The planar RobotArm this 3D layer drives (shoulder/elbow/wrist + gripper).
    RobotArm& arm() { return m_arm; }
    const RobotArm& arm() const { return m_arm; }
    // The base yaw joint.
    Joint& base() { return m_base; }
    const Joint& base() const { return m_base; }
    // The planar Joint at this index (0=shoulder, 1=elbow, 2=wrist). Forwarded to
    // the planar arm, which clamps out-of-range indices (never out of bounds).
    Joint& joint(int index) { return m_arm.joint(index); }
    const Joint& joint(int index) const { return m_arm.joint(index); }
    // The gripper the arm drives.
    Gripper& gripper() { return m_arm.gripper(); }
    // How many planar joints are registered (the base is counted separately).
    int jointCount() const { return m_arm.jointCount(); }
    // Whether a base has been registered.
    bool hasBase() const { return m_hasBase; }
    // Whether a gripper has been registered.
    bool hasGripper() const { return m_arm.hasGripper(); }

private:
    // Attach an injected output as the base, applying the "centered on forward"
    // calibration convention and (if enabled) smoothing.
    bool attachBase(IServoOutput& output);
    // Re-derive the base soft limits + centering offset from the current travel.
    void applyBaseConvention();
    // Command a joint to `deg`; returns true if `deg` was inside that joint's soft
    // limits (i.e. it was not clamped on the way to the servo).
    static bool commandWithinLimits(Joint& joint, float deg);

    ArmKinematics3D m_kin;
    RobotArm m_arm;
    Joint m_base;
    SlewRateLimiter m_baseSlew;
    IClock* m_clock;
    float m_travelDeg;
    float m_defaultApproachDeg;
    float m_maxSpeed;
    bool m_hasBase;
    JointAngles3D m_lastSolution;

#ifdef ARDUINO
    // Storage for the base output built by the channel-based addBase().
    Pca9685Servo m_baseOutput;
#endif
};

}  // namespace roboarm
