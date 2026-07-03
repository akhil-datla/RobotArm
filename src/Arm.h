// Arm.h — RobotArm, the optional one-call convenience layer.
//
// RobotArm is NOT a black box: it is built *out of* the same public components a
// student can use directly — Joint, ArmKinematics, and Gripper — and it exposes
// them (kinematics(), joint(i), gripper()) so you can drop from the short path
// down to the parts at any time. moveTo() is literally "solve IK -> command the
// joints -> update", the same three steps you would write by hand.
//
// The core methods here (injecting IServoOutput / IClock) are always available
// and fully host-tested. The channel-based convenience overloads that build a
// PCA9685-backed output for you are added under #ifdef ARDUINO in the hardware
// phase. See CLAUDE.md sections 5, 9, 11.
#pragma once

#include "Gripper.h"
#include "Joint.h"
#include "core/Kinematics.h"
#include "core/SlewRateLimiter.h"
#include "hal/IClock.h"
#include "hal/IServoOutput.h"

#ifdef ARDUINO
#include "arduino/ArduinoClock.h"
#include "arduino/Pca9685Servo.h"
#include "arduino/ServoDriver.h"
#endif

namespace roboarm {

// Maximum joints the arm can hold (fixed storage, no dynamic memory — §4.3).
constexpr int kMaxJoints = 6;

// Default geometry (mm) so a minimal sketch has something sane.
constexpr float kDefaultL1Mm = 100.0f;
constexpr float kDefaultL2Mm = 100.0f;
constexpr float kDefaultL3Mm = 60.0f;

// The three inverse-kinematics joints live at these fixed indices.
constexpr int kShoulderIndex = 0;
constexpr int kElbowIndex = 1;
constexpr int kWristIndex = 2;

class RobotArm {
public:
    // Host/advanced: inject the clock (tests pass a FakeClock).
    explicit RobotArm(IClock& clock);

    // Arduino/default: the clock is wired up in begin() (Phase 10).
    RobotArm();

    // Non-copyable / non-movable: the arm holds internal self-references — each
    // joint's smoothing limiter (and, on Arduino, its owned PCA9685 output) lives
    // inside this object — so a copy would leave those pointers dangling. Declare
    // one arm and use it.
    RobotArm(const RobotArm&) = delete;
    RobotArm& operator=(const RobotArm&) = delete;

    // Bind (or replace) the clock used for motion timing.
    void setClock(IClock& clock);

#ifdef ARDUINO
    // Select the PCA9685 I2C address and servo PWM frequency (defaults 0x40 @
    // 50 Hz). Optional — only call it if your board differs from the defaults.
    void usePCA9685(uint8_t i2cAddress = kDefaultPca9685Address,
                    float pwmFreqHz = kDefaultServoFreqHz);

    // Channel-based joint registration: builds a PCA9685-backed output for you.
    bool addShoulder(uint8_t channel);
    bool addElbow(uint8_t channel);
    bool addWrist(uint8_t channel);
    bool addJoint(uint8_t channel);
    bool addGripper(uint8_t channel, float openDeg, float closeDeg);

    // Access the shared PCA9685 board wrapper.
    ServoDriver& board() { return m_board; }
#endif

    // --- configuration ---
    // Servo mechanical travel in degrees (default 180 for the RDS3225).
    void setServoTravel(float degrees);
    // The three link lengths in millimeters.
    void setLinkLengths(float l1Mm, float l2Mm, float l3Mm);
    // Approach angle used by the 2-argument moveTo().
    void setDefaultApproachAngle(float approachDeg);
    // Cap joint speed (deg/sec); enables slew-rate smoothing on every joint. A
    // value of 0 or less turns smoothing back off (joints move instantly).
    void setMaxSpeed(float degPerSec);
    // Hook for hardware start-up. On host this is a harmless no-op.
    void begin();

    // --- joint registration (inject an output; channel overloads come in P10) ---
    bool addShoulder(IServoOutput& output);
    bool addElbow(IServoOutput& output);
    bool addWrist(IServoOutput& output);
    bool addJoint(IServoOutput& output);
    bool addGripper(IServoOutput& output, float openDeg, float closeDeg);

    // --- motion ---
    // Solve IK for (x, y, approachDeg), command the joints, and push it out.
    // Returns false (and clamps to the nearest reachable pose) if unreachable.
    bool moveTo(float xMm, float yMm, float approachDeg);
    // 2-arg form uses the default approach angle.
    bool moveTo(float xMm, float yMm);
    // Command the three joint angles directly, bypassing IK (still limit-clamped).
    void setJointAngles(float shoulderDeg, float elbowDeg, float wristDeg);
    // Advance motion shaping and refresh every servo. Call each loop().
    void update();

    // --- gripper ---
    void openGripper();
    void closeGripper();
    void setGripper(float fraction);

    // --- queries ---
    // Is (x, y, approachDeg) reachable? (Does not command anything.)
    bool reachable(float xMm, float yMm, float approachDeg) const;
    // Forward kinematics of the current joint angles.
    ToolPose currentPose() const;
    // The most recent IK result.
    JointAngles lastSolution() const { return m_lastSolution; }
    bool lastReachable() const { return m_lastSolution.reachable; }

    // --- component access (the convenience layer is transparent) ---
    ArmKinematics& kinematics() { return m_kin; }
    const ArmKinematics& kinematics() const { return m_kin; }
    Joint& joint(int index) { return m_joints[index]; }
    const Joint& joint(int index) const { return m_joints[index]; }
    Gripper& gripper() { return m_gripper; }
    int jointCount() const { return m_jointCount; }
    bool hasGripper() const { return m_hasGripper; }

protected:
    // Attach an output at a fixed joint index, applying travel + smoothing.
    bool attachJointAt(int index, IServoOutput& output);

    ArmKinematics m_kin;
    Joint m_joints[kMaxJoints];
    SlewRateLimiter m_slews[kMaxJoints];
    Gripper m_gripper;
    IClock* m_clock;
    int m_jointCount;
    bool m_hasGripper;
    float m_travelDeg;
    float m_defaultApproachDeg;
    float m_maxSpeed;
    JointAngles m_lastSolution;

#ifdef ARDUINO
    // The shared board + owned per-channel outputs for the convenience path.
    ServoDriver m_board;
    Pca9685Servo m_outputs[kMaxJoints];
    Pca9685Servo m_gripperOutput;
#endif
};

}  // namespace roboarm
