// ServoCalibration.h — the angle<->pulse mapping, made explicit.
//
// THE IDEA: a hobby servo doesn't understand "degrees" — it understands a pulse
// width. A short pulse (~500 us) parks it at one end of its travel, a long pulse
// (~2500 us) at the other. Calibration is the straight-line map between the joint
// angle you want (degrees) and the pulse the servo needs (microseconds), plus
// three per-servo mounting facts:
//   direction (+1/-1)  — is the servo mounted forwards or backwards?
//   offsetDeg          — added to the commanded angle to trim the mounting; a
//                        positive offset acts as if you commanded a larger angle.
//   minDeg/maxDeg      — soft limits so a command can never drive past the stops.
// It is pure math (no hardware), so the whole thing is host-tested.
//
// Defaults target the DSSERVO RDS3225 (180-degree version): 0 deg -> 500 us,
// 180 deg -> 2500 us, neutral 1500 us. See CLAUDE.md sections 4.4, 11, 14.
#pragma once

namespace roboarm {

// RDS3225 (180-degree) reference values. Real units often reach their stops a
// little inside 500/2500 us, so calibration lets each one be tuned per unit.
constexpr float kServoTravelDeg    = 180.0f;   // full sweep of the 180-deg unit
constexpr float kRds3225MinPulseUs = 500.0f;   // pulse at 0 deg
constexpr float kRds3225MaxPulseUs = 2500.0f;  // pulse at full travel
constexpr float kRds3225NeutralUs  = 1500.0f;  // pulse at 90 deg (mechanical center)

// The linear angle<->pulse map plus mounting direction, offset, and soft limits.
class ServoCalibration {
public:
    // Constructs with the RDS3225 (180-degree) defaults.
    ServoCalibration();

    // Set the pulse endpoints (microseconds) that correspond to 0 deg and full
    // travel. Use example 02 to find each servo's true mechanical endpoints.
    void setPulseRange(float minPulseUs, float maxPulseUs);

    // Set the mechanical travel that the full pulse range sweeps (180 default;
    // 270 for the 270-degree RDS3225 variant).
    void setTravel(float travelDeg);

    // Mounting direction: +1 normal, -1 if the servo is mounted so that a bigger
    // joint angle needs a shorter pulse. Any non-negative value means +1.
    void setDirection(int direction);

    // Mounting trim (degrees), added to the commanded joint angle before mapping —
    // a positive offset behaves as if you had commanded that much larger an angle
    // (which way the pulse then moves depends on the mount direction). Use it when
    // the servo's zero doesn't line up with the joint's zero.
    void setOffset(float offsetDeg);

    // Soft limits (degrees). Commanded angles are clamped into [minDeg, maxDeg]
    // before mapping, so a command can never push the servo past a safe range.
    void setAngleLimits(float minDeg, float maxDeg);

    // Convert a commanded joint angle (degrees) to a pulse width (microseconds).
    // Applies, in order: soft-limit clamp on the angle, mounting offset/direction,
    // the linear map, and finally a clamp of the pulse to the microsecond bounds.
    float angleToPulseUs(float angleDeg) const;

    // Clamp an angle to the soft limits (used by Joint to report currentDeg()).
    float clampAngle(float angleDeg) const;

    // --- read-back accessors ---
    float minPulseUs() const { return m_minUs; }
    float maxPulseUs() const { return m_maxUs; }
    float travelDeg() const { return m_travelDeg; }
    int   direction() const { return m_direction; }
    float offsetDeg() const { return m_offsetDeg; }
    float minDeg() const { return m_minDeg; }
    float maxDeg() const { return m_maxDeg; }

private:
    float m_minUs;
    float m_maxUs;
    float m_travelDeg;
    int   m_direction;  // +1 or -1
    float m_offsetDeg;
    float m_minDeg;
    float m_maxDeg;
};

}  // namespace roboarm
