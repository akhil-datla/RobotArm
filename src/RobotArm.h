// RobotArm.h — umbrella header. The ONLY include a sketch needs:
//
//     #include <RobotArm.h>
//     using namespace roboarm;
//
// It pulls in every public class so a beginner includes nothing else. Public
// types all live in namespace roboarm. See CLAUDE.md sections 5.1 and 7.4.
#pragma once

// ---- pure core (always available, host-testable) --------------------------
#include "core/MathUtils.h"
#include "core/Vec2.h"
#include "core/PIDController.h"
#include "core/SlewRateLimiter.h"
#include "core/TrapezoidProfile.h"
#include "core/ServoCalibration.h"
#include "core/Kinematics.h"
#include "core/Kinematics3D.h"

// ---- hardware seams (interfaces, no Arduino) ------------------------------
#include "hal/IServoOutput.h"
#include "hal/IClock.h"

// ---- public component + convenience layer ---------------------------------
#include "Joint.h"
#include "Gripper.h"
#include "Arm.h"
#include "Arm3D.h"

// ---- Arduino-only implementations -----------------------------------------
// These pull in Adafruit_PWMServoDriver.h / Wire.h, so they are guarded so the
// umbrella stays harmless if ever included off-device. On the Arduino toolchain
// ARDUINO is defined and these provide the real PCA9685 hardware path plus the
// channel-based convenience overloads.
#ifdef ARDUINO
#include "arduino/ServoDriver.h"
#include "arduino/Pca9685Servo.h"
#include "arduino/ArduinoClock.h"
#endif
