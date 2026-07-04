# RobotArm

A beginner-friendly Arduino library for standing up a **2–4 joint planar robot
arm** with a handful of friendly function calls. Reach to a point in millimeters
with inverse kinematics, drive high-torque servos through a PCA9685 board, and
move smoothly with built-in PID and slew-rate limiting — all the hard trig and
pulse math lives behind a tiny, safe API you can learn one component at a time.

It is a **glass box, not a black box**: the library does the tedious, error-prone
work (the inverse-kinematics trig, the pulse-width math, the control-loop
arithmetic, the timing) but keeps every idea visible as its own component you use
on purpose. You still run inverse kinematics and read the joint angles back, still
command each joint, still run the control loop in your own `loop()`.

---

## Contents
1. [What you need](#what-you-need)
2. [Installing the library](#installing-the-library)
3. [Wiring the PCA9685 and servos](#wiring-the-pca9685-and-servos)
4. [The reference servo: DSSERVO RDS3225 (180°)](#the-reference-servo-dsservo-rds3225-180)
5. [The coordinate frame](#the-coordinate-frame)
6. [Quickstart](#quickstart)
7. [Calibration walkthrough](#calibration-walkthrough)
8. [API cheat-sheet](#api-cheat-sheet)
9. [Troubleshooting](#troubleshooting)
10. [Examples](#examples)

---

## What you need

- An **Arduino** (Uno or similar).
- A **PCA9685** 16-channel PWM/servo driver board.
- 2–4 hobby servos. The library is tuned for the **DSSERVO RDS3225 (180°)**.
- A **separate 5–6 V power supply** for the servos, sized for their combined
  stall current (RDS3225 ≈ 2.9 A each at stall). **Never power these servos from
  the Arduino's 5 V pin.**
- The **Adafruit PWM Servo Driver Library** (see below).

---

## Installing the library

The library installs and imports like any other Arduino library, so the first
line of your sketch is simply:

```cpp
#include <RobotArm.h>
using namespace roboarm;   // one friendly line so you can type Joint, RobotArm, ...
```

Install it any of three ways:

1. **Copy/symlink** the whole `RobotArm/` folder into your sketchbook
   `libraries/` directory (e.g. `~/Documents/Arduino/libraries/RobotArm/`), then
   restart the IDE.
2. **Zip and add:** zip the `RobotArm/` folder and use
   *Sketch → Include Library → Add .ZIP Library…*.
3. **CLI:** `arduino-cli lib install --zip-path RobotArm.zip`, or point the
   IDE/CLI at the repo with a sketchbook symlink during development.

### Required dependency

Install the **Adafruit PWM Servo Driver Library** from the Library Manager
(*Tools → Manage Libraries…*, search "Adafruit PWM Servo Driver"). Installing it
automatically pulls in **Adafruit BusIO**, which it uses for I²C. The built-in
**Wire** library is already part of the Arduino core.

---

## Wiring the PCA9685 and servos

```
   Arduino                 PCA9685                     Servo supply (5-6 V, several amps)
   -------                 -------                     ----------------------------------
   SDA  ------------------ SDA
   SCL  ------------------ SCL
   5V   ------------------ VCC   (logic power only)
   GND  ------+----------- GND   (logic ground)
             |             V+  ------------------------ (+)  5-6 V   <-- servo power
             |             GND ------------------------ (-)  GND
             |                                           |
             +-------------------------------------------+   <-- STAR GROUND: tie
                 Arduino GND == PCA9685 GND == supply GND     all three together

   Add a bulk capacitor (~470-1000 uF) across the PCA9685 V+ / GND terminals to
   absorb current spikes. Servos plug into channels 0..15 (signal/V+/GND headers).
```

Key points:

- **I²C:** `SDA`/`SCL` go to the Arduino's I²C pins (Uno: A4 = SDA, A5 = SCL).
- **Logic vs. servo power are separate.** `VCC` powers the PCA9685's chip from the
  Arduino 5 V. `V+` (the screw terminal) powers the *servos* from your external
  5–6 V supply (use ~6 V for full torque). These are different pins.
- **Common ground.** Tie Arduino GND, PCA9685 GND, and the supply GND together at
  one star point, or nothing will work reliably.
- **Bulk capacitor.** A 470–1000 µF capacitor across `V+`/GND absorbs the current
  spikes that cause brown-outs and jitter. Use thick wire for the power run.
- **Address jumpers.** The board defaults to I²C address **0x40**. Solder the
  address jumpers to change it (e.g. for a second board). Pass your address to
  `ServoDriver(address, freqHz)` or `arm.usePCA9685(address, freqHz)`.
- **Channel numbering.** Channels are `0`–`15`, silk-screened on the board. You
  bind a joint to a channel by its number.

---

## The reference servo: DSSERVO RDS3225 (180°)

The library's defaults target the **RDS3225, 180° control-angle version**:

| Property | Value |
|---|---|
| Signal | standard PWM, **500–2500 µs**, **neutral 1500 µs** |
| Refresh rate | **50 Hz** |
| Travel | full 500–2500 µs sweep = **180°** (≈ 11.1 µs/degree) |
| Default map | `0° → 500 µs`, `90° → 1500 µs`, `180° → 2500 µs` |
| Operating voltage | 4.8–6.8 V (use ~6 V for full torque) |
| Stall current | ≈ **2.9 A** per servo |

**Confirm your variant.** The RDS3225 also ships as a **270°** unit. If yours is
270°, keep the 500–2500 µs range but tell the library the travel:
`joint.setTravel(270)` (or `arm.setServoTravel(270)`), and widen the soft limits
to match. Everything else stays the same.

Real units often reach their mechanical stops slightly *inside* 500/2500 µs (e.g.
~550–2450), where they buzz and stall. **Calibration** (below) finds and stays
just inside each unit's true endpoints.

---

## The coordinate frame

The arm is a **3R planar manipulator**: three revolute joints — **shoulder →
elbow → wrist** — all moving in one **vertical plane**, plus a **gripper** that
only opens/closes (not part of IK).

```
        +Y (up)
         ^
         |            (wrist)o----o tip / gripper
         |               .  L3
         |          o . '
         |      L2 /  (elbow)
         |        /
         |   L1  /
         |      /
   (shoulder)  o------------------------> +X (horizontal, "forward")
              origin

   theta1 (shoulder): measured from +X, positive lifting toward +Y
   theta2 (elbow):    joint angle, 0 = forearm straight in line with upper arm
   theta3 (wrist):    joint angle, 0 = hand straight in line with forearm
   phi (approach):    absolute hand direction from +X = theta1 + theta2 + theta3
                      phi =   0  -> gripper points horizontally forward
                      phi = -90  -> gripper points straight down (grab from above)
                      phi = +90  -> gripper points straight up
```

- Origin at the **shoulder pivot**. Targets are `(x, y)` in **millimeters**.
- Links: `L1` upper arm (shoulder→elbow), `L2` forearm (elbow→wrist), `L3` hand
  (wrist→gripper tip).
- **Public units are degrees and millimeters.** (Radians are used only internally.)

---

## Quickstart

There are **two ways** to use the library. The **component way** is primary — you
assemble the parts and see each concept.

### (A) The component way

```cpp
#include <RobotArm.h>
using namespace roboarm;

ServoDriver board(0x40, 50);              // the PCA9685
Joint shoulder(board, 0);                 // a Joint = channel + calibration + limits
Joint elbow(board, 1);
Joint wrist(board, 2);
Gripper gripper(board, 3, 30, 120);       // open at 30 deg, closed at 120 deg
ArmKinematics kin(100, 100, 60);          // link lengths L1, L2, L3 in mm

void setup() {
  board.begin();
  shoulder.setPulseRange(500, 2500); shoulder.setTravel(180); shoulder.setLimits(0, 180);
  elbow.setPulseRange(500, 2500);    elbow.setTravel(180);    elbow.setLimits(0, 180);
  wrist.setPulseRange(500, 2500);    wrist.setTravel(180);
  // The wrist bends BACK past its own zero to point the hand down/level, so its
  // angle goes negative. Center its 180-deg range with an offset so theta3 =
  // -180..0 maps onto the servo's 0..180 (else the wrist silently clamps to 0).
  wrist.setOffset(180);              wrist.setLimits(-180, 0);
}

void loop() {
  JointAngles a = kin.solve(150, 60, -90);   // STEP 1: inverse kinematics
  if (a.reachable) {
    shoulder.setAngle(a.shoulder);           // STEP 2: command the joints
    elbow.setAngle(a.elbow);
    wrist.setAngle(a.wrist);
  }
  shoulder.update(); elbow.update(); wrist.update();   // STEP 3: push it out
  gripper.close();
}
```

### (B) The short way

`RobotArm` composes the *same* Joints, ArmKinematics, and Gripper for you.

```cpp
#include <RobotArm.h>
using namespace roboarm;

RobotArm arm;

void setup() {
  arm.usePCA9685(0x40, 50);
  arm.setServoTravel(180);
  arm.addShoulder(0); arm.addElbow(1); arm.addWrist(2); arm.addGripper(3, 30, 120);
  arm.setLinkLengths(100, 100, 60);
  // Wrist bends back for downward/level grasps -> negative angle. Center it (drop
  // to the parts via joint(2) to configure the same Joint the arm uses).
  arm.joint(2).setOffset(180); arm.joint(2).setLimits(-180, 0);
  arm.begin();
}

void loop() {
  arm.moveTo(150, 60, -90);   // = solve IK, command the joints, update — for you
  arm.closeGripper();
  arm.update();
}
```

`moveTo` returns `true` only if the hand actually reached the pose. It returns
`false` — and still moves to the nearest safe pose — when the target is
kinematically unreachable *or* when a joint's soft limit clamps the solution.
Drop down to the parts any time with `arm.kinematics()`, `arm.joint(i)`, and
`arm.gripper()`.

---

## Calibration walkthrough

Each servo is a little different. Calibration teaches a `Joint` the true map
between angle and pulse. See **example 02** for a guided version.

1. Upload example `02_CalibrateJoint` and open the Serial Monitor (115200 baud).
2. It sweeps the raw pulse from 500 → 2500 µs. **Watch the servo.** Note the pulse
   where it *just* reaches each mechanical end without buzzing or straining (often
   ~550 and ~2450 µs).
3. Put those two numbers into `joint.setPulseRange(trueMin, trueMax)`.
4. Set the mounting direction: `joint.setDirection(+1)` (or `-1` if the joint
   moves the wrong way), and `joint.setOffset(deg)` if the servo's zero doesn't
   line up with the joint's zero.
5. Command `joint.setAngle(90)` and check `joint.commandedPulseUs()` — it should
   be near the midpoint (~1500 µs) if your endpoints are symmetric.
6. Set safe soft limits with `joint.setLimits(minDeg, maxDeg)` so a command can
   never grind the servo against a stop.

---

## API cheat-sheet

**`ServoDriver(address = 0x40, freqHz = 50)`** — the PCA9685 board.
`begin()`, `writeChannelMicroseconds(ch, us)`.

**`Joint(board, channel)`** — a servo channel + calibration + soft limits.
`setPulseRange(minUs, maxUs)`, `setTravel(deg)`, `setLimits(minDeg, maxDeg)`,
`setDirection(±1)`, `setOffset(deg)`, `setAngle(deg)`, `update()`, `currentDeg()`,
`commandedPulseUs()`, `useSmoothing(SlewRateLimiter&)`, `usePID(PIDController&)`,
`setFeedback(deg)`.

**`Gripper(board, channel, openDeg, closeDeg)`** — a Joint with two positions.
`open()`, `close()`, `set(0..1)`.

**`ArmKinematics(L1, L2, L3)`** — the geometry.
`solve(xMm, yMm, approachDeg) → JointAngles {reachable, clamped, shoulder, elbow,
wrist}`, `forward(shoulderDeg, elbowDeg, wristDeg) → ToolPose {x, y, approachDeg}`.

**`PIDController(kP, kI, kD)`** — a control loop you run yourself.
`calculate(setpoint, measurement, dtSeconds)`, `setOutputLimits(min, max)`,
`setIntegratorLimits(min, max)`, `setTolerance(t)`, `atSetpoint()`, `reset()`.

**`SlewRateLimiter(maxUnitsPerSec)`** — cap how fast a value changes.
`calculate(target, dtSeconds)`, `reset(value)`.

**`TrapezoidProfile({maxVelocity, maxAcceleration})`** — velocity/accel-limited move.
`calculate(t, start, goal) → State {position, velocity}`, `totalTime(...)`,
`isFinished(...)`.

**`ServoCalibration`** — the angle↔pulse map (used inside Joint).
`setPulseRange`, `setTravel`, `setDirection`, `setOffset`, `setAngleLimits`,
`angleToPulseUs(deg)`, `clampAngle(deg)`.

**`RobotArm`** — the convenience layer (built from the parts above).
`usePCA9685(address, freqHz)`, `setServoTravel(deg)`, `setLinkLengths(L1, L2, L3)`,
`setDefaultApproachAngle(deg)`, `setMaxSpeed(degPerSec)`,
`addShoulder/addElbow/addWrist(channel)`, `addJoint(channel)`,
`addGripper(channel, openDeg, closeDeg)`, `begin()`,
`moveTo(x, y, approachDeg)`, `moveTo(x, y)`, `setJointAngles(sh, el, wr)`,
`update()`, `openGripper()/closeGripper()/setGripper(0..1)`,
`reachable(x, y, approachDeg)`, `currentPose()`, `kinematics()`, `joint(i)`,
`gripper()`.

---

## Troubleshooting

| Symptom | Likely fix |
|---|---|
| **Arm reaches the wrong direction** | Flip the joint's mounting direction: `joint.setDirection(-1)`. Adjust `setOffset(deg)` if its zero is off. |
| **Wrist points the wrong way / won't go "down"** | Pointing the hand down or level needs a large *negative* wrist angle. Center the wrist's range: `wrist.setOffset(180); wrist.setLimits(-180, 0);` (see examples 03/04). Without it the wrist clamps to 0. |
| **Servo jitter / the Arduino resets (brown-out)** | Use a **separate servo supply** (5–6 V, several amps) on `V+`, add the bulk capacitor, and tie all grounds together. Do not power servos from the Arduino. |
| **Nothing moves** | Check the I²C address (`0x40`?) and `SDA`/`SCL` wiring; make sure you called `board.begin()` / `arm.begin()`; confirm the servo is on the channel you configured. |
| **Won't reach a point** | `solve()`/`moveTo()` returned `reachable == false`. Check your link lengths (`L1, L2, L3`) and that the target is within `|L1−L2| .. L1+L2+L3`. Try a different approach angle. |
| **Servo buzzes/strains at the ends** | Your true endpoints are inside 500/2500 µs. Re-run calibration and use `setPulseRange(trueMin, trueMax)` and tighter `setLimits(...)`. |
| **Motion is too jerky** | Add smoothing: `arm.setMaxSpeed(deg/s)` (or `joint.useSmoothing(SlewRateLimiter)`). |

---

## Examples

| Sketch | Teaches |
|---|---|
| `01_SweepSingleServo` | What a `Joint` is; how degrees become a pulse. |
| `02_CalibrateJoint` | Finding a servo's true endpoints; the pulse↔angle map. |
| `03_WristAngle` | Forward kinematics and the approach angle. |
| `04_MoveToXY` | Inverse kinematics as a visible step; the component way, then the `RobotArm` short version. |
| `05_SmoothMotionPID` | Slew-rate smoothing and running a `PIDController` yourself. |

---

## Developing / running the host tests

All the math and control logic is unit-tested on your computer (no hardware, no
Arduino) with a vendored copy of [doctest](https://github.com/doctest/doctest):

```sh
cd test
make            # runs the core-purity check, then the full suite
```

`src/core/` is provably free of any Arduino symbols (enforced by
`make check-core-purity`), which is what lets the kinematics, PID, and motion
math be tested on a laptop.

---

## License

MIT — see [LICENSE](LICENSE).
