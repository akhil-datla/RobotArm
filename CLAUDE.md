# CLAUDE.md — `RobotArm` Arduino Library

> **How to use this document (read first):** You are building an Arduino library
> called **RobotArm** that lets a high-school student stand up a multi-servo
> robot arm with a handful of friendly function calls. Work through the
> **phased build plan** in order. Do **not** skip the TDD loop. Do **not** move
> to the next phase until every test in the current phase passes and the host
> build is clean. If a phase's tests cannot be made to pass, stop and explain
> why rather than weakening the test. Treat the verified math vectors in
> Appendix A as ground truth.

---

## 1. Mission

Build a small, dependable, well-documented Arduino library that makes it *easy*
for a beginner to:

1. **Set up servos** for each joint of a robot arm (PCA9685 channels, limits,
   calibration).
2. **Move the arm in its plane** using inverse kinematics — "reach to (x, y) in
   millimeters with the gripper at a given angle" — instead of computing joint
   angles by hand.
3. **Control motion smoothly and safely** using a PID controller and motion
   shaping (slew-rate limiting / trapezoidal profiles).

The student should be able to wire up a 2–4 joint arm, paste a short sketch, and
have it move. All the hard math lives behind a tiny, obvious public API.

---

## 2. Design audience

The end user is a **high-school robotics student**, likely their first time
writing C++. Optimize every public-facing decision for them:

- **Glass box, not black box.** The library does the *hard, tedious, error-prone*
  work — the inverse-kinematics trig, the pulse-width math, the PID arithmetic,
  the `dt` timing — but it does **not** hide the ideas behind it. The student
  still runs inverse kinematics and gets joint angles back, still commands each
  joint, still runs a PID loop in their own `loop()`. Remove the drudgery, keep
  the understanding.
- **Components first.** The primary surface is a set of small, named,
  single-purpose classes that each map to one real idea (a servo driver, a joint,
  inverse kinematics, a PID controller, a gripper). The student composes them and
  learns what each does. A convenience `RobotArm` that wires them together exists
  for the short path, but it is built *on top of* those same public components —
  never a parallel black box.
- Use **degrees** and **millimeters** in the public API. Nobody types radians.
- Provide **sane defaults** so a minimal sketch works.
- Fail **safely and loudly**: clamp to limits, never command a servo out of
  range, make unreachable targets obvious instead of silently wrong.
- Every public class and method gets a one-line doc comment a beginner can read.

The deeper plumbing — the HAL interfaces, the pure-math internals, the test
fakes — is not student-facing; keep it clean, layered, and testable. But the
**component layer above it is**, and it should read like a clearly labeled set of
parts a student can pick up one at a time, not a sealed appliance.

---

## 3. Guiding principles (WPILib-inspired)

We are deliberately modeling the *developer experience* of WPILib (the FRC
library), adapted to Arduino and hobby servos:

- **Teach, don't hide (glass box).** Make the hard/tedious parts easy — the trig,
  the pulse math, the control-loop arithmetic, the timing — but keep each
  *concept* visible as its own component the student uses on purpose. Prefer
  exposing the steps ("solve IK → read the joint angles → command the joints")
  over collapsing them into one opaque call. `RobotArm.moveTo()` may exist as
  convenience, but it must be nothing more than those same public components
  wired together.
- **Composable, single-purpose classes.** `ServoDriver`, `Joint`, `Gripper`,
  `ArmKinematics`, `PIDController`, `SlewRateLimiter`, `TrapezoidProfile` each
  model one real idea and snap together. A student builds an arm out of parts they
  understand.
- **Stable, discoverable public API.** Once a public method exists, treat its
  signature as a contract. Beginners copy examples; don't break them casually.
- **Separation of policy from hardware.** Control math and kinematics know
  nothing about `Adafruit_PWMServoDriver` or any servo hardware. Hardware is an
  injected dependency behind an interface. (This is also what makes the whole
  thing testable on a laptop.)
- **Safety is built in, not bolted on.** Soft limits, output clamping, and
  reachability checks are part of the core, not the student's responsibility.
- **Units are explicit and consistent.** Degrees + millimeters at the boundary;
  radians internally; convert exactly once at each edge.

---

## 4. Non-negotiable constraints

These are hard rules. Do not violate them without stopping to ask.

### 4.1 Tooling minimalism
- **Host unit tests** require only: a C++ compiler (`g++` or `clang++`),
  `make`, and **one vendored single-header test framework** committed into the
  repo. Use **doctest** (`test/doctest.h`, MIT-licensed, one file). No package
  managers, no CMake, no Python, no Node, no network fetches.
- **Arduino build/upload** uses the standard **Arduino IDE** or `arduino-cli`.
  Nothing else. (`arduino-cli` is optional, only for scripted compile checks.)
- The library's required Arduino dependency is the **`Adafruit PWM Servo Driver
  Library`** (header `Adafruit_PWMServoDriver.h`) plus the built-in **`Wire`**
  library it uses for I²C. Servos are driven through a **PCA9685** 16-channel PWM
  board over I²C — this is the standard, core hardware path, not an option.
  Installing the Adafruit library via the Library Manager also pulls in
  **`Adafruit BusIO`** automatically. Do not add any further dependency without
  stopping to justify.

If you think a new tool or dependency is genuinely needed, stop and justify it
before adding it.

### 4.2 The "pure core" rule
Everything under `src/core/` and the interfaces in `src/hal/` must compile on a
plain desktop with **no Arduino headers**. Specifically, files in `src/core/`
must **never** include `Arduino.h`, `Wire.h`, `Adafruit_PWMServoDriver.h`,
`Servo.h`, or use `String`, `Serial`, `millis()`, `delay()`, etc. They use only
standard C++.
- This rule is what lets us unit-test the math on the host.
- Enforce it mechanically with a Makefile target (`make check-core-purity`)
  that greps `src/core/` for forbidden symbols and fails if found.

### 4.3 Embedded reality (must run on an Arduino Uno)
The core must be friendly to small 8-bit AVR boards:
- **No dynamic memory** (`new`, `malloc`, `std::vector`, `std::string`). Use
  fixed-size arrays with a compile-time `kMaxJoints` (default 6).
- **No exceptions, no RTTI** (AVR builds disable them). Report errors via return
  values / status structs, never `throw`.
- **Prefer `float`** and the `f`-suffixed math functions (`sinf`, `cosf`,
  `atan2f`, `sqrtf`, `acosf`). AVR has no FPU; keep it lean. (`<cmath>` on host
  and `<math.h>` on AVR both provide these.)
- Keep RAM and flash footprint small; avoid large lookup tables.

### 4.4 Reference servo: DSSERVO RDS3225 (180° version)
The arm is built around the **DSSERVO RDS3225** high-torque digital servo, the
**180° control-angle version**. Bake its characteristics into the library
defaults and the docs:
- **Signal:** standard PWM, pulse width **500–2500 µs**, **neutral 1500 µs**,
  refresh **50 Hz** (digital; it tolerates up to ~330 Hz, but keep 50 Hz as the
  default). Dead band ~3 µs.
- **Travel:** the full 500–2500 µs sweep maps to **180°**, so neutral 1500 µs ≈
  90° and the scale is ≈ **11.1 µs per degree**. (The RDS3225 also ships as a
  270° version; this build uses the 180° one. Keep a named `constexpr
  kServoTravelDeg = 180` plus an optional `arm.setServoTravel(degrees)` so a 270°
  unit is one line away — but 180° is the default and what the examples assume.)
- **Default calibration** (a starting point; the student fine-tunes per unit):
  `kRds3225MinPulseUs = 500`, `kRds3225MaxPulseUs = 2500`, `kRds3225NeutralUs =
  1500`, giving the map `0° → 500 µs`, `180° → 2500 µs`. Real units often reach
  their mechanical stops slightly inside 500/2500 µs (e.g. ~550–2450), where they
  buzz and stall — so calibration finds and stays just inside each unit's true
  endpoints.
- **Power:** operating voltage **4.8–6.8 V** (use ~6 V for full torque); stall
  current is high (**≈ 2.9 A per servo**, ~24–28 kg·cm). This drives the power and
  wiring requirements in §14 — never power these from the Arduino's 5 V pin.
- **Resolution note:** at 50 Hz one PCA9685 step is ≈ 4.9 µs ≈ 0.4° with this
  mapping — finer than the arm needs, so PWM resolution is not a limiting factor.

---

## 5. Architecture overview

Four layers, depending only downward. Arrows = "depends on".

```
  ┌─────────────────────────────────────────────────────────┐
  │  PUBLIC API  (student-facing, friendly)                  │
  │    RobotArm, Joint                                       │
  └───────────────┬─────────────────────────────────────────┘
                  │ uses
  ┌───────────────▼─────────────────────────────────────────┐
  │  CORE  (pure C++, no Arduino — fully host-testable)      │
  │    PIDController, SlewRateLimiter, TrapezoidProfile,     │
  │    Kinematics (FK + IK), ServoCalibration, math utils    │
  └───────────────┬─────────────────────────────────────────┘
                  │ talks to hardware only through
  ┌───────────────▼─────────────────────────────────────────┐
  │  HAL  (interfaces only — pure virtual, no Arduino)       │
  │    IServoOutput, IClock                                  │
  └───────────────┬─────────────────────────────────────────┘
                  │ implemented by
  ┌───────────────▼─────────────────────────────────────────┐
  │  ARDUINO IMPL  (thin wrappers, the ONLY Arduino code)    │
  │    Pca9685Servo (Adafruit_PWMServoDriver, I2C)           │
  │    ArduinoClock (micros)                                 │
  └─────────────────────────────────────────────────────────┘
```

### 5.1 Architecture seams for testability (important — get this right)
The whole testing strategy rests on two injected interfaces:

- **`IServoOutput`** — abstract. One method conceptually: "command this output
  to an angle (degrees) within its range." The real `Pca9685Servo` drives one
  PCA9685 channel via `Adafruit_PWMServoDriver`; a `FakeServoOutput` (in
  `test/fakes/`) just records the last commanded value so tests can assert on it.
- **`IClock`** — abstract. Returns "now" in microseconds. `ArduinoClock` returns
  `micros()`; `FakeClock` returns a value the test sets and advances manually,
  so PID/motion timing is deterministic in tests.

`Joint` and `RobotArm` depend on these **interfaces**, never on concrete Arduino
types. Tests inject fakes; the student's sketch injects the real ones. Provide a
**student convenience path** so beginners don't have to construct interfaces:

- `RobotArm` (and `Joint`) accept injected `IServoOutput&` / `IClock&` for tests
  and advanced users.
- Under `#ifdef ARDUINO`, also provide convenience overloads
  (e.g. `arm.addJoint(uint8_t channel, ...)`) that internally construct a
  `Pca9685Servo` bound to the arm's shared `Adafruit_PWMServoDriver`, plus a
  default constructor that uses a shared `ArduinoClock`. These overloads, the
  shared driver, and `usePCA9685(...)` are the only things guarded by
  `#ifdef ARDUINO`; the core injectable methods are always present and always
  host-testable.

---

## 6. Repository layout

Create exactly this structure. (Arduino IDE compiles **only** `src/`; the
`test/` tree is invisible to it, which is why host tests live there.)

```
RobotArm/
├── CLAUDE.md                 # this file
├── README.md                 # student-facing intro + quickstart
├── library.properties        # Arduino library metadata
├── keywords.txt              # Arduino IDE syntax highlighting
├── LICENSE
├── src/                      # compiled by Arduino — pure C++ + thin Arduino
│   ├── RobotArm.h            # umbrella header — the ONLY include a sketch needs: <RobotArm.h>
│   ├── core/
│   │   ├── MathUtils.h       # deg/rad, clamp, lerp, wrapAngle
│   │   ├── Vec2.h
│   │   ├── PIDController.h / .cpp
│   │   ├── SlewRateLimiter.h / .cpp
│   │   ├── TrapezoidProfile.h / .cpp
│   │   ├── ServoCalibration.h / .cpp
│   │   └── Kinematics.h / .cpp        # FK + IK math + public ArmKinematics class
│   ├── hal/
│   │   ├── IServoOutput.h    # pure virtual interface
│   │   └── IClock.h          # pure virtual interface
│   ├── Joint.h / .cpp        # public component (servo + calibration + limits)
│   ├── Gripper.h / .cpp      # public component (a Joint with open/close positions)
│   ├── RobotArm.h is umbrella; convenience RobotArm in Arm.h / .cpp (public)
│   └── arduino/              # ONLY place Arduino.h / Wire.h / Adafruit_* appear
│       ├── ServoDriver.h / .cpp        # public PCA9685 board wrapper (Adafruit) (#ifdef ARDUINO)
│       ├── Pca9685Servo.h / .cpp       # IServoOutput for one channel            (#ifdef ARDUINO)
│       └── ArduinoClock.h / .cpp       # wraps micros()                          (#ifdef ARDUINO)
├── examples/                 # Arduino example sketches (one folder each)
│   ├── 01_SweepSingleServo/01_SweepSingleServo.ino
│   ├── 02_CalibrateJoint/02_CalibrateJoint.ino
│   ├── 03_WristAngle/03_WristAngle.ino
│   ├── 04_MoveToXY/04_MoveToXY.ino
│   └── 05_SmoothMotionPID/05_SmoothMotionPID.ino
└── test/                     # host-only — invisible to Arduino IDE
    ├── doctest.h             # vendored single-header framework
    ├── test_main.cpp         # #define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
    ├── test_mathutils.cpp
    ├── test_pid.cpp
    ├── test_slew.cpp
    ├── test_trapezoid.cpp
    ├── test_calibration.cpp
    ├── test_kinematics_fk.cpp
    ├── test_kinematics_ik.cpp
    ├── test_joint.cpp
    ├── test_arm.cpp
    ├── fakes/
    │   ├── FakeServoOutput.h
    │   └── FakeClock.h
    └── Makefile
```

> Note: avoid a real circular `RobotArm.h`/`Arm.h` naming clash. Make
> `RobotArm.h` the umbrella header that includes the public classes; put the arm
> class implementation in `Arm.h`/`Arm.cpp` with the public class named
> `RobotArm`. Keep includes acyclic.

---

## 7. Tooling & commands

### 7.1 Host test Makefile (`test/Makefile`)
Provide these targets. Keep it dependency-free.

- `make test` — compile all `test_*.cpp` + needed `src/core` + `src/hal` sources
  with `-std=c++17 -Wall -Wextra -Wpedantic`, link, run the binary. This is the
  command you run constantly during TDD.
- `make check-core-purity` — fail if `src/core/` contains any of:
  `Arduino.h`, `Wire.h`, `Adafruit_`, `Servo.h`, `Serial`, `String`, `millis`,
  `delay`, `analogWrite`, `digitalWrite`, `new `, `malloc`. (Grep-based; no tools.)
- `make clean` — remove build artifacts.
- Default goal runs `check-core-purity` then `test`.

Do **not** compile anything under `src/arduino/` in the host build — those files
include `Adafruit_PWMServoDriver.h`/`Wire.h`/`Arduino.h` and only ever build
inside the Arduino toolchain.

### 7.2 Arduino compile check (optional, scripted)
If `arduino-cli` is available, you may verify examples compile, e.g.
`arduino-cli compile --fqbn arduino:avr:uno examples/04_MoveToXY`. Treat this as
a nice-to-have CI step, not a hard requirement of local TDD.

### 7.3 Arduino metadata
- `library.properties`: `name=RobotArm`, semantic `version`, `author`,
  `maintainer`, a beginner-friendly `sentence`/`paragraph`, `category=Device
  Control`, `architectures=*`, `includes=RobotArm.h`,
  `depends=Adafruit PWM Servo Driver Library`.
- `keywords.txt`: `KEYWORD1` for classes (`RobotArm`, `Joint`, `PIDController`,
  …), `KEYWORD2` for methods, `LITERAL1` for public constants. Tab-separated.
  Generate/refresh this in the docs phase.

### 7.4 Importing the library (`#include <RobotArm.h>`)
The library must install and import like any other Arduino library, so the first
line of a sketch is simply:

    #include <RobotArm.h>

What makes this work:
- `src/RobotArm.h` is the **umbrella header** and the single public entry point.
  It `#include`s every public class a sketch needs (`RobotArm`, `Joint`,
  `PIDController`, `SlewRateLimiter`, …) so the student includes nothing else.
- Public types live in `namespace roboarm`; examples do `using namespace
  roboarm;` once.
- The Arduino-only headers the umbrella pulls in (`Pca9685Servo`, `ArduinoClock`,
  and therefore `Adafruit_PWMServoDriver.h`/`Wire.h`) are wrapped in
  `#ifdef ARDUINO`, so the umbrella stays harmless if ever included off-device.
  (Host tests still include specific `core/`/`hal/` headers directly, never the
  umbrella.)
- `library.properties` sits at the library root with `includes=RobotArm.h`;
  Arduino adds `src/` to the include path, which is what makes the angle-bracket
  form resolve.

Installation (document all three in the README):
1. **Copy/symlink** the whole `RobotArm/` folder into the sketchbook
   `libraries/` directory (e.g. `~/Documents/Arduino/libraries/RobotArm/`), then
   restart the IDE.
2. **Zip and add:** zip the `RobotArm/` folder and use *Sketch → Include Library
   → Add .ZIP Library…*.
3. **CLI:** `arduino-cli lib install --zip-path RobotArm.zip`, or during
   development point the IDE/CLI at the repo via a sketchbook symlink.

Also install the **Adafruit PWM Servo Driver Library** (the Library Manager pulls
in **Adafruit BusIO**). The README must state this dependency explicitly.

---

## 8. Coding conventions

- **Language:** C++17 on host; AVR-compatible subset (see §4.3). No exceptions,
  no RTTI, no dynamic allocation in shipped code.
- **Namespace:** wrap library code in `namespace roboarm { ... }`. Examples do
  `using namespace roboarm;` once at the top — one friendly line for students.
- **Naming:** `PascalCase` types, `camelCase` methods/variables, `kPascalCase`
  for `constexpr` constants, `m_` prefix for private members (or trailing
  underscore — pick one and be consistent everywhere).
- **Units in identifiers:** when a number's unit isn't obvious, name it so:
  `pulseUs`, `lengthMm`, `angleDeg`. Internal radians use `Rad` suffix.
- **Headers:** `#pragma once`. Keep includes minimal; forward-declare where you
  can. Core headers include only `<cmath>`/`<cstdint>` and sibling core headers.
- **Comments:** every public declaration gets a short doc comment aimed at a
  beginner. Internal math gets comments explaining the *why*, with a pointer to
  Appendix A for the derivation.
- **No magic numbers:** servo pulse bounds, default limits, epsilon tolerances
  are named `constexpr`s.
- **Const-correctness:** queries are `const`; pure functions are `static`/free
  functions in `Kinematics`/`MathUtils`.

---

## 9. The TDD workflow (how you must work)

For **every** unit of behavior, follow Red → Green → Refactor:

1. **Red.** Write a failing test that names the behavior precisely (one concept
   per `TEST_CASE`/`SUBCASE`). Run `make test`; watch it fail for the *right*
   reason.
2. **Green.** Write the *minimum* production code to make it pass. Run
   `make test`; confirm green.
3. **Refactor.** Clean up names, extract helpers, remove duplication — with
   tests green the whole time. Run `make test` again.
4. **Commit.** Small, descriptive commit. Then take the next behavior.

Rules:
- **Never** write production code before a failing test exists for it.
- Keep the host suite **fast** (sub-second). Run it after every change.
- Prefer **pure functions** and **dependency injection** so behavior is testable
  without hardware. If something is hard to test, that's a design smell — refactor
  the seam, don't skip the test.
- Each **phase ends** with: all tests green, `check-core-purity` green, public
  API doc-commented, a tagged/committed milestone, and (where relevant) a working
  example sketch added.
- Use **round-trip** and **property-style** tests for math (see §13).
- When a bug is found later, first write a failing test that reproduces it, then
  fix it (regression-test discipline).

---

## 10. Units & coordinate conventions

Pin these down before writing kinematics; tests depend on them.

- **Public units:** angles in **degrees**, lengths in **millimeters**.
- **Internal units:** **radians** for all trig. Convert exactly once at each API
  boundary (`MathUtils::degToRad` / `radToDeg`), and test those conversions.
- **The arm is a 3R planar manipulator** (no base): three revolute joints —
  **shoulder → elbow → wrist** — all moving in one **vertical plane**, plus a
  **gripper** servo that only opens/closes (not part of IK).
- **Coordinate frame:**
  - The plane is the **X–Y plane**: **+X points horizontally "forward"**, **+Y
    points up**. (There is no Z / no out-of-plane motion without a base.)
  - Origin at the **shoulder pivot**. Target positions are `(x, y)` in mm.
  - Three links: `L1` (upper arm, shoulder→elbow), `L2` (forearm, elbow→wrist),
    `L3` (hand, wrist→gripper reference point).
- **Joint-angle conventions for kinematics** (before per-servo calibration):
  - **Shoulder** `θ1` measured from **+X (horizontal)**, positive lifting toward
    +Y.
  - **Elbow** `θ2` is the joint angle between forearm and the extension of the
    upper arm; `θ2 = 0` means straight.
  - **Wrist** `θ3` is the joint angle between hand and the extension of the
    forearm.
  - **Approach angle** `φ` is the **absolute** orientation of the hand/gripper
    centerline from +X: `φ = θ1 + θ2 + θ3`. So `φ = 0°` → gripper points
    horizontally forward; `φ = −90°` → gripper points straight down (grab from
    above); `φ = +90°` → straight up.
  - Per-servo mounting (offset, direction, range) is applied **after** kinematics
    by `ServoCalibration` — keep kinematics frame-pure.

Document this frame in `Kinematics.h` and `README.md` with the ASCII sketch
above. Beginners must be able to see which way is which.

---

## 11. Target public API (what success looks like)

These are **targets** to drive the design — you will arrive at them via TDD, and
exact signatures may refine slightly. The *feel* (tiny, declarative, safe) is the
spec.

There are **two ways to use the library**, and the docs and examples lead with the
first so the student learns the pieces:

**(A) The component way — primary. You assemble the parts and see each concept:**

```cpp
#include <RobotArm.h>
using namespace roboarm;

// Build the arm out of parts you understand — each one teaches one idea.
ServoDriver board(/*i2cAddress=*/0x40, /*pwmFreqHz=*/50);   // the PCA9685

// A Joint = one servo channel + its calibration + soft limits.
Joint shoulder(board, /*channel=*/0);
Joint elbow(board, /*channel=*/1);
Joint wrist(board, /*channel=*/2);
Gripper gripper(board, /*channel=*/3, /*openDeg=*/30, /*closeDeg=*/120);

// Inverse kinematics knows the arm's geometry: the 3 link lengths in mm.
ArmKinematics kin(/*L1=*/100, /*L2=*/100, /*L3=*/60);

void setup() {
  board.begin();
  // RDS3225 (180 deg): the full 500-2500us pulse maps to 0-180 deg.
  // (Calibrate each servo's real endpoints later — see example 02.)
  shoulder.setPulseRange(500, 2500); shoulder.setTravel(180); shoulder.setLimits(0, 180);
  elbow.setPulseRange(500, 2500);    elbow.setTravel(180);    elbow.setLimits(0, 180);
  wrist.setPulseRange(500, 2500);    wrist.setTravel(180);    wrist.setLimits(0, 180);
}

void loop() {
  // STEP 1 - Inverse kinematics: what joint angles put the hand at (x, y, phi)?
  JointAngles a = kin.solve(/*xMm=*/150, /*yMm=*/60, /*approachDeg=*/-90);

  if (a.reachable) {
    // STEP 2 - Command each joint. The Joint turns degrees into a pulse for you.
    shoulder.setAngle(a.shoulder);
    elbow.setAngle(a.elbow);
    wrist.setAngle(a.wrist);
  }

  // STEP 3 - update() pushes the (smoothed) targets out to the servos.
  shoulder.update();
  elbow.update();
  wrist.update();
  gripper.close();
}
```

**(B) The short way — convenience. `RobotArm` wires the exact same parts together:**

```cpp
#include <RobotArm.h>
using namespace roboarm;

RobotArm arm;   // composes Joints + ArmKinematics + Gripper internally

void setup() {
  arm.usePCA9685(0x40, 50);
  arm.setServoTravel(180);
  arm.addShoulder(0, 0, 180);
  arm.addElbow(1, 0, 180);
  arm.addWrist(2, 0, 180);
  arm.addGripper(3, 30, 120);
  arm.setLinkLengths(100, 100, 60);
  arm.begin();
}

void loop() {
  arm.moveTo(150, 60, -90);   // = solve IK, command the joints, update — for you
  arm.closeGripper();
  arm.update();
}
```

The public **components** (drive these out with tests). Each is a real concept the
student uses directly; `RobotArm` at the end just composes them:

- **`PIDController`** (WPILib-style, standalone):
  - `PIDController(float kP, float kI, float kD)`
  - `float calculate(float setpoint, float measurement, float dtSeconds)`
  - `void setOutputLimits(float min, float max)` (clamp + anti-windup)
  - `void setIntegratorLimits(float min, float max)`
  - `void setTolerance(float positionTolerance)`; `bool atSetpoint() const`
  - `void reset()`
- **`SlewRateLimiter`**: `SlewRateLimiter(float maxUnitsPerSec)`,
  `float calculate(float target, float dtSeconds)`, `void reset(float value)`.
- **`TrapezoidProfile`** (optional but recommended): velocity/accel-limited
  approach to a setpoint; query position/velocity at a time. Keep API small.
- **`ServoCalibration`**: the angle↔pulse mapping made explicit — a linear map
  between **joint angle (deg)** and **pulse (µs)** from two calibration points,
  plus `direction` (+1/−1), `offsetDeg`, and `minDeg`/`maxDeg` soft limits. Pure
  math, no hardware. Defaults target the **RDS3225 (180°)**: `0° → 500 µs`,
  `180° → 2500 µs`, neutral `1500 µs`.
- **`ServoDriver`**: thin, student-facing wrapper around the PCA9685 /
  `Adafruit_PWMServoDriver` — `ServoDriver(address, freqHz)`, `begin()`. Joints
  bind to it by channel. (This is the one place hardware shows up in the component
  path.)
- **`Joint`**: a servo channel + its calibration + soft limits, used directly by
  the student. `Joint(board, channel)`, `setPulseRange(minUs, maxUs)`,
  `setTravel(deg)`, `setLimits(minDeg, maxDeg)`, `setAngle(deg)`, `update()`,
  `currentDeg()`; optional `useSmoothing(SlewRateLimiter)` /
  `usePID(PIDController)` for smoothing or continuous-rotation feedback. Advanced
  users can hand it a `ServoCalibration` directly.
- **`Gripper`**: a `Joint` with two named positions — `Gripper(board, channel,
  openDeg, closeDeg)`, `open()`, `close()`, `set(0..1)`. Teaches that a gripper is
  just a servo you drive to two angles.
- **`ArmKinematics`**: the geometry made usable — `ArmKinematics(L1, L2, L3)`,
  `JointAngles solve(xMm, yMm, approachDeg)` returning `{reachable, clamped,
  shoulder, elbow, wrist}`, and `ToolPose forward(shoulder, elbow, wrist)`
  returning `{x, y, approachAngle}` so the student can check their work. The trig
  is done for them; the *step* (IK produces angles) stays visible.
- **`RobotArm`** (optional convenience — **composes the components above**, does
  not replace them): driver config (`usePCA9685(i2cAddress, pwmFreqHz)`, optional
  — defaults to `0x40` @ `50 Hz`; `setServoTravel(degrees)`, optional — defaults
  to `180` for the RDS3225), joint registration by PCA9685 channel
  (`addShoulder/addElbow/addWrist`, generic `addJoint`, plus `addGripper(channel,
  openDeg, closeDeg)`), `setLinkLengths(L1, L2, L3)`,
  `setDefaultApproachAngle(deg)`, `setMaxSpeed`, `begin`, `bool moveTo(x, y,
  approachDeg)` and a 2-arg `moveTo(x, y)` (both return `false` if unreachable and
  clamp to the nearest reachable pose), `setJointAngles(shoulder, elbow, wrist)`,
  `openGripper()/closeGripper()/setGripper(0..1)`, `update()`, reachability query,
  and pose/angle accessors. Internally it holds the same `Joint`, `ArmKinematics`,
  and `Gripper` objects — and exposes them (e.g. `arm.kinematics()`,
  `arm.joint(i)`) so a student can drop from the convenience layer down to the
  parts.
- **HAL**: `IServoOutput`, `IClock`; Arduino impls `Pca9685Servo`
  (`Adafruit_PWMServoDriver`-backed) and `ArduinoClock`.

Keep the surface this small. Resist adding knobs a beginner won't use.

---

## 12. Phased build plan

Each phase: **Goal → Write tests first → Implement → Definition of Done (DoD)**.
Finish a phase completely before starting the next.

### Phase 0 — Scaffolding & test harness
**Goal:** A repo where `make test` runs and passes a trivial test, with metadata
in place.
**Tests first:** add `test_main.cpp` and a `TEST_CASE("harness works")` asserting
`1 + 1 == 2`.
**Implement:** directory tree from §6; vendor `doctest.h`; write `test/Makefile`
with `test`, `check-core-purity`, `clean`; create `library.properties`,
`keywords.txt` (stub), `README.md` (stub), `LICENSE`, umbrella `RobotArm.h`
(empty include hub).
**DoD:** `make test` green; `make check-core-purity` green; committed.

### Phase 1 — Math utilities & `Vec2`
**Goal:** Tiny pure helpers everything else uses.
**Tests first:** `degToRad`/`radToDeg` exactness on 0/90/180/360 and round-trip;
`clamp` at/inside/outside bounds; `lerp` endpoints + midpoint; `wrapAngleDeg`
mapping e.g. 190°→−170°, −190°→170°, 360°→0°; `Vec2` length, add/sub, `atan2`
direction.
**Implement:** `core/MathUtils.h`, `core/Vec2.h`. `constexpr` where possible; use
`float` + `*f` math.
**DoD:** all edge cases covered and green; no Arduino symbols.

### Phase 2 — `PIDController`
**Goal:** A standalone, deterministic PID (the WPILib-style building block).
**Tests first (use explicit `dt`, no real clock):**
- Pure P: error 10, kP 2 → output 20; at setpoint → 0.
- Integral: constant error accumulates as `kI * error * dt` over N steps;
  integrator clamp prevents windup.
- Derivative: responds to change in error; zero when error constant.
- `setOutputLimits` clamps output both directions.
- `atSetpoint()` true within tolerance, false outside.
- `reset()` zeroes integral and previous-error state.
**Implement:** `core/PIDController.{h,cpp}`. Anti-windup via integrator limits;
derivative on error (or on measurement — pick one, document it, test it).
**DoD:** deterministic outputs match hand-computed values; green.

### Phase 3 — Motion shaping (`SlewRateLimiter`, then `TrapezoidProfile`)
**Goal:** Smooth, safe approach to targets so servos don't snap.
**Tests first:** `SlewRateLimiter`: output moves at most `maxRate*dt` per step,
reaches target eventually, handles direction reversal, `reset` sets value.
`TrapezoidProfile` (if included): never exceeds max velocity/accel; monotonic
approach; arrives at setpoint with ~zero velocity; symmetric in/out.
**Implement:** `core/SlewRateLimiter.{h,cpp}`,
`core/TrapezoidProfile.{h,cpp}`.
**DoD:** rate/accel limits provably respected in tests; green.

### Phase 4 — `ServoCalibration` + HAL interfaces + fakes
**Goal:** Pure angle↔pulse mapping with mounting + soft limits, and the seams
for hardware.
**Tests first:**
- Linear map from two points: e.g. (0°→500µs, 180°→2500µs) ⇒ 90°→1500µs;
  monotonic; clamps pulse to configured µs bounds. Confirm the RDS3225 defaults
  (`kRds3225MinPulseUs`/`kRds3225MaxPulseUs`, `kServoTravelDeg = 180`) produce
  exactly this map.
- `direction = −1` reverses; `offsetDeg` shifts; `minDeg/maxDeg` clamp the
  commanded angle.
- Define `IServoOutput`/`IClock` and a `FakeServoOutput`/`FakeClock`; a test
  drives a calibration→fake output and asserts the recorded command.
**Implement:** `core/ServoCalibration.{h,cpp}`, `hal/IServoOutput.h`,
`hal/IClock.h`, `test/fakes/*`.
**DoD:** mapping + limits exhaustively tested; fakes usable; green.

### Phase 5 — Forward kinematics (planar 2-link, then 3-link)
**Goal:** Compute joint/tip positions from joint angles. Build the 2-link case
first (it is the inner solver for IK), then the full 3-link chain that also
reports the tip `(x, y)` and the approach angle `φ = θ1+θ2+θ3`.
**Tests first (Appendix A vectors):**
- 2-link, symmetric `L1=L2=100`: `FK2(0°, 90°) = (100, 100)`;
  `FK2(0°, 0°) = (200, 0)`.
- 2-link, asymmetric `L1=120, L2=80`: `FK2(30°, 60°) = (103.923, 140.0)`.
- 3-link, `L1=L2=100, L3=60`: `FK3(30°, 60°, −30°) = tip (116.603, 201.962)`,
  `φ = 60°`.
- 3-link, asymmetric `L1=120, L2=80, L3=50`:
  `FK3(20°, 40°, 30°) = tip (152.763, 160.324)`, `φ = 90°`. (tolerance 1e-3)
**Implement:** `core/Kinematics.{h,cpp}` FK functions (radians internally; provide
a degrees-facing wrapper). Expose elbow/wrist joint positions too, for limits.
**DoD:** matches Appendix A within tolerance; green.

### Phase 6 — Inverse kinematics (2-link planar — the inner solver)
**Goal:** Compute `(θ1, θ2)` from a target point `(x, y)`, with elbow-up/down,
reachability, and clamping. This is the core solver the 3R layer (Phase 7) calls
on the wrist point.
**Tests first:**
- **Round-trip (the key test):** for many `(θ1, θ2)` in range, `FK2→IK2→FK2`
  returns the same `(x, y)` within tolerance (angles may differ between
  solutions, so assert on *position*, not angles).
- Exact vectors from Appendix A (incl. asymmetric).
- **Reachability:** `D > L1+L2` ⇒ `reachable = false`; with clamping enabled,
  result lies on the same ray at distance `L1+L2`. `D < |L1−L2|` ⇒ unreachable
  (dead zone).
- **Boundaries:** `D = L1+L2` ⇒ `θ2 = 0`; `D = |L1−L2|` ⇒ `θ2 = 180°`.
- **Elbow config:** requesting elbow-up vs elbow-down yields the correct branch;
  both `FK2` back to the target.
- **Numerical safety:** clamp `acos` argument to `[−1, 1]` so boundary inputs
  don't `NaN`.
**Implement:** 2-link IK in `Kinematics`. Return a small status struct, e.g.
`{ bool reachable; bool clamped; float theta1Rad; float theta2Rad; }`.
**DoD:** round-trip holds broadly; all edge cases green; never returns `NaN`.

### Phase 7 — 3R wrist decoupling → `(x, y, approach angle)`
**Goal:** Add the wrist so the API takes a tip target `(x, y)` plus an approach
angle `φ`. The trick: subtract the hand link to get the wrist point, solve the
2-link IK (Phase 6) for it, then set the wrist.
**Tests first (Appendix A):**
- `IK3(116.603, 201.962, φ=60°)` with `L1=L2=100, L3=60` ⇒ `shoulder = 30°,
  elbow = 60°, wrist = −30°` (tolerance 1e-2 deg).
- `IK3(152.763, 160.324, φ=90°)` with `L1=120, L2=80, L3=50` ⇒ `shoulder = 20°,
  elbow = 40°, wrist = 30°`.
- Confirm the decoupling: `xw = x − L3·cosφ`, `yw = y − L3·sinφ`; 2-link IK on
  `(xw, yw)`; `θ3 = φ − (θ1 + θ2)`.
- **Round-trip:** random reachable `(θ1, θ2, θ3)` → `FK3` → `IK3` → `FK3` returns
  the same tip and `φ`.
- **Approach angle affects reachability:** a tip reachable at one `φ` can be
  unreachable at another (the wrist point leaves the 2-link annulus). `IK3`
  reports `reachable = false` in that case.
**Implement:** `IK3` in `Kinematics` layering wrist decoupling over the 2-link
solver; return shoulder/elbow/wrist + reachable/clamped flags. Wrap the pure FK/IK
functions in a public **`ArmKinematics`** class (holds `L1,L2,L3`; `solve(x, y,
φ) → JointAngles`, `forward(...) → ToolPose`) — this is the student-facing
kinematics object, but keep the underlying functions free and host-tested.
**DoD:** decoupling + round-trip + exact vectors green.

### Phase 8 — `Joint`
**Goal:** A joint that takes a target angle, shapes motion, respects soft limits,
and drives an `IServoOutput` — tested with fakes + `FakeClock`.
**Tests first:**
- `setTargetDeg` beyond soft limits ⇒ commanded value clamped to limit.
- With a `SlewRateLimiter`, repeated `update(dt)` ramps the commanded angle
  toward target at the configured rate (advance `FakeClock`, assert
  `FakeServoOutput` history).
- `currentDeg()` tracks commanded value.
- (Optional CR-servo path) with feedback + `PIDController`, output drives error
  toward zero in a simulated loop.
**Implement:** `Joint.{h,cpp}` depending only on `IServoOutput`,
`ServoCalibration`, and optionally `SlewRateLimiter`/`PIDController`. No Arduino.
Give `Joint` the student-facing API from §11 on top of the injectable core
(`setPulseRange`/`setTravel`/`setLimits`/`setAngle`/`update`/`currentDeg`,
`useSmoothing`/`usePID`). Add a small **`Gripper`** component — a `Joint` with two
named angles and `open()`/`close()`/`set(0..1)` — and test it with fakes.
**DoD:** clamping + smoothing behaviors green under fakes.

### Phase 9 — `RobotArm` (optional convenience layer)
**Goal:** Compose the **public** components (`Joint`, `ArmKinematics`, `Gripper`)
into a one-call convenience for the short path — built from the same types a
student can use directly, not a parallel implementation. Fully host-tested with
fakes.
**Tests first (inject fake outputs + `FakeClock`):**
- After `setLinkLengths(L1, L2, L3)` and adding the 3 joints, `moveTo(x, y,
  approachDeg)` to a known reachable pose drives each fake output to the expected
  calibrated command (cross-check against Appendix A).
- `moveTo` to an unreachable target returns `false` and clamps to the nearest
  reachable pose (no out-of-range commands); a target reachable at one approach
  angle but not another is handled correctly.
- `setMaxSpeed` makes `update()` ramp joints over multiple ticks rather than
  jumping (advance `FakeClock`).
- `setJointAngles(...)` bypasses IK and commands angles directly (with limit
  clamping).
- Gripper open/close commands the gripper output to its configured endpoints.
- `arm.kinematics()` and `arm.joint(i)` return the same component objects the arm
  uses internally (the convenience layer is transparent, not a black box).
- Adding more than `kMaxJoints` is rejected gracefully (no overflow, clear
  failure signal).
**Implement:** `Arm.{h,cpp}` (public class `RobotArm`), umbrella `RobotArm.h`.
`RobotArm` must be implemented **using** the public `Joint`/`ArmKinematics`/
`Gripper` classes (compose them; do not reimplement the kinematics or pulse math)
and must expose them through accessors.
Core injectable methods always present; `#ifdef ARDUINO` convenience overloads
(channel-based `addJoint`, shared PCA9685 driver, default `ArduinoClock`) added
in Phase 10.
**DoD:** end-to-end arm behavior verified on host with zero hardware; green.

### Phase 10 — Arduino hardware layer (PCA9685 via Adafruit_PWMServoDriver)
**Goal:** The thin, real-hardware implementations + student convenience path,
driving all servos through a PCA9685.
**Tests first:** these are hardware wrappers, so host *unit* testing is limited;
verify the **seam** instead — host tests already confirm `RobotArm`/`Joint` work
through the `IServoOutput`/`IClock` interfaces with fakes. Add an `arduino-cli`
compile check for the examples if available. Any pure helper you factor out
(e.g. an angle→pulse-µs conversion) may still be host-tested; keep the actual
`Adafruit_PWMServoDriver` calls in the thin wrapper.
**Implement (guarded by `#ifdef ARDUINO`, only inside `src/arduino/`):**
- `arduino/Pca9685Servo.{h,cpp}`: implements `IServoOutput`. Holds a pointer to a
  shared `Adafruit_PWMServoDriver` and a channel index (0–15). On command,
  convert the joint angle to a pulse width (µs) via `ServoCalibration`, then call
  `driver.writeMicroseconds(channel, us)`. (If the installed library version
  lacks `writeMicroseconds`, compute ticks
  `ticks = round(us * 4096 * freqHz / 1e6)` and call
  `driver.setPWM(channel, 0, ticks)`.)
- `arduino/ArduinoClock.{h,cpp}`: implements `IClock` via `micros()`.
- On `RobotArm`: hold one shared `Adafruit_PWMServoDriver`. `usePCA9685(address,
  freqHz)` selects the I²C address (default `0x40`) and servo PWM frequency
  (default `50 Hz`). `begin()` calls `Wire.begin()`, `driver.begin()`,
  `driver.setOscillatorFrequency(kPca9685OscHz)`, then `driver.setPWMFreq(freqHz)`.
  Channel-based convenience overloads construct `Pca9685Servo`s bound to that
  driver.
- Expose the oscillator frequency as a named `constexpr kPca9685OscHz`
  (default `27000000`) with a comment that it may need per-board calibration for
  accurate pulse widths (the PCA9685's nominal 25 MHz often measures ~26–27 MHz).
**DoD:** examples compile for `arduino:avr:uno`; `check-core-purity` still green
(`Adafruit_*`/`Wire.h`/`Arduino.h` confined to `src/arduino/`); a real arm moves.

### Phase 11 — Examples, docs, polish
**Goal:** A student can go from zero to moving arm.
**Implement:**
- Example sketches `01`–`05` (sweep, calibrate, wrist angle, move-to-XY,
  smooth PID/profiled motion). Each is short, heavily commented, and matches the
  §11 style.
- `README.md`: what it is, wiring overview, the coordinate frame sketch,
  calibration how-to, quickstart, troubleshooting, API cheat-sheet.
- Finalize `keywords.txt` for every public class/method/constant.
- Doc-comment every public declaration.
**DoD:** all examples compile; README quickstart actually reflects the API; tests
still green.

### Phase 12 — Optional stretch (only if asked / time permits)
Keep each behind its own guard and **never** required by the core path:
- Support **multiple PCA9685 boards** (a second `Adafruit_PWMServoDriver` at a
  different I²C address) so an arm with many servos plus a gripper can exceed the
  16 channels of one board.
- 4th DOF wrist-pitch (extend kinematics to keep tool orientation).
- Persist calibration to EEPROM.
- Simple Serial command interface for live jogging/calibration.
Each stretch item follows the same TDD discipline (pure parts tested on host).

---

## 13. Testing strategy details

- **Host-first.** All math (utils, PID, profiles, calibration, kinematics) and
  all `Joint`/`RobotArm` logic are tested on the desktop with fakes. Hardware
  files are thin and verified via the interface seam + compile checks.
- **Round-trip tests** are the backbone of kinematics: generate inputs, run
  `FK→IK→FK`, assert the *position* matches within tolerance. This catches sign
  errors, unit slips, and length swaps that single hand-picked cases miss.
- **Use asymmetric link lengths** (e.g. `120/80`) in at least some tests;
  symmetric `100/100` cases hide bugs.
- **Boundary & failure tests:** max reach, dead zone, exactly-on-boundary,
  `acos` domain clamping (no `NaN`), out-of-range angles, too-many-joints.
- **Deterministic timing:** never call `micros()` in a test. Inject `FakeClock`
  and advance it explicitly so PID/slew/profile behavior is reproducible.
- **Tolerances:** define a single `constexpr float kEps` (e.g. `1e-3f` for mm,
  `1e-2f` for degrees) and use doctest's `doctest::Approx` with it.
- **One concept per test**; descriptive names; `SUBCASE` for variations.
- **Fast:** the whole suite must run in well under a second so you can run it on
  every edit.

---

## 14. Safety requirements (build into the core, test them)

- **Never command a servo outside its configured range.** `ServoCalibration`
  clamps both the angle (`minDeg/maxDeg`) and the pulse (`minUs/maxUs`). Tested.
- **Unreachable targets are explicit.** `moveTo` returns `false` and clamps to
  the nearest reachable point; it must not produce out-of-range joint commands.
- **Rate limiting available and easy** (`setMaxSpeed`) so a beginner's arm moves
  smoothly instead of slamming to position.
- **No `NaN`/`Inf` ever leaves kinematics** — clamp `acos` inputs; guard
  divisions; test boundary inputs.
- **Sensible defaults** for pulse bounds and limits so a minimal sketch is safe
  out of the box.
- **Power the servos from a separate supply.** RDS3225s are high-torque digital
  servos that can each draw **≈ 2.9 A at stall**, so a multi-joint arm needs a
  dedicated **5–6 V** supply sized for the combined stall current (a shared rail
  in the multi-amp range) — never the Arduino's 5 V pin. The README must tell
  beginners to: feed the PCA9685 `V+` screw terminal from that supply (~6 V for
  full torque), add a **bulk capacitor (≈ 470–1000 µF)** across the `V+`/GND
  terminals to absorb current spikes, use thick power wiring, and tie all grounds
  together at one star point (Arduino GND ↔ PCA9685 GND ↔ supply GND). SDA/SCL go
  to the board's I²C pins. Keep commands **inside** each servo's calibrated pulse
  endpoints so it never grinds against a mechanical stop.

---

## 15. Documentation deliverables

- **`README.md`** (student-facing): intro; **how to install the library** so
  `#include <RobotArm.h>` works, and how to install the **Adafruit PWM Servo
  Driver Library** dependency (note Adafruit BusIO is pulled in automatically);
  **PCA9685 wiring** (I²C pins, `V+` external supply, common ground, channel
  numbering, address-jumper selection); the **RDS3225 (180°)** servo defaults
  (500–2500 µs, 1500 µs neutral, 50 Hz, ~6 V power) and how to confirm your
  servo's variant; the coordinate-frame sketch (reuse
  §5/§10 ASCII); calibration walkthrough; quickstart matching the real API; API
  cheat-sheet; troubleshooting ("arm reaches wrong direction → flip `direction`",
  "servo jitter / brown-out → use a separate servo supply", "nothing moves →
  check I²C address and wiring", "won't reach point → check link lengths /
  reachability").
- **Doc comments** on every public class and method, beginner-readable — and for
  each *concept* component (`ArmKinematics`, `PIDController`, `SlewRateLimiter`,
  `ServoCalibration`), a short "what this idea is and why" note so the header
  itself teaches (e.g. "Inverse kinematics: given where you want the hand, compute
  the joint angles that put it there").
- **`keywords.txt`** complete and current.
- **Inline math notes** in `Kinematics.h` referencing Appendix A for derivations.

---

## 16. Definition of done (whole project)

- Phases 0–11 complete; Phase 12 only if requested.
- `make test` green; `make check-core-purity` green; build has no warnings under
  `-Wall -Wextra -Wpedantic`.
- All examples compile for `arduino:avr:uno` (via `arduino-cli` if available).
- Public API matches the §11 spirit: composable components that teach each
  concept, with `RobotArm` as a transparent convenience built from them; safe;
  degrees + mm.
- `src/core/` is provably Arduino-free; all `Arduino.h`/`Wire.h`/`Adafruit_*`
  symbols live only in `src/arduino/`.
- A beginner can follow the README quickstart and move a real arm.
- Every public declaration is doc-commented; `keywords.txt` is complete.

---

## 17. Things to avoid

- Adding tools, build systems, or dependencies beyond §4.1 without stopping to
  justify.
- Putting `Arduino.h`/`Wire.h`/`Adafruit_*`/`Servo.h`/`Serial`/`String` anywhere
  under `src/core/`.
- Dynamic allocation, exceptions, or RTTI in shipped code.
- Writing implementation before a failing test (no "I'll test it later").
- Weakening or deleting a test to make it pass — fix the code or escalate.
- Piling on configuration knobs a high-schooler won't use. (Exposing a real
  concept as its own component — IK, PID, calibration — is good; redundant flags
  and options are not.)
- Collapsing a teachable step into an opaque call that hides the concept. Keep the
  components usable on their own; `RobotArm` may add convenience but must not be
  the only way to do something.
- Returning silently-wrong results for unreachable targets.
- Radians, meters, or inches in the public API.
- Large lookup tables or anything that won't fit comfortably on an Uno.

---

## Appendix A — Kinematics reference & verified test vectors

This arm is a **3R planar manipulator** in the X–Y plane (x = horizontal, y = up;
all angles in radians internally). Links: `L1` upper arm, `L2` forearm, `L3` hand
(wrist→gripper tip). The 2-link math is the inner solver; the 3R layer wraps it.

2-link forward kinematics (wrist point from shoulder + elbow):
```
x = L1*cos(θ1) + L2*cos(θ1 + θ2)
y = L1*sin(θ1) + L2*sin(θ1 + θ2)
```

2-link inverse kinematics (reach a point `(x, y)`):
```
D        = sqrt(x*x + y*y)                       # shoulder→point distance
reachable: |L1 - L2| <= D <= L1 + L2
cosθ2    = (D*D - L1*L1 - L2*L2) / (2*L1*L2)      # CLAMP to [-1, 1] before acos
θ2       =  acos(cosθ2)         # elbow-down  (use -acos(...) for elbow-up)
θ1       =  atan2(y, x) - atan2(L2*sin(θ2), L1 + L2*cos(θ2))
```

3-link forward kinematics (tip + approach angle):
```
x = L1*cos(θ1) + L2*cos(θ1+θ2) + L3*cos(θ1+θ2+θ3)
y = L1*sin(θ1) + L2*sin(θ1+θ2) + L3*sin(θ1+θ2+θ3)
φ = θ1 + θ2 + θ3                 # absolute gripper angle from +X
```

3R inverse kinematics (tip target `(x, y)` + approach angle `φ`):
```
xw = x - L3*cos(φ)               # wrist point = tip minus the hand link
yw = y - L3*sin(φ)
(θ1, θ2) = 2-link IK on (xw, yw) # reachability is checked on the WRIST point
θ3 = φ - (θ1 + θ2)
```

**Verified test vectors** (all hand-checked; use tolerance `1e-3` mm / `1e-2`
deg):

2-link FK (the inner solver):

| Case | L1 | L2 | Input (deg) | FK2 output (x, y) mm |
|------|----|----|-------------|----------------------|
| Sym A | 100 | 100 | θ1=0,  θ2=90 | (100.000, 100.000) |
| Sym B | 100 | 100 | θ1=0,  θ2=0  | (200.000, 0.000)   |
| Asym  | 120 | 80  | θ1=30, θ2=60 | (103.923, 140.000) |

2-link IK (the inner solver):

| IK case | L1 | L2 | Target (x, y) mm | Result (deg) |
|---------|----|----|------------------|--------------|
| Sym A inv | 100 | 100 | (100.000, 100.000) | θ1=0,  θ2=90 |
| Asym inv  | 120 | 80  | (103.923, 140.000) | θ1=30, θ2=60 |

Full 3R FK/IK round trips (tip + approach angle φ):

| 3R case | L1 | L2 | L3 | Angles (sh, el, wr) deg | Tip (x, y) mm | φ deg |
|---------|----|----|----|-------------------------|---------------|-------|
| Sym  | 100 | 100 | 60 | (30, 60, −30) | (116.603, 201.962) | 60 |
| Asym | 120 | 80  | 50 | (20, 40, 30)  | (152.763, 160.324) | 90 |

`IK3(tip, φ)` on each Tip/φ above must return the listed (shoulder, elbow,
wrist).

Reachability examples (`L1=L2=100`): target `(250, 0)` ⇒ `D=250 > 200` ⇒
unreachable; clamped result lies at `(200, 0)`. With `L1=120, L2=80`, target
`(10, 0)` ⇒ `D=10 < |L1−L2|=40` ⇒ unreachable (dead zone).

Boundary: `D = L1+L2` ⇒ `θ2 = 0` (straight); `D = |L1−L2|` ⇒ `θ2 = 180°`
(folded). Always clamp the `acos` argument to `[−1, 1]` so these inputs never
produce `NaN`.

---

## Appendix B — Example sketch intent (build these in Phase 11)

Each example uses the **components directly** so the student sees the concept; the
`RobotArm` convenience appears only as a "short version" at the end of 04.

- **01_SweepSingleServo** — create a `ServoDriver` and one `Joint`; set its pulse
  range / travel / limits; `setAngle()` + `update()` to sweep it. Teaches what a
  joint *is* (a servo channel + calibration + soft limits) and how degrees become
  a pulse.
- **02_CalibrateJoint** — sweep one RDS3225 across 500–2500 µs to find its true
  mechanical endpoints (often slightly inside, e.g. ~550–2450), set those two
  calibration points plus `direction`, and confirm 1500 µs ≈ 90°. Teaches
  calibration / the pulse-to-angle mapping.
- **03_WristAngle** — set the shoulder/elbow/wrist `Joint`s by hand and print
  `ArmKinematics::forward(...)` (the hand's x, y and approach angle φ); sweep φ
  while holding the wrist point fixed. Teaches forward kinematics and the
  approach-angle idea, and confirms the geometry matches the real arm.
- **04_MoveToXY** — the core lesson, the component way: call `ArmKinematics::
  solve(x, y, approachDeg)`, read the joint angles back, and command them to the
  `Joint`s to move the hand to a point (gripper down to "pick", level to "place",
  using `Gripper`). Then the *same* motion in three lines with the `RobotArm`
  convenience, to show it is exactly these parts wired together. Shows IK as a
  visible step and reachability handling.
- **05_SmoothMotionPID** — drive one `Joint` to setpoints, then add a
  `SlewRateLimiter` and a `PIDController` the student calls themselves
  (`pid.calculate(setpoint, measurement, dt)`) and applies, watching the
  difference. Teaches the control loop and what each piece does (with a feedback
  pot for true closed-loop if the hardware allows).

Each sketch: short, `using namespace roboarm;`, heavily commented, uses the public
components from §11, and matches the API exactly as shipped.
