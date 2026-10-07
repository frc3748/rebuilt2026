---
layout: default
title: Multiple Robots
eyebrow: Architecture
description: One codebase runs several robots. How the code picks a robot at boot, how each robot overrides only what differs, and how to add a new one.
permalink: /architecture/robots/
---

The same code runs three robots: comp, the new chassis and future main
robot; secondary, the current robot; and practice, a drivetrain. Each
robot's definition lives in `frc.robot.robots`. Everything else (drive,
vision, the superstructure, autos, action commands, controller
bindings, the state-machine framework, the game code in
`frc.robot.game`) is shared. Each robot extends the closest robot and
overrides only what differs.

## Package layout

```
robots/
├── RobotType.java              ← COMP, SECONDARY, PRACTICE; Constants.kRobot picks one
├── RobotDefinition.java        ← Abstract base: name and drivetrain, plus overridable defaults
├── comp/
│   ├── CompRobot.java              ← The full robot: two Limelights, ShooterComp, IntakeComp, mechanism constants
│   └── CompDrive.java              ← DriveConfig: WCP Swerve X2t, Pigeon 2, Spark Flex drive, CANcoder turn
├── secondary/
│   ├── SecondaryRobot.java         ← Extends CompRobot; overrides name() and drive()
│   └── SecondaryDrive.java         ← Extends CompDrive; calibrated module offsets
└── practice/
    ├── PracticeRobot.java          ← Drivetrain only
    └── PracticeDrive.java          ← DriveConfig: NavX, Spark MAX drive, Spark absolute-encoder turn
```

Shared by every robot, under `frc/robot/`:

| Class | What it is |
| --- | --- |
| `Superstructure` | Holds whichever mechanisms the robot has. |
| `Controls` | Every driver and operator binding. See [Controls]({{ '/architecture/robot-state/' | relative_url }}#controls). |
| `commands/ActionCommands` | Composite commands. See [Action Commands]({{ '/commands/action-commands/' | relative_url }}). |
| `commands/autos/` | `AutoRoutine`, `PathAuto`, one file per auto, and `Autos.all(state)`. See [Autos]({{ '/commands/autos/' | relative_url }}). |
| `subsystems/intake/Intake`, `subsystems/shooter/Shooter` | Abstract bases. `IntakeComp` and `ShooterComp` implement them. |

## Picking a robot

One constant in `Constants` picks the robot, for deploying and for the
simulator:

```java
public static final RobotType kRobot = RobotType.SECONDARY;
```

Change it to `COMP` or `PRACTICE` before deploying to that robot.
`Robot`'s constructor hands that robot's definition to `RobotState`:

```java
robotState = new RobotState(Constants.kRobot.create());
```

## `RobotDefinition`

`RobotDefinition` is an abstract class. A robot must give a name and a
drivetrain; everything else has a default it can override:

| Method | Default |
| --- | --- |
| `name()` | Abstract. Shown on the dashboard as `Robot/Type`. |
| `drive()` | Abstract. A new instance of the robot's `DriveConfig`. |
| `shooter()` | `new ShooterConstants()`. See [Per-robot constants](#per-robot-constants). |
| `cameras()` | No cameras. |
| `createSuperstructure(state)` | `new Superstructure(state)`, with no mechanisms. |
| `createControls()` | `new Controls()`. |
| `autos(state)` | `Autos.all(state)`. |

[`RobotState`]({{ '/architecture/robot-state/' | relative_url }}) builds
the robot from it in this order: `createControls()`, `shooter()` (plus a
`ShotCalculator` that uses it), `new Drive(definition.drive(), this)`,
`new Vision(this, definition.cameras())`, `createSuperstructure(this)`,
then the dashboard with `name()` and `autos(this)`. The superstructure
comes after the drive, so it can already call `state.getDrive()`.

Values are inherited. A change in `CompDrive` or `CompRobot` also
changes secondary, unless `SecondaryDrive` or `SecondaryRobot`
overrides that value.

## `Superstructure`

[`Superstructure`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/Superstructure.java)
is one class shared by every robot. A robot's `createSuperstructure`
builds it with the mechanisms it has. `CompRobot`'s:

```java
@Override
public Superstructure createSuperstructure(RobotState state) {
    return new Superstructure(state)
            .withShooter(new ShooterComp(state, flywheel(), hood(), hopper(), kicker()))
            .withIntake(new IntakeComp(state, intake()));
}
```

| Method | What it does |
| --- | --- |
| `withShooter(Shooter)` / `withIntake(Intake)` | Adds the mechanism. In `SIM` mode the first one also starts the [fuel sim]({{ '/utilities/simulation/' | relative_url }}). |
| `getShooter()` / `getIntake()` | An `Optional`, empty when the robot doesn't have that mechanism. |
| `shooterCommand(fn)` / `intakeCommand(fn)` | The command `fn` builds from the mechanism, or `Commands.none()` without it. |
| `shooterAction(fn)` / `intakeAction(fn)` | A `Runnable` that calls `fn` on the mechanism, or does nothing without it. |
| `clearOverrides()` | Releases the shooter's feed and shot overrides and clears the intake's. |
| `subsystems()` | The mechanisms added. `RobotState` adds each as a child, so it gets lifecycle events. |
| `simulationPeriodic()` | Runs the fuel sim, the sim shots and the `ShotVisualizer`. Called every sim loop. |

The autos and controller bindings reach mechanisms only through these
methods, so a step or button for a missing mechanism does nothing.

## The comp robot

The new chassis on WCP Swerve X2t (corner mount) modules with the X1/X2
ratio set (8 mm key bore, for NEO/Vortex). It will be the main robot.

- **`CompRobot`** holds the full definition: name `Comp`, `CompDrive`,
  two `LIMELIGHT_4` cameras from `chassisCamera()` and `shooterCamera()`
  (see [Vision]({{ '/subsystems/vision/' | relative_url }})), a
  superstructure with `ShooterComp` and `IntakeComp`, and the mechanism
  constants (see [Per-robot constants](#per-robot-constants)).
- **`CompDrive`** — Pigeon 2 on CAN 50, Spark Flex drive, Spark MAX turn
  with CANcoders, 28″ × 28″, 6.48 : 1 drive, 12.1 : 1 turn, 5.27 m/s max.
  The module zero rotations are `0` until calibrated. See
  [Drive]({{ '/subsystems/drive/' | relative_url }}).

The X2t shares the X2's 12.1 : 1 steering, 4″ wheel and ratio sets, so
`CompDrive` starts at the secondary robot's values. If the modules use a
different pinion (10/11/12t) or X1/X2 gear, look up the drive ratio in
WCP's [Swerve X2 ratio table](https://docs.wcproducts.com/welcome/gearboxes/wcp-swerve-x2/general-info/ratio-options)
and update `driveReduction` and `maxSpeedMetersPerSec`. `SecondaryDrive`
sets its own `driveReduction` but inherits `maxSpeedMetersPerSec`, so
set the old max speed in `SecondaryDrive` if you change comp's. Check
the drive and turn directions on first boot.

When comp becomes the main robot:

1. Calibrate the module zero rotations and set them in `CompDrive`.
2. Set `Constants.kRobot = RobotType.COMP`.
3. If comp's Limelights sit differently, change `chassisCamera()` or
   `shooterCamera()` in `CompRobot`, and override the old one in
   `SecondaryRobot` so secondary keeps it.

## The secondary robot

The current robot, on WCP Swerve X2 modules, and the one
`Constants.kRobot` is set to. It has comp's electronics, frame, cameras
and mechanisms.

- **`SecondaryRobot`** extends `CompRobot` and overrides only `name()`
  (`Secondary`) and `drive()`. Everything else comes from `CompRobot`.
- **`SecondaryDrive`** extends `CompDrive` and overrides the four
  `ModuleConstants`: the same CAN IDs with its calibrated zero
  rotations. It also sets `driveReduction = 6.48`, the same as comp
  today, so a new ratio in `CompDrive` doesn't change it.

## The practice robot

`PracticeRobot` extends `RobotDefinition` and overrides only `name()`
(`Practice`) and `drive()`, so it has no cameras and a superstructure
with no mechanisms. It still runs every shared auto. Shooter and intake
steps do nothing, but `shake(seconds)` still waits its time, so the
robot drives each auto's paths. `PracticeDrive` holds its constants,
taken from the old `2026-PracticeRobot` repo:

- NavX on `NavXComType.kUSB1` at 50 Hz (`GyroIONavX`).
- Spark MAX drive motors, NEO turn motors reading the Spark's absolute
  encoder (no CANcoders, so `canCoderId` is `-1`).
- 7.31 : 1 drive, 12.8 : 1 turn, 3.5 m/s max.
- Module zero rotations are `0.0`, as in the old repo.

CAN IDs are on the [CAN ID Map]({{ '/reference/can-ids/' | relative_url }}#practice-robot).

## Adding a robot

1. **Make a package**, `robots/<name>/`.
2. **Extend the closest robot.** A variant of comp extends `CompRobot`
   and `CompDrive`, like secondary. A new drivetrain extends
   `RobotDefinition` and `DriveConfig`, and sets fields in the
   `DriveConfig` constructor: gyro type and ID or port, drive
   controller, turn sensor, the four `ModuleConstants`, geometry, gains,
   speed limits, and the PathPlanner mass, MOI and PID. `PracticeDrive`
   is a short example.
3. **Override only what differs**: `name()`, `drive()`, and as needed
   `cameras()`, `createSuperstructure(state)` (`new Superstructure(state)`
   plus `withShooter(...)` and `withIntake(...)`), the constants methods,
   `createControls()` or `autos(state)`.
4. **Add a `RobotType` entry**, `BETA(BetaRobot::new)`, and set
   `Constants.kRobot = RobotType.BETA` to run it.
5. **Add a test** that boots it, like `SecondaryRobotTest`:

   ```java
   state = new RobotState(new BetaRobot());
   ```

To change one binding, subclass `Controls`, override the binding method
(they are all protected, as are the `driver` and `operator` fields),
call `super`, and return it from `createControls()`:

```java
public class BetaControls extends Controls {
    @Override
    protected void bindIntake(RobotState state, Intake intake) {
        super.bindIntake(state, intake);
        driver.y().onTrue(intake.transitionCommand(Intake.State.STOW));
    }
}
```

## Per-robot constants

`IntakeConstants`, `FlywheelConstants`, `HoodConstants`,
`HopperConstants` and `KickerConstants` are plain classes with public
fields: `MotorConfig`s, setpoints, speeds and geometry. Their defaults
are the comp values. Each subsystem takes its constants object in its
constructor and builds its `TunableNumber`s from it, so the dashboard
keys are the same on every robot. A robot that tunes a shared value
differently gets a `Tuning.override(...)` line in its own class (see
[Tuning]({{ '/utilities/tunable-number/' | relative_url }})). Derived values (the flywheel's
conversion from `radius`, the hood's conversion, soft limits and
starting position) are applied when the subsystem is built, so changing
one field never leaves a stale derived value.

`CompRobot` hands them to `ShooterComp` and `IntakeComp` through
protected methods: `intake()`, `flywheel()`, `hood()`, `hopper()` and
`kicker()`. Each returns a new object. A robot overrides one, calls
`super`, and changes a field. For example, if secondary's intake stowed
at a different angle (it doesn't; this only shows the pattern):

```java
@Override
protected IntakeConstants intake() {
    IntakeConstants intake = super.intake();
    intake.stowSetpoint = -90;
    return intake;
}
```

`RobotDefinition.shooter()` works the same way for `ShooterConstants`:
`shooterToRobotCenter`, `distanceAboveFunnel`,
`timeOfFlightOffsetSeconds`, `simSecondsBetweenShots`, and the shot and
time-of-flight maps. Its constructor fills the maps with `addShot`; a
robot with its own table calls `clearShots()` and adds its shots.
`RobotState` creates it once, along with a `ShotCalculator` that uses
it, and exposes them as `getShooterConstants()` and
`getShotCalculator()`. Aiming in `Drive`, the dashboard's hub distance,
`ShotVisualizer`, the fuel sim and `Hood`'s pose read it from there.
`CompRobot.shooterCamera()` calls `shooter()` itself, so the camera
follows an override too.

## Adding a subsystem variant

`Intake` and `Shooter` are abstract. Each holds its `State` enum and
the calls shared code makes:

- `Intake`: `rollIn()`, `rollOut()`.
- `Shooter`: `spinUp()`, `holdShot(setpoint, spinFlywheel)`,
  `holdShot(speed, hoodPosition, hoodFeedforward)`, `releaseShot()`,
  `stopFeed()`, `reverseFeed()`, `forceFeed()` and `releaseFeed()`, plus
  `isFiring()` and `isPassing()`, which it works out from the state.

`IntakeComp` and `ShooterComp` implement them. Their internals (motors,
children, tunables, `goTo`, `registerStateCommands`, `request`) are
protected, so a variant that changes one behavior can extend them and
override just that. A robot whose shooter works differently gets its
own class in the subsystem's package, named subsystem first, say
`ShooterPractice extends Shooter`, which implements the abstract calls
and says what each state does. Its definition passes it to
`withShooter(new ShooterPractice(state))`, and the shared autos and
bindings work with it unchanged.

## Tests

`src/test/java/frc/robot/` has one test class per robot:

- **`CompRobotTest`** boots `new RobotState(new CompRobot())`, gets the
  shooter and intake from its `Superstructure`, and checks that they
  follow requested states, that an override wins until cleared, that
  camera inputs survive a log round trip, that `Camera` turns
  observations into weighted measurements and object positions, and
  that all 14 autos load their paths by their exact file names.
- **`SecondaryRobotTest`** checks that secondary reuses `ShooterComp`,
  `IntakeComp` and the two cameras, and that its `SecondaryDrive` keeps
  comp's drive reduction and max speed but not its module zero
  rotations.
- **`PracticeRobotTest`** checks that the practice robot boots with only
  a drivetrain and its own `DriveConfig`, that mechanism commands do
  nothing without the mechanism, and that it loads every shared auto and
  drives one.

`build.gradle` sets `forkEvery = 1` on the `test` task, so each test
class boots its robot in a fresh JVM.
