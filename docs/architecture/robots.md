---
layout: default
title: Multiple Robots
eyebrow: Architecture
description: One codebase runs several robots. How the code picks a robot at boot, and how to add a new one.
permalink: /architecture/robots/
---

The same code runs the competition robot, the new competition chassis
(V2), and the practice robot.
Everything robot-specific lives in `frc.robot.robots`. Everything else
(drive, vision, the state-machine framework, the game code in
`frc.robot.game`) is shared.

## Package layout

```
robots/
├── RobotType.java                  ← Enum of robots; Constants.kRobot picks one
├── RobotDefinition.java            ← What a robot is: name, drivetrain, cameras, superstructure
├── Superstructure.java             ← A robot's mechanisms, autos, and button bindings
├── competition/
│   ├── CompetitionRobot.java           ← RobotDefinition; defines the two Limelights
│   ├── CompetitionDrive.java           ← DriveConfig: Pigeon 2, Spark Flex drive, CANcoder turn
│   ├── CompetitionSuperstructure.java  ← Builds the mechanisms and owns the driver/operator bindings
│   ├── ActionCommands.java             ← Composite commands for this robot
│   └── autos/                          ← CompetitionAuto base, one file per auto, CustomAuto
├── competitionv2/
│   ├── CompetitionV2Robot.java         ← Extends CompetitionRobot; only the drivetrain changes
│   └── CompetitionV2Drive.java         ← Extends CompetitionDrive; WCP Swerve X2t modules
└── practice/
    ├── PracticeRobot.java              ← RobotDefinition; no cameras, no mechanisms
    └── PracticeDrive.java              ← DriveConfig: NavX, Spark MAX drive, Spark absolute-encoder turn
```

## Picking a robot

One constant picks the robot, for deploying and for the simulator:

```java
public static final RobotType kRobot = RobotType.COMPETITION;
```

Change it to `COMPETITION_V2` or `PRACTICE` before deploying to that
robot. `Robot`'s constructor hands that robot's definition to `RobotState`:

```java
robotState = new RobotState(Constants.kRobot.create());
```

## `RobotDefinition`

```java
public interface RobotDefinition {
    String name();
    DriveConfig drive();
    CameraConfig[] cameras();

    default Superstructure createSuperstructure(RobotState state) {
        return new Superstructure() {};
    }
}
```

[`RobotState`]({{ '/architecture/robot-state/' | relative_url }}) builds
the shared parts from it, in this order: `new Drive(definition.drive(), this)`,
`new Vision(this, definition.cameras())`, then
`definition.createSuperstructure(this)`. The superstructure comes last,
so it can already call `state.getDrive()`.

## `Superstructure`

A robot's mechanisms. Every method has a default, so a drivetrain-only
robot doesn't need one at all.

| Method | Default | What `RobotState` does with it |
| --- | --- | --- |
| `subsystems()` | empty | Adds each state machine as a child, so it gets lifecycle events. |
| `autos()` | empty | Adds each `AutoRoutine` to the **Auto Choices** chooser. |
| `bindControls(Controls)` | nothing | Called after the shared drive bindings. |
| `simulationPeriodic()` | nothing | Called every sim loop. |

Drive bindings are the same on every robot and live in
[`Controls`]({{ '/architecture/robot-state/' | relative_url }}#controls).

## The competition robot

- **`CompetitionRobot`** — name `Competition`, the drivetrain from
  `CompetitionDrive`, and two cameras: `kChassisCamera` and
  `kShooterCamera` (both `LIMELIGHT_4`). See [Vision]({{ '/subsystems/vision/' | relative_url }}).
- **`CompetitionDrive`** — Pigeon 2 on CAN 50, Spark Flex drive, Spark MAX
  turn with CANcoders, 28″ × 28″, 5.27 m/s max. See [Drive]({{ '/subsystems/drive/' | relative_url }}).
- **`CompetitionSuperstructure`** — builds `Flywheel`, `Hood`, `Hopper`,
  `Kicker`, `Intake`, a `FuelSimulation` (sim only), and the `Shooter`
  that orchestrates them. `subsystems()` returns the shooter, hopper,
  kicker and intake (hood and flywheel are the shooter's children).
  `autos()` returns the 15 autos. `bindControls` holds every driver and
  operator mechanism binding.
- **`ActionCommands`** — composite commands that take the
  `CompetitionSuperstructure`. See [Action Commands]({{ '/commands/action-commands/' | relative_url }}).
- **`autos/`** — see [Autos]({{ '/commands/autos/' | relative_url }}).

## The competition V2 robot

The new chassis keeps the competition robot's electronics and frame
size (28″ × 28″) but moves from WCP Swerve X2 to WCP Swerve X2t
(corner mount) modules with the X1/X2 ratio set (8 mm key bore, for
NEO/Vortex). It is not the main robot yet.

- **`CompetitionV2Robot`** extends `CompetitionRobot` and overrides only
  `name()` (`Competition V2`) and `drive()`. It gets the same cameras,
  mechanisms, bindings and autos.
- **`CompetitionV2Drive`** extends `CompetitionDrive` and overrides only
  the module-specific values: the four `ModuleConstants` (same CAN IDs,
  zero rotations reset to `0` for calibration), wheel radius, drive and
  turn reductions, turn inversions, and max speed. The X2t shares the
  X2's 12.1 : 1 steering, 4″ wheel and ratio sets, so these start at the
  current robot's values (6.48 : 1 drive). If the new modules use a
  different pinion (10/11/12t) or X1/X2 gear, look up the drive ratio in
  WCP's [Swerve X2 ratio table](https://docs.wcproducts.com/welcome/gearboxes/wcp-swerve-x2/general-info/ratio-options)
  and update `driveReduction` and `maxSpeedMetersPerSec`. Check the
  drive and turn directions on first boot.

When the new chassis becomes the main robot:

1. Calibrate the module zero rotations and set them in `CompetitionV2Drive`.
2. Set `Constants.kRobot = RobotType.COMPETITION_V2`.
3. If the Limelights move, override `cameras()` in `CompetitionV2Robot`.

## The practice robot

`PracticeRobot` is a drivetrain: name `Practice`, no cameras, and the
default empty superstructure. `PracticeDrive` holds its constants,
taken from the old `2026-PracticeRobot` repo:

- NavX on `NavXComType.kUSB1` at 50 Hz (`GyroIONavX`).
- Spark MAX drive motors, NEO turn motors reading the Spark's absolute
  encoder (no CANcoders, so `canCoderId` is `-1`).
- 7.31 : 1 drive, 12.8 : 1 turn, 3.5 m/s max.
- Module zero rotations are `0.0`, as in the old repo.

CAN IDs are on the [CAN ID Map]({{ '/reference/can-ids/' | relative_url }}#practice-robot).

## Adding a robot

1. **Make a package**, `robots/<name>/`.
2. **Write a `DriveConfig` subclass.** Set fields in its constructor:
   gyro type and ID or port, drive controller, turn sensor, the four
   `ModuleConstants`, geometry, gains, speed limits, and the PathPlanner
   mass, MOI and PID. Anything you leave out keeps the default in
   `DriveConfig`. `PracticeDrive` is a short example.
3. **Write a `RobotDefinition`** that returns a name, a new instance of
   your `DriveConfig`, and its cameras (`new CameraConfig[0]` if none).
4. **Optionally write a `Superstructure`** for the robot's mechanisms,
   autos and bindings, and return it from `createSuperstructure`.
5. **Add a `RobotType` entry**, `BETA(BetaRobot::new)`, and set
   `Constants.kRobot = RobotType.BETA` to run it.
6. **Add a test** that boots it, like `PracticeRobotTest`:

   ```java
   state = new RobotState(new BetaRobot());
   ```

The mechanism classes in `subsystems/` can be reused, but their
`*Constants` files (CAN IDs, gains) describe the competition robot.

## Tests

`src/test/java/frc/robot/` has one test class per robot:

- **`CompetitionRobotTest`** boots `new RobotState(new CompetitionRobot())`
  and checks that subsystems follow requested states, that an override
  wins until cleared, that camera inputs survive a log round trip, that
  `Camera` turns observations into weighted measurements and object
  positions, and that all 15 autos load their paths by their exact file names.
- **`CompetitionV2RobotTest`** checks that the V2 robot reuses the
  competition superstructure, autos and cameras, and drives on
  `CompetitionV2Drive`.
- **`PracticeRobotTest`** checks that the practice robot boots with only
  a drivetrain and uses its own `DriveConfig`.

`build.gradle` sets `forkEvery = 1` on the `test` task, so each test
class boots its robot in a fresh JVM.
