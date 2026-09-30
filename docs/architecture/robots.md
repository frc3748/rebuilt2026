---
layout: default
title: Multiple Robots
eyebrow: Architecture
description: One codebase runs several robots. How the code picks a robot at boot, and how to add a new one.
permalink: /architecture/robots/
---

The same code runs the competition robot and the practice robot.
Everything robot-specific lives in `frc.robot.robots`. Everything else
(drive, vision, the state-machine framework, the game code in
`frc.robot.game`) is shared.

## Package layout

```
robots/
├── RobotType.java                  ← Enum of robots, picked by roboRIO serial number
├── RobotDefinition.java            ← What a robot is: name, drivetrain, cameras, superstructure
├── Superstructure.java             ← A robot's mechanisms, autos, and button bindings
├── competition/
│   ├── CompetitionRobot.java           ← RobotDefinition; defines the two Limelights
│   ├── CompetitionDrive.java           ← DriveConfig: Pigeon 2, Spark Flex drive, CANcoder turn
│   ├── CompetitionSuperstructure.java  ← Builds the mechanisms and owns the driver/operator bindings
│   ├── ActionCommands.java             ← Composite commands for this robot
│   └── autos/                          ← CompetitionAuto base, one file per auto, CustomAuto
└── practice/
    ├── PracticeRobot.java              ← RobotDefinition; no cameras, no mechanisms
    └── PracticeDrive.java              ← DriveConfig: NavX, Spark MAX drive, Spark absolute-encoder turn
```

## Picking a robot at boot

`Robot`'s constructor asks `RobotType` which robot it is on and hands
that robot's definition to `RobotState`:

```java
RobotType robotType = RobotType.detect();
Logger.recordMetadata("ROBOT_TYPE", robotType.name());
...
robotState = new RobotState(robotType.create());
```

Each `RobotType` entry holds a factory and, optionally, the roboRIO
serial numbers it runs on:

```java
public enum RobotType {
    COMPETITION(CompetitionRobot::new),
    PRACTICE(PracticeRobot::new);
    ...
}
```

`RobotType.detect()`:

- In simulation, returns `Constants.kDefaultRobot` (currently `COMPETITION`).
- On a roboRIO, reads `RobotController.getSerialNumber()` and returns the entry that lists that serial.
- If no entry matches, raises a warning `Alert` (`Unknown roboRIO serial …`) and returns `Constants.kDefaultRobot`.

> **Serials aren't filled in yet.** Neither entry lists a serial, so
> every roboRIO currently runs `kDefaultRobot` and shows the alert. Add
> the practice roboRIO's serial to `PRACTICE` before deploying there.

To run a different robot in the simulator, change `Constants.kDefaultRobot`.

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
5. **Add a `RobotType` entry** with the roboRIO serial:

   ```java
   BETA(BetaRobot::new, "03264A7B")
   ```

   The serial is shown in the "Unknown roboRIO serial" alert the first
   time you deploy.
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
- **`PracticeRobotTest`** checks that the practice robot boots with only
  a drivetrain, uses its own `DriveConfig`, and that `RobotType.detect()`
  returns `kDefaultRobot` in simulation.

`build.gradle` sets `forkEvery = 1` on the `test` task, so each test
class boots its robot in a fresh JVM.
