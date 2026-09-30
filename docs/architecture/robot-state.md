---
layout: default
title: RobotState
eyebrow: Architecture
description: The top-level state machine that builds the robot from its definition and holds the shared pose history.
permalink: /architecture/robot-state/
---

[`RobotState`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/RobotState.java)
is the root of the state-machine tree. It builds the drive, vision and
superstructure from a [`RobotDefinition`]({{ '/architecture/robots/' | relative_url }}),
holds the pose and speed history everyone reads, and hands controller,
game and dashboard work to three small helpers.

> **Pattern.** `RobotState` doesn't know which mechanisms a robot has.
> It asks the definition for a `Superstructure` and adds whatever that
> returns.

## States

```java
public enum State { UNDETERMINED, SOFT_STOP, TRAVERSING, AUTO }
```

| State | Effect |
| --- | --- |
| `SOFT_STOP` | Drive to `IDLE`. |
| `TRAVERSING` | Drive to `TRAVERSING`. Entered on teleop start and when the machine determines itself. |
| `AUTO` | Drive to `TRAVERSING`, plus the selected auto. On autonomous start the auto's `build()` command is registered as this state's command. |

Every loop, `update()` also moves the drive back to `TRAVERSING` when
the driver pushes the right stick past 0.1, so manual rotation always
beats auto-aim.

## What it owns

| Field | Built from |
| --- | --- |
| `Drive` | `definition.drive()` |
| `Vision` | `definition.cameras()` |
| `Superstructure` | `definition.createSuperstructure(this)` |
| `Controls` | The two Xbox controllers. |
| `GameState` | Match phase and hub status. |
| `DashboardManager` | Auto chooser and `Game/*` values. |
| `SimulatedRobotState` | Ground-truth pose, simulation only. |

`vision`, `drive` and `superstructure.subsystems()` are added as child
subsystems.

## Public accessors

| Method | Returns |
| --- | --- |
| `getDefinition()` | The `RobotDefinition` this robot was built from. |
| `getDrive()` / `getVision()` | `Drive` / `Vision` |
| `getSuperstructure()` | The robot's `Superstructure`. |
| `getControls()` | `Controls` |
| `getGameState()` | `GameState` |
| `getSimRobot()` | `SimulatedRobotState` (`null` on a real robot). |
| `getLatestFieldToRobot()` | Latest `(timestamp, Pose2d)` entry. |
| `getFieldToRobot(timestamp)` | Interpolated pose at a past time, as an `Optional`. |
| `getLatest…ChassisSpeed…()` | Measured (robot- and field-relative), desired, and fused chassis speeds. |
| `getMaxAbsDriveYawAngularVelocityInRange(t0, t1)` | Fastest yaw rate in a window; vision uses it to reject frames taken while spinning. |
| `getCurrentHubSetpoint()` / `getCurrentPassSetpoint()` | A fresh [`ShooterSetpoint`]({{ '/utilities/shooter-setpoint/' | relative_url }}) on every call. |
| `shouldShootHub()` | `true` when the robot is on its own side of the hub. |

Mechanisms are not on `RobotState`. Competition code receives the
`CompetitionSuperstructure` directly and calls `getShooter()`,
`getIntake()`, and so on.

## Kinematic buffers

With `LOOKBACK_TIME = 1.0` s of history each:

- **`fieldToRobot`** — `Pose2d` history, fed by `addOdometryMeasurement`.
- **Yaw, pitch and roll rate, accel X and Y** — `Double` buffers, fed by `addDriveMotionMeasurements`.

The latest chassis speeds are kept in `AtomicReference`s, not buffers.
`addVisionMeasurement` forwards accepted vision poses to the drive's
pose estimator on the real robot only.

## Controls

[`Controls`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/Controls.java)
owns the driver (port 0) and operator (port 1) `CommandXboxController`s
and the bindings every robot shares:

| Driver input | Action |
| --- | --- |
| Left stick / right stick X | Translate / rotate (the drive's default command). |
| Right bumper (hold) | Drive `SLOW`. |
| D-pad left | Drive `TRAVERSING_AT_ANGLE` (auto-aim heading). |
| D-pad right | Drive `TRAVERSING`. |
| D-pad down | Reset heading to zero. Only bound when not in a match. |

`controls.rumble(seconds)` rumbles both controllers. `RobotState` uses
it for half a second whenever the hub turns on or off. Mechanism
bindings are added by the superstructure's `bindControls(controls)`.

## Game state and dashboard

Both live in `frc.robot.game`.

- **`GameState`** reads the match time and game-specific message each
  loop and works out the phase (`Autonomous`, `Transition`,
  `Shift 1`–`Shift 4`, `End Game`), the seconds until the next shift,
  whether our hub is active, and whether we won auto.
- **`DashboardManager`** builds the **Auto Choices** chooser (`None`,
  every PathPlanner auto, then the superstructure's autos), previews the
  selected auto's paths on the field while the Driver Station is in
  autonomous mode, and publishes `Game/HubActivated`, `Game/WonAuto`,
  `Game/GameState`, `Game/ShiftCountdown`, `Robot/AutoChoosed` and
  `Robot/Type`.

## The hand-off, end to end

On the competition robot, a right-trigger press becomes a shot in
roughly this sequence:

1. **Trigger** runs `drive.stopWithX()`, then
   `ActionCommands.shootOrPassBasedOnPos(robot)`.
2. That asks `RobotState.shouldShootHub()` and requests
   `Shooter.State.SHOOTING` (or `PASSING`).
3. `Shooter` requests its children's states: flywheel `SHOOT`, hood
   `HUB_TRACKING`, hopper and kicker `SHOOT`.
4. The hood and flywheel read their targets from
   `getCurrentHubSetpoint()` every loop.
5. The hopper and kicker only feed while the flywheel reports ready.
6. Releasing the trigger runs `ActionCommands.trackBasedOnPos(robot)`,
   which drops the shooter back to `HUB_TRACKING` or `PASS_TRACKING`.
