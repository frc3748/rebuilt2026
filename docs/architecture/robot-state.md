---
layout: default
title: RobotState
eyebrow: Architecture
description: The top-level state machine that owns every subsystem and binds every controller.
permalink: /architecture/robot-state/
---

[`RobotState`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/RobotState.java)
is the largest file in the codebase and the only one that knows about
*every* subsystem. It serves three jobs at once: subsystem owner,
controller binder, and global state machine.

> **Pattern.** Think of `RobotState` as the "wiring diagram" for the
> robot. It doesn't implement subsystem behavior; it composes it.

## States

```java
public enum State {
  UNDETERMINED, SOFT_STOP, TRAVERSING, AUTO,
  CLIMBING, SHOOTING, INTAKING, PASSING
}
```

These are not redundant with subsystem states — they're the *match
flow*. The `RobotState` doesn't care which intake position you're in;
it cares whether you're "currently intaking" or "currently shooting"
so that, e.g., the climb subsystem won't accept a deploy request
mid-shot.

## What it owns

In its constructor, `RobotState` instantiates:

- One `Drive` (with a `GyroIO` + 4× `ModuleIO`)
- One `VisionSubsystem` (with 1+ `VisionIO`)
- One `Shooter` (with `TurretIO`, `HoodIO`, `FlywheelIO`)
- One `Intake` (with `IntakeIO`)
- One `Hopper` (with `HopperIO`)
- One `Kicker` (with `KickerIO`)
- One `Climb` (with `ClimbIO`)

The choice of IO implementation comes from a single `robotState`
integer field — see [the IO pattern]({{ '/architecture/io-pattern/' | relative_url }}).

## Public accessors

The rest of the code accesses subsystems through `RobotState`:

| Method | Returns |
| --- | --- |
| `getDrive()` | `Drive` |
| `getVision()` | `VisionSubsystem` |
| `getShooter()` | `Shooter` |
| `getIntake()` | `Intake` |
| `getHopper()` | `Hopper` |
| `getKicker()` | `Kicker` |
| `getClimb()` | `Climb` |
| `getController()` | Driver `CommandXboxController` |
| `getCurrentHubSetpoint()` | A `Supplier<ShooterSetpoint>` for hub shots |
| `getCurrentPassSetpoint()` | A `Supplier<ShooterSetpoint>` for passes |
| `getLatestFieldToRobot()` | Most recent pose from the buffer |
| `getPredictedFieldToRobot()` | Pose extrapolated to the *next* shot release |

The two pose accessors deserve a callout: shots aim at where the robot
*will be* a few ms in the future, not where it is now. The latency
compensation lives in the kinematic buffers.

## Kinematic buffers

`RobotState` holds three
[`ConcurrentTimeInterpolatableBuffer`]({{ '/utilities/time-buffers/' | relative_url }})
instances, each with ~1s of history:

- **`fieldToRobotBuffer`** — pose history. Lets vision measurements
  fuse at the timestamp they were taken, not received.
- **`turretAngleBuffer`** — turret yaw history. Lets the turret's
  vision pipeline correct for the angle the turret was at when the
  frame was captured.
- **`driveSpeedsBuffer`** — chassis velocity history. Used by the
  shooter solver to compensate for robot motion at release time.

## Controller bindings

Driver and operator controllers get bound to subsystem transitions:

```java
controller.rightTrigger().onTrue(ActionCommands.aimAndShoot(this));
controller.leftBumper() .onTrue(intake.transitionCommand(Intake.State.INTAKE));
controller.leftBumper() .onFalse(intake.transitionCommand(Intake.State.STOW));
controller.a()          .onTrue(climb.transitionCommand(Climb.State.UP));
```

Every binding goes through a state machine — never a raw motor write.
This is what makes the codebase replayable and testable.

## Auto selection

A `LoggedDashboardChooser<Command>` is populated from
[`AutoCommands`]({{ '/commands/auto-commands/' | relative_url }}). The
selected `Command` is returned by `getAutoCommand()` and runs in
`Robot.autonomousInit()`.

## Subsystem registration

Every subsystem registers itself with
[`SubsystemManager`]({{ '/architecture/subsystem-manager/' | relative_url }})
in its own constructor. By the time `RobotState`'s constructor
returns, all of them are wired into the lifecycle broadcast.

## The hand-off, end to end

A right-trigger press becomes a shot in roughly this sequence:

1. **Trigger** fires `ActionCommands.aimAndShoot(robotState)`.
2. The composite first calls `shooter.transitionCommand(HUB_TRACKING)`.
3. `Shooter` enters `HUB_TRACKING`; its state command sets the child
   subsystems into `HUB_TRACKING` too, each pulling its target from
   the supplier in `RobotState.getCurrentHubSetpoint()`.
4. The supplier asks `getPredictedFieldToRobot()` for where the robot
   *will be* on shot release.
5. The shooter waits until all three children report `isReady()`.
6. It transitions to `SHOOTING`; the state command spins hopper and
   kicker.
7. The piece launches. After a short timeout, `Shooter` returns to
   `IDLE`.

That whole flow exists because `RobotState` knows about every piece
and can wire them together. The subsystems themselves remain unaware
of each other.
