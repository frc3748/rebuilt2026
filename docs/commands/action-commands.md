---
layout: default
title: Action Commands
eyebrow: Commands
description: High-level composite commands — what driver buttons actually invoke.
permalink: /commands/action-commands/
---

[`ActionCommands`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/commands/ActionCommands.java)
is the "buttons-to-behavior" layer. Every static method here returns
a `Command` that orchestrates multiple subsystems.

Think of it as the playbook: each method is one named play that the
driver or auto can call.

## Factory pattern

Every method is `static` and takes the `RobotState` as its first
argument:

```java
public static Command aimAndShoot(RobotState state) { … }
public static Command shootOrPassBasedOnPos(RobotState state) { … }
public static Command trackBasedOnPos(RobotState state) { … }
public static Command autoClimb(RobotState state) { … }
public static Command goToFixedPosAndShoot(RobotState state, Pose2d target) { … }
public static Command shakeIntake(RobotState state) { … }
```

This keeps the factory stateless — every command captures the state
reference at build time.

## The "shoot" family

### `aimAndShoot(state)`

The basic full shot:

1. Request `Shooter.State.HUB_TRACKING`.
2. Wait until the shooter reports ready.
3. Request `Shooter.State.SHOOTING`.
4. After the shot completes (timeout or beam clear), return to `IDLE`.

### `shootOrPassBasedOnPos(state)`

Same as `aimAndShoot`, but picks the target based on the alliance and
the robot's field X coordinate:

- **Blue alliance, X ≤ hub-X** — shoot the hub.
- **Red alliance, X ≥ hub-X** — shoot the hub.
- **Otherwise** — pass to teammate.

This lets the same button do the right thing from either side of the
field. The target factory (`BallTargetFactory` vs.
`PassTargetFactory`) is chosen accordingly.

### `trackBasedOnPos(state)`

Same target-selection logic as above, but doesn't shoot — just aims.
Useful as a "hold trigger to aim, release to shoot" pattern combined
with another binding.

### `goToFixedPosAndShoot(state, target)`

Drives to a hardcoded pose via
[`AutoAlignToPoseCommand`]({{ '/commands/auto-align/' | relative_url }}),
then runs `aimAndShoot`. Used in auto routines that have known
scoring positions.

## The "climb" family

### `autoClimb(state)`

Coordinates drive + climb for a rung climb:

1. Drive auto-aligns to the climb pose.
2. Climb deploys: `STOW → UP`.
3. Waits for an operator confirmation (a specific button press) to
   start `UP → DOWN`.
4. The `Climb` subsystem auto-completes to `CLIMB` once the motor
   stall-current threshold is exceeded.

## The "utility" family

### `shakeIntake(state)`

Repeatedly toggles the intake between `INTAKE` and `SHAKE` to unjam a
stuck piece. Cancellable — release the button and the intake goes
back to `STOW`.

## Where these get bound

In `RobotState`'s controller-binding section:

```java
controller.rightTrigger().onTrue(ActionCommands.shootOrPassBasedOnPos(this));
controller.rightBumper() .onTrue(ActionCommands.autoClimb(this));
controller.x()           .whileTrue(ActionCommands.shakeIntake(this));
```

The pattern is always **bind to an `ActionCommands` method**, not to a
raw transition, when the behavior spans more than one subsystem.

## Adding a new action

The shape:

```java
public static Command myNewAction(RobotState state) {
  return Commands.sequence(
    state.getShooter().transitionCommand(Shooter.State.HUB_TRACKING),
    Commands.waitUntil(state.getShooter().getFlywheel()::isReady),
    state.getShooter().transitionCommand(Shooter.State.SHOOTING),
    Commands.waitSeconds(0.5),
    state.getShooter().transitionCommand(Shooter.State.IDLE)
  );
}
```

Then bind it in `RobotState`'s setup. Never bypass the state machines
— always go through `transitionCommand`.
