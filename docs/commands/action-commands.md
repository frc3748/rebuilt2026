---
layout: default
title: Action Commands
eyebrow: Commands
description: High-level composite commands shared by every robot — what driver buttons and autos actually invoke.
permalink: /commands/action-commands/
---

[`ActionCommands`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/commands/ActionCommands.java)
is the "buttons-to-behavior" layer. Every robot uses the same one.

Think of it as the playbook: each method is one named play that the
driver or an auto can call.

## Factory pattern

Every method is `static`, takes the `RobotState`, and returns a
`Command`:

```java
public static Command shakeIntake(RobotState state)
public static Command aimAtHub(RobotState state)
public static Command turnToHub(RobotState state)
public static Command aimAndShoot(RobotState state)
public static Command shootOrPassBasedOnPos(RobotState state)
public static Command trackBasedOnPos(RobotState state)
public static Command goToFixedPosAndShoot(RobotState state)
```

Mechanisms are reached through `state.getSuperstructure()`'s
`intakeCommand`, `shooterCommand` and `shooterAction`, so on a robot
without that mechanism the command does nothing: `shakeIntake`,
`aimAndShoot`, `shootOrPassBasedOnPos` and `trackBasedOnPos` become
`Commands.none()`, and `goToFixedPosAndShoot` only drives.

## Aiming

| Method | What it does |
| --- | --- |
| `aimAtHub` | [`AutoAlignToPoseCommand`]({{ '/commands/auto-align/' | relative_url }}) to the current position with the heading from `Drive#getAimRotationForHub()` (`AlignType.DEFAULT`). |
| `turnToHub` | Same target, `AlignType.ROTATION`: turns in place. |

Both are `DeferredCommand`s, so the target is taken when the command
starts, not when it's built.

## Shooting

| Method | What it does |
| --- | --- |
| `aimAndShoot` | Requests `HUB_TRACKING`, then `SHOOTING`. |
| `shootOrPassBasedOnPos` | `SHOOTING` if `RobotState#shouldShootHub()`, else `PASSING`. |
| `trackBasedOnPos` | `HUB_TRACKING` or `PASS_TRACKING`, by the same test. Aims without feeding. |
| `goToFixedPosAndShoot` | Drives to a fixed spot 80″ out from the hub face (alliance-flipped), then calls `shooter.holdShot(speed, hoodPosition, hoodFeedforward)` with `FixedPos/RPS`, `FixedPos/Hood` and `FixedPos/Hood FF`. |

`shouldShootHub()` is true when the robot is between its own alliance
wall and the hub; otherwise the shot becomes a pass to
`PassTargetFactory`'s spot.

## Utility

| Method | What it does |
| --- | --- |
| `shakeIntake` | Repeats intake `SHAKE` for 0.6 s, `IDLE` for 0.6 s, until cancelled. |

## Where these get bound

[`Controls.bind(state)`]({{ '/architecture/robot-state/' | relative_url }}#controls)
binds every button. The driver only drives; the operator runs the
mechanisms. Intake buttons are bound only when the robot has an intake,
shooter buttons only when it has a shooter:

| Operator input | Bound with | Command |
| --- | --- | --- |
| Left bumper | Intake | Intake down: `IDLE` (deployed, rollers off). |
| Left trigger (hold) | Intake | Intake `INTAKE` (down and spinning); `IDLE` on release. |
| Right bumper | Intake | Intake up: `STOW`. |
| Right trigger (hold) | Shooter | `shootOrPassBasedOnPos`; release runs `trackBasedOnPos`. |
| B (hold) | Intake | `shakeIntake`; `IDLE` on release. |
| A (hold) | Intake, shooter | Eject: intake `OUTAKE` and shooter `OUTTAKE`; release goes back to `IDLE` and `trackBasedOnPos`. |
| Y (hold) | Shooter | `fixedShot`: the hub-front shot from `FixedPos/RPS` and `FixedPos/Hood`, no vision needed. |
| X (hold) | Shooter | Unjam: `reverseFeed` while held, `releaseFeed` on release. |
| D-pad up / down | Shooter | Shot speed multiplier +2% / −2%. |
| Back | Shooter | Reset the shot speed multiplier. |
| Start | Any | `Superstructure.clearOverrides()`. |

The hood stays down unless the shooter is `SHOOTING` or `PASSING`.
Tracking keeps the flywheel spinning so a shot starts fast, but holds the
hood at its lower limit, so nothing has to watch for the trench.

## Adding a new action

```java
public static Command myNewAction(RobotState state) {
    return state.getSuperstructure().shooterCommand(shooter -> Commands.sequence(
            shooter.transitionCommand(Shooter.State.HUB_TRACKING),
            aimAtHub(state),
            shooter.transitionCommand(Shooter.State.SHOOTING)));
}
```

Then bind it in `Controls`, or in one robot's
[`Controls` subclass]({{ '/architecture/robots/' | relative_url }}#adding-a-robot),
or call it from an auto. Never bypass the state machines — always go
through `transitionCommand` or `requestTransition`.
