---
layout: default
title: Action Commands
eyebrow: Commands
description: High-level composite commands for the competition robot — what driver buttons and autos actually invoke.
permalink: /commands/action-commands/
---

[`ActionCommands`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/robots/competition/ActionCommands.java)
is the "buttons-to-behavior" layer for the competition robot. It lives
in `robots/competition/` because every method needs that robot's
mechanisms.

Think of it as the playbook: each method is one named play that the
driver or an auto can call.

## Factory pattern

Every method is `static`, takes the `CompetitionSuperstructure`, and
returns a `Command`:

```java
public static Command shakeIntake(CompetitionSuperstructure robot)
public static Command aimAtHub(CompetitionSuperstructure robot)
public static Command turnToHub(CompetitionSuperstructure robot)
public static Command aimAndShoot(CompetitionSuperstructure robot)
public static Command shootOrPassBasedOnPos(CompetitionSuperstructure robot)
public static Command trackBasedOnPos(CompetitionSuperstructure robot)
public static Command goToFixedPosAndShoot(CompetitionSuperstructure robot)
```

The superstructure gives access to the mechanisms (`getShooter()`,
`getIntake()`, …) and `robot.state()` gives the `RobotState`.

> **Keep this signature.** [`CustomAuto`]({{ '/commands/autos/' | relative_url }}#customauto)
> finds actions by reflection: every public static method that takes
> exactly one `CompetitionSuperstructure` and returns `Command` shows
> up in its dashboard choosers automatically.

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
| `goToFixedPosAndShoot` | Drives to a fixed spot 80″ out from the hub face (alliance-flipped), then overrides the flywheel and hood with `FixedPos/RPS`, `FixedPos/Hood` and `FixedPos/Hood FF`. |

`shouldShootHub()` is true when the robot is between its own alliance
wall and the hub; otherwise the shot becomes a pass to
`PassTargetFactory`'s spot.

## Utility

| Method | What it does |
| --- | --- |
| `shakeIntake` | Repeats intake `SHAKE` for 0.6 s, `IDLE` for 0.6 s, until cancelled. |

## Where these get bound

In `CompetitionSuperstructure.bindDriver`:

| Driver input | Command |
| --- | --- |
| Right trigger | `drive.stopWithX()`, then `shootOrPassBasedOnPos`; release runs `trackBasedOnPos`. |
| Left trigger | Intake `INTAKE` while held, `IDLE` on release. |
| Left bumper | Intake `STOW`. |
| A | Shooter `OUTTAKE` and intake `OUTAKE`; release returns to `trackBasedOnPos`. |
| X (hold) | `goToFixedPosAndShoot`; release calls `shooter.releaseShot()`. |
| B (hold) | `shakeIntake`. |
| D-pad up (hold) | `turnToHub`. |

The operator controller holds overrides. Hold **X** (hopper and
kicker), **B** (intake) or **A** (shooter) and press a trigger or bumper
to override that group, or the right stick to clear it. The left stick
clears every override.

Drive bindings shared by every robot are in
[`Controls`]({{ '/architecture/robot-state/' | relative_url }}#controls).

## Adding a new action

```java
public static Command myNewAction(CompetitionSuperstructure robot) {
    Shooter shooter = robot.getShooter();
    return Commands.sequence(
            shooter.transitionCommand(Shooter.State.HUB_TRACKING),
            aimAtHub(robot),
            shooter.transitionCommand(Shooter.State.SHOOTING));
}
```

Then bind it in `CompetitionSuperstructure`. It also appears in the
custom auto choosers. Never bypass the state machines — always go
through `transitionCommand` or `requestTransition`.
