---
layout: default
title: Intake
eyebrow: Subsystem
description: Pivoting intake arm that pulls game pieces off the floor.
permalink: /subsystems/intake/
---

| | |
| --- | --- |
| **Source** | `src/main/java/frc/robot/subsystems/intake/` |
| **Public class** | [`Intake`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/subsystems/intake/Intake.java) (abstract) extends `StateMachine<Intake.State>`; `IntakeComp` extends it |
| **Constants** | `IntakeConstants`, from `CompRobot.intake()` |
| **Built by** | `CompRobot.createSuperstructure` |

`Intake` holds the `State` enum and `rollIn()` / `rollOut()`, the calls
shared code makes. `IntakeComp` holds the logic below. It takes an
`IntakeConstants` and builds its motors and its `Intake/…` tunables
from it, so a robot can change a setpoint by overriding `intake()`; see
[Per-robot constants]({{ '/architecture/robots/' | relative_url }}#per-robot-constants).
A robot with a different intake adds its own subclass; see
[Adding a subsystem variant]({{ '/architecture/robots/' | relative_url }}#adding-a-subsystem-variant).

## States

```java
enum State {
  UNDETERMINED, IDLE, INTAKE, OUTAKE, STOW, SHAKE
}
```

What `IntakeComp` does in each:

| State | Behavior |
| --- | --- |
| `STOW` | Extension to the stow setpoint, rollers off. |
| `IDLE` | Extension down at the intake setpoint, rollers off. |
| `INTAKE` | Extension down, rollers in. |
| `OUTAKE` | Extension to the outtake setpoint, rollers reversed. |
| `SHAKE` | Extension to the shake setpoint, rollers in. |

`ActionCommands.shakeIntake(state)` alternates `SHAKE` and `IDLE` every
0.6 s to knock stuck pieces loose. It runs while the driver holds **B**,
and in autos through `shake(seconds)`.

## Trench constraint

When the robot is near a trench (see [`TrenchZone`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/game/TrenchZone.java)),
the intake extension is forced to the intake setpoint so it fits under.
This lives in `IntakeComp.applyConstraints()`, so it wins over
every state and every operator override:

```java
@Override
protected void applyConstraints() {
    if (TrenchZone.intakeLowerRequired(robotState)) {
        extension.set(intakeSetpoint.get());
    }
}
```

## Operator override

```java
intake.setOverride(Intake.State.STOW);
intake.setOverride(intake::rollIn);
intake.clearOverride();
```

## Mechanism

- **Extension** — Spark MAX on CAN 46 with a follower on 47. MAXMotion
  position with cosine gravity feedforward; gains are tunable live.
- **Rollers** — Spark Flex on CAN 48, velocity control.

The extension uses its relative encoder. The "Intake Zero" dashboard
button resets it to 0.

## Logging

In simulation, the intake publishes `Intake/Pose` (the arm angle) and
`Intake/ExtensionPose` to AdvantageScope, so you can see it deploy on
the field view. The motors log under `Motors/Intake Roller` and
`Motors/Intake Extension` on every robot.

## Pitfalls

- **Setpoints are off by a constant.** The extension encoder reads 0
  wherever the arm was at boot. Move the arm to its zero position and
  press "Intake Zero".
- **Pieces eject before reaching the hopper.** Roller speed too high.
  Tune `Intake/Roller Intake Speed` down.
