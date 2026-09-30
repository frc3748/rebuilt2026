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
| **Public class** | [`Intake`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/subsystems/intake/Intake.java) extends `StateMachine<Intake.State>` |
| **Constants** | `IntakeConstants` |
| **Built by** | `CompetitionSuperstructure` |

## States

```java
enum State {
  UNDETERMINED, IDLE, INTAKE, OUTAKE, STOW, SHAKE
}
```

| State | Behavior |
| --- | --- |
| `STOW` | Extension to the stow setpoint, rollers off. |
| `IDLE` | Extension down at the intake setpoint, rollers off. |
| `INTAKE` | Extension down, rollers in. |
| `OUTAKE` | Extension to the outtake setpoint, rollers reversed. |
| `SHAKE` | Extension to the shake setpoint, rollers in. |

`ActionCommands.shakeIntake(robot)` alternates `SHAKE` and `IDLE` every
0.6 s to knock stuck pieces loose. On the competition robot it runs
while the driver holds **B**.

## Trench constraint

When the robot is near a trench (see [`TrenchZone`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/game/TrenchZone.java)),
the intake extension is forced to the intake setpoint so it fits under.
This lives in `applyConstraints()`, so it wins over every state and every
operator override:

```java
@Override
protected void applyConstraints() {
    if (TrenchZone.intakeLowerRequired(robotState)) {
        extension.set(kIntakeSetpoint.get());
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

The intake publishes `Intake/Pose` (the arm angle) and
`Intake/ExtensionPose` to AdvantageScope, so you can see it deploy on
the field view.

## Pitfalls

- **Setpoints are off by a constant.** The extension encoder reads 0
  wherever the arm was at boot. Move the arm to its zero position and
  press "Intake Zero".
- **Pieces eject before reaching the hopper.** Roller speed too high.
  Tune `Intake/Roller Intake Speed` down.
