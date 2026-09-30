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

## States

```java
enum State {
  UNDETERMINED, IDLE, INTAKE, OUTAKE, STOW, SHAKE
}
```

| State | Behavior |
| --- | --- |
| `IDLE` | Pivot held at stow angle, rollers off. |
| `INTAKE` | Pivot down to ground angle, rollers in. |
| `OUTAKE` | Pivot down, rollers reversed. |
| `STOW` | Same as `IDLE` but signals "intentional stow". |
| `SHAKE` | Oscillates the pivot ±N degrees to unjam stuck pieces. |

`SHAKE` is wired through `ActionCommands.shakeIntake()`, which is
typically bound to a "did the piece get stuck?" operator button.

## Trench constraint

When the robot is near a trench (see [`TrenchZone`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/TrenchZone.java)),
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

- **Pivot motor** — NEO with absolute encoder. Closed-loop position with
  feedforward against gravity (cosine of arm angle × `kG`).
- **Roller motors** — NEO 550s; open-loop voltage.

## Logging

The intake publishes a 3D pose for its arm joint to AdvantageScope,
so you can see the deployed/stowed angle on the simulated field.

## Pitfalls

- **"Stuck" intake on boot.** Check the absolute encoder offset in
  `IntakeConstants`. If the reported angle disagrees with the visual
  position, the rest of the math will all be wrong.
- **Pieces eject before reaching hopper.** Roller voltage too high.
  Tune `IntakeConstants.kIntakeRollerVolts` down.
