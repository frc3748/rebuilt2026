---
layout: default
title: Kicker
eyebrow: Subsystem
description: A short final-stage roller that pushes the piece the last inch into the flywheel.
permalink: /subsystems/kicker/
---

| | |
| --- | --- |
| **Source** | `src/main/java/frc/robot/subsystems/kicker/` |
| **Public class** | [`Kicker`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/subsystems/kicker/Kicker.java) extends `StateMachine<Kicker.State>` |
| **Constants** | `KickerConstants`, from `CompRobot.kicker()` |
| **Built by** | `ShooterComp`, as a child subsystem |

The kicker is the simplest powered mechanism on the robot: one motor
that is off, feeding or reversing. It holds the piece between the
hopper and the flywheel, so the shooter can spin up without feeding
early.

## States

```java
enum State { UNDETERMINED, IDLE, SHOOT, OUTAKE }
```

| State | Behavior |
| --- | --- |
| `IDLE` | Motor off. The piece waits at the kicker. |
| `SHOOT` | Runs at `Kicker/Shot Speed` while the flywheel is ready, pushing the piece into it. Otherwise off. |
| `OUTAKE` | Runs at `Kicker/Outtake Speed`, backing the piece out. |

Like the hopper, it gets `Flywheel#isReady` from `ShooterComp`, and
`Kicker#feed()` runs at the shot speed without waiting, for the
operator's `forceFeed()` override.

## Mechanism

Spark MAX on CAN 42, velocity control. `KickerConstants` holds the
`MotorConfig` and the two speeds, which become `TunableNumber`s when the
kicker is built.

## Pitfalls

- **Piece won't transfer.** Raise the magnitude of `Kicker/Shot Speed`
  (the default is -40), and check the roller isn't slipping on the
  piece.
