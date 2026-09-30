---
layout: default
title: Hopper
eyebrow: Subsystem
description: Internal conveyor that carries pieces from intake to shooter.
permalink: /subsystems/hopper/
---

| | |
| --- | --- |
| **Source** | `src/main/java/frc/robot/subsystems/hopper/` |
| **Public class** | [`Hopper`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/subsystems/hopper/Hopper.java) extends `StateMachine<Hopper.State>` |
| **Constants** | `HopperConstants`, from `CompRobot.hopper()` |
| **Built by** | `ShooterComp`, as a child subsystem |

## States

```java
enum State { UNDETERMINED, IDLE, OUTAKE, SHOOT }
```

| State | Behavior |
| --- | --- |
| `IDLE` | Motor off. |
| `SHOOT` | Runs at `Hopper/Shoot Speed`, feeding the kicker, but only while the flywheel is ready. Otherwise off. |
| `OUTAKE` | Runs at `Hopper/Outtake Speed`, the other way, to eject. |

`Hopper#feed()` runs at the shoot speed without waiting for the
flywheel. The operator's `forceFeed()` override uses it.

## Coordination with the shooter

`Hopper` doesn't decide when to shoot. Its parent, `ShooterComp`,
requests `SHOOT` when the shooter enters `SHOOTING` or `PASSING`, and
passes in `Flywheel#isReady` as a `BooleanSupplier`, so the hopper only
feeds once the flywheel is within tolerance of its setpoint.

## Mechanism

Spark Flex on CAN 15, velocity control. `HopperConstants` holds the
`MotorConfig`, the two speeds (turned into `TunableNumber`s when the
hopper is built), the roller radius and the pose origin.

## Logging

`update()` integrates the roller's velocity over `rollerRadiusMeters`
and, in simulation, publishes the spin as `Hopper/Pose` for mechanism
visualization in AdvantageScope. The motor logs under `Motors/Hopper`.
