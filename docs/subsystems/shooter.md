---
layout: default
title: Shooter
eyebrow: Subsystem
description: A fixed shooter, hood plus flywheel, orchestrated by one parent state machine that also drives the hopper and kicker.
permalink: /subsystems/shooter/
---

The shooter is fixed to the chassis: the drive rotates the robot to aim
(`Drive#getAimRotationForHub`), the hood sets the launch angle, and the
flywheel sets the exit speed. `Shooter` owns the hood and flywheel as
child state machines and requests hopper and kicker states so that the
whole shot is one transition.

| | |
| --- | --- |
| **Source** | `src/main/java/frc/robot/subsystems/shooter/` |
| **Children** | `Hood`, `Flywheel` |
| **Constants** | `ShooterConstants` (shot map, time-of-flight map), `HoodConstants`, `FlywheelConstants` |

## States

| Shooter | Flywheel | Hood | Hopper | Kicker |
| --- | --- | --- | --- | --- |
| `IDLE` | `IDLE` | `IDLE` | `IDLE` | `IDLE` |
| `HUB_TRACKING` | `TRACKING` | `HUB_TRACKING` | `IDLE` | `IDLE` |
| `PASS_TRACKING` | `TRACKING` | `PASS_TRACKING` | `IDLE` | `IDLE` |
| `SHOOTING` | `SHOOT` | `HUB_TRACKING` | `SHOOT` | `SHOOT` |
| `PASSING` | `PASS` | `PASS_TRACKING` | `SHOOT` | `SHOOT` |
| `OUTTAKE` | `IDLE` | `IDLE` | `OUTAKE` | `OUTAKE` |
| `TUNING` | `TUNING` | `TUNING` | `IDLE` | `IDLE` |

Hopper and kicker only feed while `Shooter#isReady()` is true, which is
the flywheel being within tolerance of its setpoint.

## Hood

Spark MAX, MAXMotion position, gravity feedforward, soft limits. Near a
trench the hood is clamped to `kMaxSetpointUnderTrench` unless an auto
has set `setAutoOverride(true)`.

## Flywheel

Two Spark Flex, leader plus follower, coast, velocity control in meters
per second of surface speed. A dashboard multiplier scales the hub shot.

## Shot map

`ShooterConstants` holds distance → (exit velocity, hood angle) and
distance → time-of-flight maps. On the real robot `ShooterSetpoint`
interpolates these; in simulation it uses the funnel-clearance solve in
[`ShotCalculator`]({{ '/utilities/shot-calculator/' | relative_url }}).

## Operator override

`Shooter#setOverride(setpoint, spinFlywheel)` freezes the hood and
flywheel on a captured setpoint until `clearOverride()`.

## Simulation

In simulation `Shooter` launches game pieces into `FuelSim` while in
`SHOOTING` or `PASSING`, and logs the trajectory under `Shooter/Trajectory`.
