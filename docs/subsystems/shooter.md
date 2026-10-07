---
layout: default
title: Shooter
eyebrow: Subsystem
description: A fixed shooter, hood plus flywheel, orchestrated by one parent state machine that also owns the hopper and kicker.
permalink: /subsystems/shooter/
---

The shooter is fixed to the chassis: the drive rotates the robot to aim
(`Drive#getAimRotationForHub`), the hood sets the launch angle, and the
flywheel sets the exit speed. `ShooterComp` builds and owns the
hood, flywheel, hopper and kicker as child state machines, so the whole
shot is one transition.

| | |
| --- | --- |
| **Source** | `src/main/java/frc/robot/subsystems/shooter/` |
| **Classes** | `Shooter` (abstract base), `ShooterComp` |
| **Built by** | `CompRobot.createSuperstructure` |
| **Children** | `Hood`, `Flywheel`, `Hopper`, `Kicker` |
| **Constants** | `FlywheelConstants`, `HoodConstants`, `HopperConstants`, `KickerConstants` from `CompRobot`; `ShooterConstants` from `RobotDefinition.shooter()`. See [Per-robot constants]({{ '/architecture/robots/' | relative_url }}#per-robot-constants). |

`Shooter` is the abstract base. It holds the `State` enum and the calls
that shared code (autos, `ActionCommands`, `Controls`) makes:
`spinUp()`, both `holdShot` overloads, `releaseShot()`, `stopFeed()`,
`reverseFeed()`, `forceFeed()` and `releaseFeed()`. `isFiring()`
(`SHOOTING` or `PASSING`) and `isPassing()` (`PASSING` or
`PASS_TRACKING`) come from the state. A robot with a different shooter
adds its own subclass; see
[Adding a subsystem variant]({{ '/architecture/robots/' | relative_url }}#adding-a-subsystem-variant).

## States

`ShooterComp` maps each state to its children in `registerStateCommands()`:

| Shooter | Flywheel | Hood | Hopper | Kicker |
| --- | --- | --- | --- | --- |
| `IDLE` | `IDLE` | `IDLE` | `IDLE` | `IDLE` |
| `HUB_TRACKING` | `TRACKING` | `HUB_TRACKING` | `IDLE` | `IDLE` |
| `PASS_TRACKING` | `TRACKING` | `PASS_TRACKING` | `IDLE` | `IDLE` |
| `SHOOTING` | `SHOOT` | `HUB_TRACKING` | `SHOOT` | `SHOOT` |
| `PASSING` | `PASS` | `PASS_TRACKING` | `SHOOT` | `SHOOT` |
| `OUTTAKE` | `IDLE` | `IDLE` | `OUTAKE` | `OUTAKE` |
| `TUNING` | `TUNING` | `TUNING` | `IDLE` | `IDLE` |

In `SHOOT`, the hopper and kicker only feed while `Flywheel#isReady()`
is true, which is the flywheel being within tolerance of its setpoint.

## Hood

Spark MAX, MAXMotion position, gravity feedforward. `Hood` applies the
conversion from `radiansPerRotation`, the soft limits from `minLimit` and
`maxLimit` (25° to 50°), and a starting position of `minLimit` when it
is built; "Zero hood" on the dashboard's Test tab resets it to `minLimit`,
so press it with the hood resting on its bottom stop. If the hood's Spark
reboots (a power blip drops it), it comes back reading 0°, so the IO sets
it back to `minLimit`, where the unpowered hood has fallen to. Every hood command
is clamped to the limits. The hood is driven down to `minLimit` in
`IDLE`, and the shooter's tracking states use `IDLE`, so the hood only
rises while shooting or passing.

## Flywheel

Two Spark Flex, leader plus follower, coast, velocity control in meters
per second of surface speed. `Flywheel` works out the conversion from
`FlywheelConstants.radius` when it is built. A dashboard multiplier
scales the hub shot.

## Shot map

`ShooterConstants` holds distance → (exit velocity in m/s, hood angle)
and distance → time-of-flight maps, filled by `addShot` in its
constructor. `RobotState` creates one per robot. On the real robot
[`ShotCalculator`]({{ '/utilities/shot-calculator/' | relative_url }})
interpolates these; in simulation it uses its funnel-clearance solve.
`ShooterConstants.tune()` makes every shot-table row tunable under
`Shot Table/<distance>/` (exit velocity, hood, time of flight), plus the
time-of-flight offset. Saved values are written back into the
`addShot(...)` lines.

## Operator override

`holdShot(setpoint, spinFlywheel)` freezes the hood and flywheel on a
captured setpoint, and `holdShot(speed, hoodPosition, hoodFeedforward)`
on fixed values, until `releaseShot()`. `stopFeed()`, `forceFeed()` and
`reverseFeed()` override the hopper and kicker until `releaseFeed()`.

## Logging

While the shooter isn't `IDLE` or `UNDETERMINED`, `ShooterComp.update()`
logs the setpoint it is using (pass if `isPassing()`, hub otherwise):
`Shooter/Setpoint/Speed` (m/s), `Shooter/Setpoint/HoodAngle` and
`Shooter/Setpoint/AimError` (the robot-relative azimuth), both in
radians. The flywheel logs `Flywheel/Ready`, and every motor logs under
`Motors/<name>`.

## Simulation

In simulation the `Superstructure` launches game pieces through the
[`FuelSimulation`]({{ '/utilities/simulation/' | relative_url }})
while the shooter `isFiring()`, and `ShotVisualizer` logs the
trajectory under `Shooter/Trajectory` for as long as it fires.
