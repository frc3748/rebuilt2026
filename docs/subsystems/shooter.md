---
layout: default
title: Shooter
eyebrow: Subsystem
description: A composite of turret, hood, and flywheel — three child state machines orchestrated by one parent.
permalink: /subsystems/shooter/
---

The shooter is the only subsystem in the codebase that owns child
subsystems. It coordinates three independent mechanisms — turret, hood,
flywheel — into a single shooting motion.

| | |
| --- | --- |
| **Source** | `src/main/java/frc/robot/subsystems/shooter/` |
| **Public class** | [`Shooter`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/subsystems/shooter/Shooter.java) extends `StateMachine<Shooter.State>` |
| **Children** | `Turret`, `Hood`, `Flywheel` |

## States

```java
public enum State {
  UNDETERMINED, IDLE,
  HUB_TRACKING, PASS_TRACKING,    // aim only
  SHOOTING, PASSING,              // aim + release
  OUTTAKE,                        // reverse rollers to eject
  TUNING                          // dashboard-driven; doesn't auto-recalc
}
```

State transitions follow a "aim-then-shoot" pattern:

```
       IDLE ──▶ HUB_TRACKING ──▶ SHOOTING ──▶ IDLE
              │
              └──▶ PASS_TRACKING ──▶ PASSING ──▶ IDLE
```

## How the parent commands its children

When `Shooter` enters `HUB_TRACKING`, its state command:

1. Tells `Turret` to enter `HUB_TRACKING`.
2. Tells `Hood` to enter `HUB_TRACKING`.
3. Tells `Flywheel` to enter `TRACKING`.

Each child has its own state machine, its own PID loop, and its own
setpoint supplier. The parent never reads sensors directly.

When `SHOOTING` is entered:

1. The parent waits until `flywheel.isReady() && hood.isReady() && turret.isReady()`.
2. Then it requests `Hopper.SHOOT` and `Kicker.SHOOT`.
3. After a short timeout, it returns to `IDLE`.

This pattern keeps each subsystem focused on its own job.

## Turret

| | |
| --- | --- |
| **Source** | `subsystems/shooter/turret/` |
| **States** | `IDLE`, `HUB_TRACKING`, `PASS_TRACKING`, `TUNING` |
| **Constants** | `TurretConstants` |

A continuous-rotation turret driven by a NEO with an absolute encoder.
Setpoint comes from [`TurretCalculator`]({{ '/utilities/turret-calculator/' | relative_url }}),
which computes the angle from the turret's 3D position to the target
position.

The turret feeds its current angle into
[`RobotState`]({{ '/architecture/robot-state/' | relative_url }})'s
turret angle buffer on every loop. The vision pipeline reads this
buffer to correct for the turret angle *at the time the frame was
captured*.

### Feedforward

The turret setpoint includes a feedforward term proportional to the
robot's angular velocity, so it stays locked on a target while the
chassis is yawing. The term is calculated in `ShooterSetpoint`.

## Hood

| | |
| --- | --- |
| **Source** | `subsystems/shooter/hood/` |
| **States** | `IDLE`, `HUB_TRACKING`, `PASS_TRACKING`, `TUNING` |
| **Constants** | `HoodConstants` |

A motorized hood that adjusts the launch angle. Driven open-loop in
`TUNING`, profiled-PID in `HUB_TRACKING`. Setpoint comes from the same
`ShooterSetpoint` supplier as the flywheel — it's a function of
distance to the target.

## Flywheel

| | |
| --- | --- |
| **Source** | `subsystems/shooter/flywheel/` |
| **States** | `IDLE`, `TRACKING`, `SHOOT`, `PASS`, `TUNING` |
| **Constants** | `FlywheelConstants` |

Two NEOs in a master/follower configuration spinning a flywheel wheel.
Velocity control with feedforward (`kS`, `kV`, `kA`).

`isReady()` returns true when actual RPS is within tolerance of
setpoint for at least a few consecutive loops (avoids twitchy
`true`/`false` flapping at edges).

## The shooting solver

A "shot" reduces to three numbers — turret yaw, hood angle, flywheel
RPS — that depend on:

- Target 3D position (`BallTargetFactory` or `PassTargetFactory`).
- Robot 3D position (from drive's pose estimator, projected forward to release time).
- Robot velocity (for motion compensation).

This calculation lives in
[`ShooterSetpoint`]({{ '/utilities/shooter-setpoint/' | relative_url }}).
It returns a `ShooterSetpoint` record:

```java
record ShooterSetpoint(
  double shooterRPS,
  double turretRadiansFromCenter,
  double hoodAngleRadians,
  double turretFF,
  double hoodFF
) {}
```

The supplier form means the value gets recomputed *every loop*, so a
moving robot keeps the aim true.

## Tuning

`State.TUNING` lets you override the solver from the dashboard. The
flywheel respects `Flywheel/Override/RPS`, the hood respects
`Hood/Override/Angle`, the turret respects `Turret/Override/Angle`.
Useful for shot calibration days where you want to find the right
combination for a specific distance.

## Public API

```java
Turret   getTurret();
Hood     getHood();
Flywheel getFlywheel();
```

Plus the inherited `StateMachine` API.

## Diagram

```
           HUB_TRACKING                    SHOOTING
                │                              │
                ▼                              ▼
   ┌──────────────────────┐         ┌──────────────────┐
   │  Turret → HUB_TRACK  │         │  await isReady   │
   │  Hood   → HUB_TRACK  │         │   on all three   │
   │  Flywh. → TRACKING   │         │       ↓          │
   └──────────────────────┘         │  Hopper → SHOOT  │
            │                       │  Kicker → SHOOT  │
            │ supplier recalculated │       ↓          │
            │ every loop            │  wait for clear  │
            ▼                       └──────────────────┘
   ShooterSetpoint:
     (shooterRPS, turretRad,
      hoodRad, turretFF, hoodFF)
```
