---
layout: default
title: Shooter Setpoint
eyebrow: Utilities
description: Distance-aware solver for turret angle, hood angle, and flywheel speed.
permalink: /utilities/shooter-setpoint/
---

| | |
| --- | --- |
| **Source** | [`ShooterSetpoint.java`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/ShooterSetpoint.java) |
| **Related** | [`TurretCalculator`]({{ '/utilities/turret-calculator/' | relative_url }}), [`BallTargetFactory`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/BallTargetFactory.java), [`PassTargetFactory`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/PassTargetFactory.java) |

A "shot" reduces to three numbers — turret yaw, hood angle, flywheel
RPS. This file computes them.

## The record

```java
public record ShooterSetpoint(
    double shooterRPS,
    double turretRadiansFromCenter,
    double hoodAngleRadians,
    double turretFF,
    double hoodFF
) {}
```

- **`shooterRPS`** — flywheel target velocity, rotations per second.
- **`turretRadiansFromCenter`** — turret yaw relative to the chassis
  center.
- **`hoodAngleRadians`** — hood angle in radians.
- **`turretFF`** — feedforward voltage component for the turret. Used
  to compensate for chassis angular velocity so the turret stays
  locked while the robot yaws.
- **`hoodFF`** — feedforward for the hood (small; mostly gravity
  compensation).

## Factory methods

```java
// Aim at the hub
Supplier<ShooterSetpoint> ShooterSetpoint.speakerSetpointSupplier(RobotState state);

// Aim at a teammate for a pass
Supplier<ShooterSetpoint> ShooterSetpoint.passSetpointSupplier(RobotState state);
```

Both return *suppliers*, not values. The supplier captures
`RobotState`; each `.get()` recomputes from the latest pose, target,
and chassis speed.

`RobotState` exposes these as:

```java
robotState.getCurrentHubSetpoint();   // returns the speaker supplier
robotState.getCurrentPassSetpoint();  // returns the pass supplier
```

## How the solve works

For each call to the supplier:

1. **Choose target.** `BallTargetFactory` (hub) or `PassTargetFactory`
   (teammate pass) returns a 3D target.
2. **Predict robot pose.** Call
   `RobotState#getPredictedFieldToRobot()` for where the robot will
   be when the shot leaves the flywheel (accounts for shot latency).
3. **Compute distance.** `TurretCalculator#getDistanceToTarget` does
   the 3D distance from the *turret* (not robot center) to the target.
4. **Choose flywheel RPS.** From a distance → RPS function (currently
   a lookup curve tunable via `GetTuned`).
5. **Solve hood angle.** Using projectile motion with the chosen
   muzzle speed, solve for the launch angle that intersects the target
   height. `TurretCalculator#calculateAngleFromVelocity` does this.
6. **Solve turret angle.** `GeomUtil#angleTo` gives the yaw to point
   at the target's XY.
7. **Feedforward.** Add chassis-velocity-derived terms so a yawing
   chassis doesn't cause the turret to lag.
8. **Apply overrides.** If `Flywheel/Override/RPS` is set (via
   `GetTuned`), use it instead. Same for hood and turret.

## Tuning overrides

For calibration days, you can pin individual outputs from the
dashboard:

| Key | Effect |
| --- | --- |
| `Shooter/Override/RPS` | Force a flywheel RPS regardless of distance. |
| `Shooter/Override/Hood` | Force a hood angle. |
| `Shooter/Override/Turret` | Force a turret angle relative to chassis. |
| `Shooter/Override/Enabled` | Master switch. |

Toggle `Enabled` off to fall back to the computed solution.

## Pitfalls

- **Shots are short.** The distance → RPS curve is wrong, or the
  shot-latency estimate is wrong. Calibrate one variable at a time.
- **Shots curl.** Turret feedforward gain is wrong. Check
  `Shooter/TurretFF/kV`.
- **Turret oscillates between two solutions.** Possible if the target
  is very close and the angle calculation has two roots. The solver
  picks the lower-angle root by default; check it isn't picking the
  high one on rare frames.
