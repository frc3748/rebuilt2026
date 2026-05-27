---
layout: default
title: Turret Calculator
eyebrow: Utilities
description: 3D geometry and projectile-motion math for the shooter.
permalink: /utilities/turret-calculator/
---

[`TurretCalculator`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/TurretCalculator.java)
is the math kernel behind the shooter. It's all static methods, all
unit-aware via WPILib `Measure` types.

## `getDistanceToTarget`

```java
public static Distance getDistanceToTarget(
    RobotState state,
    Pose3d robotPose,
    Pose3d target);
```

Returns the 3D distance from the **turret center** (not robot center)
to the target. Uses the turret-to-robot transform from
[`VisionConstants`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/subsystems/vision/VisionConstants.java).

## `calculateAngleFromVelocity`

```java
public static Angle calculateAngleFromVelocity(
    LinearVelocity muzzleSpeed,
    Distance horizontalDist,
    Distance verticalDist,
    LinearAcceleration gravity);  // typically -9.81 m/s²
```

Solves projectile motion for the launch angle that hits the target
given a known muzzle speed. The relevant equation:

```
v² · cos²(θ) · [ tan(θ) · d − ½ · g · d² / (v² · cos²(θ)) ] = h
```

Where `d` is horizontal distance, `h` is height above launch, `v` is
muzzle speed, `g` is gravity, `θ` is launch angle.

Two roots exist (a low arc and a high arc). The implementation returns
the low arc — better for a flat trajectory and a tighter time window.

Returns `null` (or throws, depending on the variant) if the target is
out of range — no angle satisfies the equation.

## Unit safety

The signatures use WPILib's `Measure` system:

- `Distance` (meters)
- `Angle` (radians or degrees, interconverting safely)
- `LinearVelocity` (m/s)
- `LinearAcceleration` (m/s²)

This is intentional: shooter math is exactly the kind of code where a
silently-mixed unit (cm vs m) destroys an entire match. The Measure
types catch it at compile time.

## Tunable gravity

Gravity is exposed as a `GetTuned` value (`Shooter/Physics/Gravity`)
so you can fudge it to compensate for ball drag — real game pieces
don't follow ideal projectile motion. The default is `-9.81 m/s²` but
calibration may push it toward `-10` or `-11` to account for drag.

## Pitfalls

- **`null` returns.** The high arc is unreachable, or your muzzle
  speed is below the minimum required for the distance. Caller must
  null-check.
- **Off by π.** Sign conventions on angle. `calculateAngleFromVelocity`
  returns the angle *above* horizontal; if you pass it directly to a
  hood that measures *from* horizontal, you're fine — but verify on
  the bench, don't trust the doc.
- **Position drift accumulates.** The function is correct for one
  instant; over time, errors in `getDistanceToTarget` (from pose drift)
  dominate the launch error. Vision corrections matter more than
  exact gravity.
