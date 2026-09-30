---
layout: default
title: Shot Calculator
eyebrow: Utilities
description: Projectile-motion math for the fixed shooter.
permalink: /utilities/shot-calculator/
---

[`ShotCalculator`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/game/ShotCalculator.java)
(in `frc.robot.game`) is the math behind the shooter. `RobotState`
builds one from the robot's `ShooterConstants` and returns it from
`getShotCalculator()`. Everything is measured from that robot's shooter
position (`shooterToRobotCenter`); see
[Per-robot constants]({{ '/architecture/robots/' | relative_url }}#per-robot-constants).

| Method | Purpose |
| --- | --- |
| `getDistanceToTarget(robot, target)` | Horizontal distance from the shooter to a target. |
| `calculateAzimuthAngle(robot, target)` | Robot-relative angle to the target. |
| `calculateTimeOfFlight(...)` | Static. Flight time used to lead a moving target. |
| `predictTargetPos(...)` | Static. Moves the target against the robot's field velocity. |
| `calculateShotFromFunnelClearance(...)` | Solves exit velocity and hood angle so the arc clears the funnel lip. |
| `iterativeMovingShotFromMap(...)` | Real robot: interpolate `ShooterConstants.shotMap`, iterate on the predicted target. |
| `iterativeMovingShotFromFunnelClearance(...)` | Simulation: same iteration with the physics solve. |

Each returns a `ShotData`: exit velocity in meters per second, hood
angle in radians, and the (predicted) target.

Funnel gravity, funnel geometry and iteration counts are tunable under
`Shooter/…`. The funnel height starts at the field's funnel height plus
`ShooterConstants.distanceAboveFunnel`.
