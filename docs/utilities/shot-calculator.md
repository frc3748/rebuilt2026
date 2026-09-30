---
layout: default
title: Shot Calculator
eyebrow: Utilities
description: Projectile-motion math for the fixed shooter.
permalink: /utilities/shot-calculator/
---

[`ShotCalculator`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/game/ShotCalculator.java)
(in `frc.robot.game`) is the math behind the shooter. Everything is
measured from the shooter position (`ShooterConstants.kShooterToRobotCenter`).

| Method | Purpose |
| --- | --- |
| `getDistanceToTarget(robot, target)` | Horizontal distance from the shooter to a target. |
| `calculateAzimuthAngle(robot, target)` | Robot-relative angle to the target. |
| `calculateTimeOfFlight(...)` | Flight time used to lead a moving target. |
| `predictTargetPos(...)` | Moves the target against the robot's field velocity. |
| `calculateShotFromFunnelClearance(...)` | Solves exit velocity and hood angle so the arc clears the funnel lip. |
| `iterativeMovingShotFromMap(...)` | Real robot: interpolate `ShooterConstants.kShotMap`, iterate on the predicted target. |
| `iterativeMovingShotFromFunnelClearance(...)` | Simulation: same iteration with the physics solve. |

Gravity, funnel geometry and iteration counts are tunable under `Shooter/…`.
