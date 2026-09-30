---
layout: default
title: Shooter Setpoint
eyebrow: Utilities
description: Distance-aware solver for hood angle, flywheel speed, and the heading the drive should hold.
permalink: /utilities/shooter-setpoint/
---

[`ShooterSetpoint`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/game/ShooterSetpoint.java)
lives in `frc.robot.game`. Each instance is one solved shot:

| Getter | Meaning |
| --- | --- |
| `getShooterRPS()` | Flywheel target, meters per second of surface speed. |
| `getAzimuthRadians()` | Robot-relative angle to the predicted target. |
| `getHoodRadians()` | Hood angle. |
| `getHoodFF()` | Small feedforward for chassis motion toward the target. |
| `getHeight()` | Height of the solved target. |

`ShooterSetpoint.hubSetpointSupplier(state)` and `passSetpointSupplier(state)`
recompute on every call; `RobotState` exposes them as
`getCurrentHubSetpoint()` and `getCurrentPassSetpoint()`.

Each solve goes through `state.getShotCalculator()`. On the real robot
the robot's shot map in `ShooterConstants` is interpolated and the
target is led using its time-of-flight map (tunable live under
`TOF Tuning/<distance>`). In simulation the funnel-clearance solve is
used. See [Shot Calculator]({{ '/utilities/shot-calculator/' | relative_url }}).
