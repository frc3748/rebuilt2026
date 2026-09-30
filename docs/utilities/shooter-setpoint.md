---
layout: default
title: Shooter Setpoint
eyebrow: Utilities
description: Distance-aware solver for hood angle, flywheel speed, and the heading the drive should hold.
permalink: /utilities/shooter-setpoint/
---

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

On the real robot the shot map in `ShooterConstants` is interpolated and
the target is led using the time-of-flight map (tunable live under
`TOF Tuning/<distance>`). In simulation the funnel-clearance solve is used.
