---
layout: default
title: Field Constants
eyebrow: Reference
description: 2026 field dimensions, tag IDs, and game-piece geometry.
permalink: /reference/field-constants/
---

Field-relative constants live in
[`VisionConstants`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/subsystems/vision/VisionConstants.java)
and a few other utility classes. This page summarizes the values
you'll reach for most often.

## Field dimensions

| Constant | Value |
| --- | --- |
| Field length | ~16.54 m |
| Field width | ~8.21 m |
| Tag layout | 2026 Rebuilt & Andymark AprilTag layout |

## AprilTags

| Property | Value |
| --- | --- |
| Tag IDs in use | A subset of 1–32 (see `kValidTagIds` in `VisionConstants`) |
| Tag size | Standard FRC 6.5″ |
| Family | AprilTag 36h11 |

The `kValidTagIds` list is what `VisionSubsystem` will accept. Tags
outside that list are silently rejected even if Limelight reports
them — this rejects misidentified or invalid detections.

## Target positions

Hub and pass targets come from factories rather than constants:

- [`BallTargetFactory`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/BallTargetFactory.java)
  — hub target 3D pose. Alliance-aware (the hub is on different sides
  for blue vs. red).
- [`PassTargetFactory`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/PassTargetFactory.java)
  — teammate-pass target. Configurable; usually a region of the field
  where teammates are expected to be.

Each factory returns a `Pose3d` for use by
[`ShooterSetpoint`]({{ '/utilities/shooter-setpoint/' | relative_url }}).

## Trench zones

[`TrenchZone`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/TrenchZone.java)
defines the field regions where:

- The intake auto-deploys (`intakeLowerRequired(pose)`).
- A given scoring pose is "closest" (`getDistanceToClosestShootingPose(pose)`).

The zones are alliance-mirrored — code computes the right zone for
your current alliance automatically.

## Coordinate conventions

- **Origin** — bottom-left of the field, as seen from the blue
  alliance station.
- **+X** — toward the red alliance.
- **+Y** — to the left.
- **+yaw** — counter-clockwise (right-hand rule with +Z up).

These match WPILib's standard field convention. Don't fight it.

## Alliance handling

`DriverStation.getAlliance()` is queried lazily by the factories.
Code that constructs poses early (like at robot init, before FMS data
arrives) needs to defer alliance-dependent calculations until
`autonomousInit` or later.

When in doubt, use a `Supplier<Pose2d>` that recomputes — the rest of
the codebase already follows this pattern.

## Pitfalls

- **Coordinates off by mirror.** Almost always alliance handling.
  Verify `DriverStation.Alliance` matches the dashboard.
- **Tag rejected for no reason.** Check `kValidTagIds`. If you added
  a new tag, you have to add it to that list too.
- **Target seems off-center.** The factories include an offset for
  shot trajectory (you aim at the center of the goal, not the edge).
  Verify the constant in the factory.
