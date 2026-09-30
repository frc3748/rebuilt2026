---
layout: default
title: Field Constants
eyebrow: Reference
description: 2026 field dimensions, tag IDs, hub and trench geometry, and alliance flipping.
permalink: /reference/field-constants/
---

Field constants live in
[`FieldConstants`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/game/FieldConstants.java),
with the rest of the 2026-game code in `frc.robot.game`. This page
summarizes the values you'll reach for most often.

## Field dimensions

| Constant | Value |
| --- | --- |
| `FIELD_LENGTH` | 650.12″ (~16.51 m) |
| `FIELD_WIDTH` | 316.64″ (~8.04 m) |
| `TAG_LAYOUT` | `AprilTagFields.k2026RebuiltAndymark` |
| `LAYOUT_LENGTH_METERS` / `LAYOUT_WIDTH_METERS` | Read from `TAG_LAYOUT`. Used for alliance flipping and vision's off-field check. |

## AprilTags

| Property | Value |
| --- | --- |
| `TAG_IDS` | All 32 tags on the field. |
| Family | AprilTag 36h11 |

`CameraIOLimelight` passes `TAG_IDS` to
`LimelightHelpers.SetFiducialIDFiltersOverride`, so a Limelight only
solves from those tags.

## Hub and funnel

| Constant | Value |
| --- | --- |
| `HUB_BLUE` | 181.56″ from the blue wall, centered in Y, 56.4″ high. |
| `HUB_RED` | Mirror of `HUB_BLUE`. |
| `HUB_NEAR_FACE` | Pose of tag 26. `goToFixedPosAndShoot` drives to 80″ in front of it. |
| `FUNNEL_RADIUS` | 24″ |
| `FUNNEL_HEIGHT` | 72″ − 56.4″ |

## Target positions

Hub and pass targets come from factories rather than constants. Both
return a `Translation3d` for
[`ShooterSetpoint`]({{ '/utilities/shooter-setpoint/' | relative_url }}):

- [`BallTargetFactory.generate(state)`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/game/BallTargetFactory.java)
  — our alliance's hub, raised by a distance-based height map.
- [`PassTargetFactory.generate(state)`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/game/PassTargetFactory.java)
  — `PASSING_SPOT_LEFT` or `PASSING_SPOT_RIGHT`, whichever matches the
  robot's side of the field, flipped for red.

## Trench zones

[`TrenchZone`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/game/TrenchZone.java)
places four trenches from `TRENCH_BUMP_X` and `TRENCH_WIDTH`, mirrored
for both alliances. Every method takes the `RobotState`:

| Method | Used by |
| --- | --- |
| `intakeLowerRequired(state)` | `Intake.applyConstraints` — within 1.0 m of a trench. |
| `hoodLowerRequired(state)` | The hood's trench clamp — within 0.8 m. |
| `driveRotationOverrideRequired(state)` | `Drive#getAimRotationForHub` in `SLOW`: square up to 0° or 180°. |
| `getDistanceToClosestShootingPose(state)` | Logged as `Distance to Hub`. |

## Coordinate conventions

- **Origin** — bottom-left of the field, as seen from the blue
  alliance station.
- **+X** — toward the red alliance.
- **+Y** — to the left.
- **+yaw** — counter-clockwise (right-hand rule with +Z up).

These match WPILib's standard field convention. Don't fight it.

## Alliance handling

[`AllianceFlip`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/game/AllianceFlip.java)
does the mirroring:

```java
AllianceFlip.isRed();              // DriverStation alliance, Blue if unknown
AllianceFlip.flip(pose);           // rotate a Pose2d or Translation3d 180° about the field center
AllianceFlip.forAlliance(bluePose) // flip only on red
```

Write field positions for blue and flip them at the point of use.
Alliance is only known once the Driver Station connects, so compute
alliance-dependent values late (in a `DeferredCommand` or a supplier),
not at robot init.

## Pitfalls

- **Coordinates off by mirror.** Almost always alliance handling.
  Verify `DriverStation.Alliance` matches the dashboard.
- **Target seems off-center.** `BallTargetFactory` adds a
  distance-based height and offset to the hub center. Check its maps.
