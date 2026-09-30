---
layout: default
title: AutoAlign
eyebrow: Commands
description: Profiled-PID alignment of the chassis to an arbitrary field pose.
permalink: /commands/auto-align/
---

[`AutoAlignToPoseCommand`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/commands/AutoAlignToPoseCommand.java)
drives the chassis to a target pose using two profiled-PID controllers:
one on the distance to the target, one on heading.

It's the underlying primitive behind:

- `ActionCommands.aimAtHub` and `turnToHub`.
- `ActionCommands.goToFixedPosAndShoot`.
- `PathAuto.nudge(meters)`.

## Constructor signature

```java
new AutoAlignToPoseCommand(Drive drive, RobotState state, Pose2d target, double constraintFactor);
new AutoAlignToPoseCommand(Drive drive, RobotState state, Pose2d target, double constraintFactor, AlignType alignType);
```

| Parameter | Meaning |
| --- | --- |
| `drive` | The drive to command. The command requires it. |
| `state` | Where the current pose and speeds are read from. |
| `target` | A fixed field pose. Wrap the command in a `DeferredCommand` to compute the target when it starts. |
| `constraintFactor` | Scales max velocity and acceleration. `1.0` is normal. |
| `alignType` | `DEFAULT` if omitted. |

## `AlignType`

```java
enum AlignType { DEFAULT, ROTATION, TRANSLATION }
```

- **`DEFAULT`** — both controllers active; finishes when both are at goal.
- **`ROTATION`** — turns in place; finishes when the heading is at goal.
- **`TRANSLATION`** — heading held still; finishes when the position is at goal.

## Constants and tolerances

Everything comes from the robot's
[`DriveConfig`]({{ '/subsystems/drive/' | relative_url }}#driveconfig)
through `drive.getConfig()`:

| Field | Used for |
| --- | --- |
| `driveToPointP` / `driveToPointHeadingP` | Translation and heading kP. |
| `maxSpeedMetersPerSec`, `maxLinearAcceleration` | Translation profile, times `constraintFactor`. |
| `maxAngularSpeed()`, `maxAngularAcceleration()` | Heading profile. |
| `metersTolerance` | Translation goal tolerance, 0.04 m by default. |
| `radiansTolerance` | Heading goal tolerance, 2° by default. |

## Tuning

Each gain and tolerance is also a
[`TunableNumber`]({{ '/utilities/tunable-number/' | relative_url }}), so
you can adjust it from the dashboard without a redeploy:

- `Auto Align/Drive kP`, `Auto Align/Turn kP`
- `Auto Align/Meters Tolerance`, `Auto Align/Radians Tolerance`

They are created once, in a static `Tuning` shared by every
`AutoAlignToPoseCommand`, and applied to the controllers in
`initialize()`, so an edit takes effect on the next align.

Copy good values back into the `DriveConfig`; tunables reset on reboot.
See [Tuning]({{ '/utilities/tunable-number/' | relative_url }}).

## Usage examples

### Drive to a fixed pose

```java
new AutoAlignToPoseCommand(drive, state, new Pose2d(2.5, 5.0, Rotation2d.fromDegrees(90)), 1.0);
```

### Turn in place to a heading chosen at start

```java
new DeferredCommand(() -> {
    Pose2d pose = state.getLatestFieldToRobot().getValue();
    Pose2d aimed = new Pose2d(pose.getTranslation(), drive.getAimRotationForHub());
    return new AutoAlignToPoseCommand(drive, state, aimed, 1, AlignType.ROTATION);
}, Set.of(drive));
```

This is what `ActionCommands.turnToHub` does.

## Logging

`initialize()` logs the goal as `DriveToPose/Target` and sets
`DriveToPose/Active` to `true`; `end()` sets it back to `false`.

## Pitfalls

- **Robot overshoots.** Lower `constraintFactor` or `Auto Align/Drive kP`.
- **Never finishes.** Heading and translation tolerances are
  independent; one being too tight blocks `isFinished()` in `DEFAULT`.
  Check both in AdvantageScope.
- **Vision corrects mid-align.** Expected and fine — but if it causes
  jumps, raise the camera's `stdDevFactor`.
