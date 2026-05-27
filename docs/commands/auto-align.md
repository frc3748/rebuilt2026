---
layout: default
title: AutoAlign
eyebrow: Commands
description: Profiled-PID alignment of the chassis to an arbitrary field pose.
permalink: /commands/auto-align/
---

[`AutoAlignToPoseCommand`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/commands/AutoAlignToPoseCommand.java)
drives the chassis to a target pose using two independent profiled-PID
controllers: one for translation, one for heading.

It's the underlying primitive behind:

- "Drive to scoring location" autos.
- `ActionCommands.goToFixedPosAndShoot`.
- `ActionCommands.autoClimb`'s initial positioning step.

## Constructor signature

```java
new AutoAlignToPoseCommand(
  Drive drive,
  Supplier<Pose2d> target,
  AlignType type,
  double constraintScale);
```

| Parameter | Meaning |
| --- | --- |
| `drive` | The drive to command. |
| `target` | A supplier — re-evaluated every loop, so you can target a moving point. |
| `type` | One of `DEFAULT`, `TRANSLATION_ONLY`, `ROTATION_ONLY`. |
| `constraintScale` | Scales max velocity / accel. `1.0` = normal, `0.5` = gentle, `1.5` = aggressive (use with caution). |

## `AlignType`

```java
enum AlignType { DEFAULT, TRANSLATION_ONLY, ROTATION_ONLY }
```

- **`DEFAULT`** — both controllers active. Robot drives to the pose
  and ends up at the heading.
- **`TRANSLATION_ONLY`** — heading controller off; robot keeps
  whatever yaw it had.
- **`ROTATION_ONLY`** — translation controller off; robot snaps to
  the target heading in place.

## Tolerances

Tolerances live in `DriveConstants` and gate `isFinished()`:

| Constant | Meaning |
| --- | --- |
| `kAutoAlignPositionTolerance` | Meters; both X and Y must be within this. |
| `kAutoAlignAccelTolerance` | Meters/s²; translation acceleration must be below this (so we don't declare done while still moving). |
| `kAutoAlignHeadingTolerance` | Radians. |
| `kAutoAlignAngularAccelTolerance` | Rad/s². |

The command finishes only when **all four** are satisfied for at least
a few consecutive loops.

## Tuning via DogLog

Every gain in this command is wrapped in
[`GetTuned`]({{ '/utilities/get-tuned/' | relative_url }}) so you can
adjust them mid-match from the dashboard:

- `AutoAlign/Translation/kP`, `kI`, `kD`
- `AutoAlign/Heading/kP`, `kI`, `kD`
- `AutoAlign/MaxVelocity`, `AutoAlign/MaxAccel`

## Usage examples

### Drive to a fixed pose

```java
new AutoAlignToPoseCommand(
  robotState.getDrive(),
  () -> new Pose2d(2.5, 5.0, Rotation2d.fromDegrees(90)),
  AlignType.DEFAULT,
  1.0);
```

### Snap to a specific heading without moving

```java
new AutoAlignToPoseCommand(
  robotState.getDrive(),
  () -> new Pose2d(robotState.getLatestFieldToRobot().getTranslation(),
                   Rotation2d.fromDegrees(0)),
  AlignType.ROTATION_ONLY,
  1.0);
```

### Track a moving target

```java
new AutoAlignToPoseCommand(
  robotState.getDrive(),
  () -> nearestScoringPose(robotState),  // recomputed every loop
  AlignType.DEFAULT,
  0.7);
```

## Pitfalls

- **Robot overshoots.** Decrease `MaxAccel` or `kP`. Acceleration is
  usually the issue, not P.
- **Robot oscillates near target.** Tighten the tolerance — too-loose
  tolerances let the command keep "re-correcting" inside the deadband.
- **Never finishes.** Heading and translation tolerances are
  independent; one being too tight blocks `isFinished()` forever.
  Check both in the AdvantageScope plot.
- **Vision corrects mid-align.** Expected and fine — but if it causes
  jumps, you may want to raise vision std devs during alignment.
