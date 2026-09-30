---
layout: default
title: Drive Commands
eyebrow: Commands
description: The default drive command and the characterization routines.
permalink: /commands/drive-commands/
---

[`DriveCommands`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/commands/DriveCommands.java)
is the home of all `Command`s that act on `Drive` exclusively. It is
shared by every robot and reads its limits from `drive.getConfig()`.

## `smartDrive`

```java
public static Command smartDrive(
    Drive drive,
    DoubleSupplier x, DoubleSupplier y, DoubleSupplier manualOmega,
    Supplier<Rotation2d> autoRotationGoal,
    Supplier<Drive.State> stateSupplier);
```

The drive's default command, set in `Drive`'s constructor from the
driver controller in `Controls`:

- Left stick → field-relative translation. Deadband 0.1, squared, scaled to `drive.getMaxLinearSpeedMetersPerSec()` (the slow limit in `SLOW`).
- Right stick X → rotation. Deadband 0.1, squared, scaled to `drive.getMaxAngularSpeedRadPerSec()`.
- In `TRAVERSING_AT_ANGLE` (and `SLOW`, which maps to it), a profiled PID drives the heading to `autoRotationGoal` — `Drive#getAimRotationForHub()` — instead of the right stick. Its kP is tunable as `Auto Turn`.
- Field-relative frame flips 180° on the red alliance.

## Characterization

Neither routine is bound to a button or the auto chooser. Schedule one
from a temporary binding when you need it.

### `wheelRadiusCharacterization(Drive)`

Slowly spins the robot in place. Compares the gyro yaw to the
integrated wheel travel and prints the effective wheel radius, using
`config.driveBaseRadius()`. Useful when wheel wear has shifted from
nominal; update `wheelRadiusMeters` in the robot's `DriveConfig`.

### `feedforwardCharacterization(Drive)`

Ramps drive output at 0.1 V/s and records velocity, then fits `kS` and
`kV`. Results print to the console and go to `Character/kS` and
`Character/kV` on SmartDashboard.

### SysId

`drive.sysIdQuasistatic(direction)` and `drive.sysIdDynamic(direction)`
return the WPILib SysId commands. Run each direction, then process the
log with the SysId tool.

## Pitfalls

- **Robot drifts at neutral stick.** Raise `kDeadband` in `DriveCommands`.
- **SysId looks terrible.** Make sure the floor isn't carpeted in a
  way that varies friction — sweeps need consistent traction.
- **Wheel radius char gives a strange number.** Check that the gyro
  yaw is wrapped correctly — if it folds at ±180 mid-spin, the math
  diverges.
