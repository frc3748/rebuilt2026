---
layout: default
title: Drive Commands
eyebrow: Commands
description: Joystick mapping, characterization routines, and SysId tools for the drive.
permalink: /commands/drive-commands/
---

[`DriveCommands`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/commands/DriveCommands.java)
is the home of all `Command`s that act on `Drive` exclusively.

## Joystick drive

```java
public static Command joystickDrive(
    Drive drive,
    DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot);
```

The default teleop binding:

- Translation from left stick, scaled to `DriveConstants.kMaxLinearSpeed`.
- Rotation from right stick X, scaled to `DriveConstants.kMaxAngularSpeed`.
- Deadband + cubed response for fine control near zero.
- Field-relative; the gyro yaw rotates the joystick frame.

### `joystickDriveAtAngle`

Same as above, but the right stick selects a *heading* instead of an
angular velocity. A profiled-PID controller drives the chassis to that
heading while the left stick still moves.

## Characterization

### `wheelRadiusCharacterization(Drive)`

Slowly spins the robot in place. Compares the gyro yaw (truth) to the
integrated wheel position (estimate). The ratio gives you the
effective wheel radius — useful when wheel wear has shifted from
nominal.

### `feedforwardCharacterization(Drive)`

Sweeps drive voltage from 0 to full while recording velocity. Fits
`kS`, `kV`, `kA` so you can update `DriveConstants`.

Outputs go to AdvantageKit; the post-processing step is a small
Python notebook the team keeps separately.

### SysId routines

Four `Command`s — quasistatic forward/reverse and dynamic
forward/reverse — that drive the WPILib SysId workflow. Run each in
sequence, then process the log with the SysId tool.

## How they get scheduled

The joystick command is set as the drive's *default* command in
`RobotState`:

```java
drive.setDefaultCommand(
  DriveCommands.joystickDrive(
    drive,
    () -> -controller.getLeftY(),
    () -> -controller.getLeftX(),
    () -> -controller.getRightX()));
```

Characterization and SysId routines are exposed via the
`SendableChooser` in `RobotState`'s auto-selector — they show up
alongside real autos so you can run them at home but never accidentally
pick one at a competition.

## Pitfalls

- **Robot drifts at neutral stick.** Deadband too small. Bump
  `DriveCommands#JOYSTICK_DEADBAND` (or the equivalent constant) until
  zero is firmly zero.
- **SysId looks terrible.** Make sure the floor isn't carpeted in a
  way that varies friction — sweeps need consistent traction.
- **Wheel radius char gives a strange number.** Check that the gyro
  yaw is wrapped correctly — if it folds at ±180 mid-spin, the math
  diverges.
