---
layout: default
title: Drive
eyebrow: Subsystem
description: Four-module swerve drive with high-rate odometry, vision fusion, and PathPlanner integration, configured per robot by a DriveConfig.
permalink: /subsystems/drive/
---

The drive subsystem is the most complex and the most foundational.
Every other subsystem that needs a pose asks `RobotState`, which `Drive`
feeds. Every robot has one; its constants come from the robot's
[`DriveConfig`](#driveconfig).

| | |
| --- | --- |
| **Source** | `src/main/java/frc/robot/subsystems/drive/` |
| **Public class** | [`Drive`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/subsystems/drive/Drive.java) extends `StateMachine<Drive.State>` |
| **Children** | Four `Module` wrappers (FL, FR, BL, BR) |
| **Constants** | A [`DriveConfig`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/subsystems/drive/DriveConfig.java) subclass per robot: `CompDrive`, `SecondaryDrive` (extends `CompDrive`), `PracticeDrive` |

## States

```java
public enum State {
  UNDETERMINED,
  IDLE,                   // stopped
  CROSSED,                // wheels in an X
  TRAVERSING,             // joystick-driven
  TRAVERSING_AT_ANGLE,    // joystick translation, heading aimed at the target
  PATHFINDING,
  ALIGNING,
  SLOW                    // reduced max speed, heading aimed at the target
}
```

The drive determines itself straight into `TRAVERSING`.

## `DriveConfig`

`DriveConfig` is a plain class of public fields with defaults. A robot
subclasses it and overwrites fields in the constructor. `Drive`,
`ModuleIOSpark`, `ModuleIOSim`, the gyro IOs, `DriveCommands` and
`AutoAlignToPoseCommand` all read from `drive.getConfig()`.

| Group | Fields |
| --- | --- |
| Hardware | `gyro` (`PIGEON2` or `NAVX`), `pigeonCanId`, `navXPort`, `navXUpdateRateHz`, `driveController` (`SPARK_FLEX` or `SPARK_MAX`; `TALON_FX` is refused), `turnSensor` (`CANCODER` or `SPARK_ABSOLUTE_ENCODER`) |
| Modules | `frontLeft`, `frontRight`, `backLeft`, `backRight`: `ModuleConstants(driveCanId, turnCanId, canCoderId, zeroRotation, driveInverted)` |
| Geometry | `trackWidth`, `wheelBase`, `bumperHeight`, `wheelRadiusMeters`, `driveReduction`, `turnReduction` |
| Gains | `driveKp`…`driveKv`, `driveSparkKv`, `turnKp`…`turnKv`, and the `…Sim…` gains for `ModuleIOSim` |
| Limits | `maxSpeedMetersPerSec`, `slowSpeedMetersPerSec`, `maxLinearAcceleration`, current limits |
| PathPlanner | `robotMassKg`, `robotMOI`, `wheelCOF`, `pathTranslationPid`, `pathRotationPid`, `pathConstraints` |
| Heading | `aimP`, `aimD` (aiming in `TRAVERSING_AT_ANGLE`), `headingLockP`, `headingLockD`, `headingLockToleranceRadians`, `headingLockCaptureRadPerSec` |
| Auto-align | `driveToPointP`, `driveToPointHeadingP`, and four tolerances |

Helpers compute the rest: `moduleTranslations()`, `driveBaseRadius()`,
`maxAngularSpeed()`, `pathPlannerConfig()`, and so on.

## The drivetrains

`SecondaryDrive` extends `CompDrive`. It overrides the four
`ModuleConstants` with its calibrated zero rotations (comp's are `0`
until calibrated) and sets the same 6.48 : 1 `driveReduction` itself,
so the comp column covers both. See
[Multiple Robots]({{ '/architecture/robots/' | relative_url }}).

| | Comp and secondary (`CompDrive`) | Practice (`PracticeDrive`) |
| --- | --- | --- |
| Gyro | Pigeon 2, CAN 50 | NavX, USB1, 50 Hz |
| Drive motors | NEO on Spark Flex, 6.48 : 1 | NEO on Spark MAX, 7.31 : 1 |
| Turn motors | NEO 550 on Spark MAX, 12.1 : 1 | NEO on Spark MAX, 12.8 : 1 |
| Turn sensor | CANcoder | Spark absolute encoder |
| Track width × wheel base | 28″ × 28″ | 0.80 m × 0.80 m |
| Wheel radius | 0.0508 m (2″) | 0.0508 m (2″) |
| Max speed | 5.27 m/s | 3.5 m/s |
| Slow-mode max | 0.5 m/s | 0.5 m/s (default) |

Module CAN IDs for each are on the [CAN ID Map]({{ '/reference/can-ids/' | relative_url }}).

## Hardware abstraction

- **`GyroIO`** / `GyroIOPigeon2` / `GyroIONavX` — yaw, pitch, roll, rates, acceleration.
- **`ModuleIO`** / `ModuleIOSpark` / `ModuleIOSim` — per-wheel I/O.
- **`DriveIO`** — chassis-level logged inputs (module states, pose, aim goal).

`ModuleIOSpark` builds a Spark Flex or Spark MAX for the drive motor
from `driveController`, and throws if it is `TALON_FX` (Kraken swerve
needs its own `ModuleIO`). For turning, `CANCODER` writes `zeroRotation` as
the CANcoder's magnet offset and seeds the Spark's relative encoder
from it; `SPARK_ABSOLUTE_ENCODER` reads the Spark's absolute encoder and
subtracts `zeroRotation` in code. Drive and turn PID are tunable live as
`Drive PID/…` and `Turn PID/…` through
[`SparkUtil.tune`]({{ '/utilities/tunable-number/' | relative_url }}#motor-gains).

On the real robot a **`SparkOdometryThread`** samples wheel positions
and gyro yaw at `odometryFrequency` (100 Hz), faster than the 50 Hz loop.

## Pose estimation

A WPILib `SwerveDrivePoseEstimator` fuses:

1. **Module odometry** — every loop, the high-rate samples are replayed in time order.
2. **Gyro** — yaw from the gyro; if it disconnects, the drive falls back to kinematics and raises an alert.
3. **Vision** — `Drive#addVisionMeasurement(pose, timestamp, stdDevs)`, called through `RobotState#addVisionMeasurement` on the real robot.

Each loop the drive pushes its pose and motion into `RobotState`, and
everyone reads it from there:

```java
robotState.getLatestFieldToRobot().getValue();   // pose now
robotState.getFieldToRobot(timestamp);           // pose at a past time
```

## PathPlanner integration

`Drive` configures `AutoBuilder` with a `PPHolonomicDriveController`
using `pathTranslationPid` / `pathRotationPid` and
`config.pathPlannerConfig()`. Paths flip for the red alliance. Every
[auto]({{ '/commands/autos/' | relative_url }}) follows paths through it.

## Driver control

The default command is `DriveCommands.smartDrive(...)`:

- Left stick → field-relative translation, squared, up to `getMaxLinearSpeedMetersPerSec()`.
- Right stick X → rotation, squared.
- Right stick released → `HeadingLock` holds the heading. It waits until the robot's yaw rate drops under `headingLockCaptureRadPerSec`, saves the gyro heading, and PID-corrects any drift back to it (errors under `headingLockToleranceRadians` are ignored). It follows the raw gyro, so pose resets and vision corrections don't move it.
- In `TRAVERSING_AT_ANGLE` and `SLOW`, a profiled PID holds `getAimRotationForHub()` instead.

The shared bindings (slow mode, aim, heading reset) are listed under
[Controls]({{ '/architecture/robot-state/' | relative_url }}#controls).

## Public API (selected)

```java
Pose2d getPose();
void   setPose(Pose2d pose);                            // teleport, usually at auto start
void   addVisionMeasurement(Pose2d, double t, Matrix);  // from RobotState
void   runVelocity(ChassisSpeeds speeds);               // robot-relative
void   stopWithX();
void   runCharacterization(double output);              // for SysId
Rotation2d getAimRotationForHub();                      // heading that points the shooter at the target
DriveConfig getConfig();
```

## Characterization

- `DriveCommands.wheelRadiusCharacterization(drive)` and
  `DriveCommands.feedforwardCharacterization(drive)` — see
  [Drive Commands]({{ '/commands/drive-commands/' | relative_url }}).
- `drive.sysIdQuasistatic(direction)` / `drive.sysIdDynamic(direction)` —
  WPILib SysId routines.

## Common pitfalls

- **A module points the wrong way at zero.** Check that module's
  `zeroRotation` in the robot's `DriveConfig`.
- **Wheels skip in autos.** Lower `pathTranslationPid` or the path's
  max acceleration.
- **Vision fusion overpowers odometry.** Raise the camera's
  `stdDevFactor` or the baselines in
  [`VisionConstants`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/subsystems/vision/VisionConstants.java).
