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
| Gains | `driveKp`…`driveKv`, `driveKa`, `driveSparkKv`, `turnKp`…`turnKv`, and `driveSimP/D`, `turnSimP/D` for `ModuleIOSim` |
| Auto limits | `autoSpeedFraction` (0.85 of `maxSpeedMetersPerSec`), `autoMaxAcceleration` (3.5 m/s²) and `autoTurnFraction` (0.5 of `maxAngularSpeed()`); every path is capped to them when it loads |
| Teleop feel | `teleopAcceleration` (6 m/s²), `teleopTurnAcceleration` (20 rad/s²), `useSetpointGenerator` (off) |
| Steering | `steerFeedforward` (1.0 = the turn motor's own kV, from `turnGearbox` and `turnReduction`) |
| Limits | `maxSpeedMetersPerSec`, `slowSpeedMetersPerSec`, `maxLinearAcceleration`, current limits |
| PathPlanner | `robotMassKg`, `robotMOI`, `wheelCOF`, `pathTranslationPid`, `pathRotationPid`, `pathConstraints` |
| Heading | `aimP`, `aimD` (aiming in `TRAVERSING_AT_ANGLE`), `headingLockP`, `headingLockD`, `headingLockToleranceRadians`, `headingLockCaptureRadPerSec` |
| Auto-align | `driveToPointP`, `driveToPointHeadingP`, `metersTolerance` (0.04 m), `radiansTolerance` (2°) |

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

- **`GyroIO`** / `GyroIOPigeon2` / `GyroIONavX` / `GyroIOSim` — yaw, pitch, roll, their rates (`yawRateRadPerSec` and so on), acceleration.
- **`ModuleIO`** / `ModuleIOSpark` / `ModuleIOSim` — per-wheel I/O, including drive and turn motor temperatures.

The drive velocity target is in **m/s** with an acceleration in m/s². Each module's voltage feedforward is
`driveKs·sign(v) + driveKv·v + driveKa·a` (volts per m/s and per m/s²). The Spark's velocity loop
runs on wheel rad/s (the target divided by the wheel radius), so `driveKp` is per rad/s of error.
`driveSparkKv` stays in volts per m/s and is converted for the Spark. During autos the acceleration
comes from PathPlanner's per-module feedforwards, and in teleop it's 0.

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

### Simulation

In simulation the drivetrain is a [maple-sim](https://github.com/Shenzhen-Robotics-Alliance/maple-sim)
`SwerveDriveSimulation` built from the robot's own `DriveConfig` by `DriveSimulation`:
- **Physics:** mass, gearing, wheel radius, motors, current limits, and `wheelCOF` for tire grip. So wheels can slip in the sim the way they do on carpet.
- **Sensors:** `ModuleIOSim` drives the simulated modules. `GyroIOSim` reads the simulated gyro and works out acceleration from the robot's true motion. The sim feedforward comes from the motor model.
- **True pose:** the true pose (`getSimulation()…getSimulatedDriveTrainPose()`) feeds the vision and fuel sims, so odometry can drift from it like it does on a real field.
- **Pose resets:** `setPose` also moves the simulated robot.
- **The arena:** `RobotState#updateSimulation` steps the arena (`SimulatedArena`, the 2026 field by default; tests use an empty one).

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

## Slip

`SlipCorrector` checks every odometry sample with sensors the real robot has: wheel encoders, gyro yaw, and the gyro's accelerometer. Its thresholds are tunable as `Slip/…`.
- **One wheel:** once the gyro's rotation is taken out, a rigid robot's wheels all move the same way. A wheel that clearly stands out from the other three (above `Slip/Module Threshold` and at least twice as far off as the next-worst wheel) is slipping. Its distance is replaced by what the other three agree on.
- **All wheels:** if the wheels speed up or slow down more than `Slip/Robot Threshold` faster than the accelerometer says the robot does, they're spinning or skidding together. Odometry then moves at the speed the accelerometer supports. The accelerometer is ignored when the robot is tilted past `Slip/Max Tilt` (bumps) or the hit is over 1.5 g.

It logs `Drive/Slip/Modules`, `Robot`, `Scale`, `EstimatedSpeed` and `Events`. `SlipCorrectorTest` covers the logic.

What it can't see: wheels that all scrub sideways together. That happens when the robot drives and turns at the same time. Steering feedforward and the auto turn cap keep it small: over every diagnostic auto with no vision, PRACTICE now ends within about 3–5 cm of the simulated robot (it was 10–25 cm before them). With cameras, `visionKeepsTheRobotWhereItReallyIs` checks the pose stays within 5 cm after every diagnostic; it stays within about 3 cm.

## Collisions

`CollisionDetector` watches the gyro's accelerometer for a hit:

- **A jolt the wheels don't explain:** more than `Collision/Threshold Gs` (1 g) beyond what the wheels' own speed change shows. This catches being shoved sideways.
- **Any hit over `Collision/Hard Hit Gs`** (2 g), since the robot can't push itself harder than its grip (about 1.2–1.4 g). This catches head-on hits, where the wheels stop with the robot.

After a hit, and while the robot is tilted past `Collision/Max Tilt` (bumps), vision standard deviations are multiplied by `Collision/Vision Std Dev Scale` (0.25) for `Collision/Vision Seconds` (1 s). The pose then snaps back to what the cameras see. A bounce within that second counts as the same hit. It logs `Drive/Collision/Jolt`, `Hit`, `Tilted`, `TrustingVision` and `Events`. `CollisionDetectorTest` covers the logic, and `hittingTheWallIsACollision` rams a simulated wall.

## PathPlanner integration

`Drive` configures `AutoBuilder` with a `PPHolonomicDriveController`
using `pathTranslationPid` / `pathRotationPid` and
`config.pathPlannerConfig()`, and passes PathPlanner's per-module
acceleration feedforwards to the modules. Paths flip for the red alliance.
`PathAuto` caps every path's speed, acceleration and turn rate to the
robot's auto limits (`DriveConfig#limitForAuto`), so a slower robot
doesn't fall behind paths drawn for a faster one, and turning while
driving scrubs less. Every
[auto]({{ '/commands/autos/' | relative_url }}) follows paths through it.

## Driver control

The default command is `DriveCommands.smartDrive(...)`:

- Left stick → field-relative translation, squared, up to `getMaxLinearSpeedMetersPerSec()`.
- Right stick X → rotation, squared.
- Right stick released → `HeadingLock` holds the heading. It waits until the robot's yaw rate drops under `headingLockCaptureRadPerSec`, saves the gyro heading, and PID-corrects any drift back to it (errors under `headingLockToleranceRadians` are ignored). It follows the raw gyro, so pose resets and vision corrections don't move it.
- In `TRAVERSING_AT_ANGLE` and `SLOW`, a profiled PID holds `getAimRotationForHub()` instead.

Before the speeds reach the modules:

- **`DriveSlew`** limits how fast the driver can *speed up*: `Drive/Teleop Acceleration` for driving and `Drive/Teleop Turn Acceleration` for the right stick. Slowing down and changing direction happen right away, so the robot doesn't slide around corners. It only scales the speed, so the direction always follows the stick. Aiming and heading lock aren't slewed, since that would make them lag.
- **The swerve setpoint generator** (PathPlanner's, from 254) is off by default (`useSetpointGenerator`). It turns every `runVelocity(ChassisSpeeds)` into module states the robot can reach, limiting each module's acceleration to its motor, current limit and grip, and turning modules no faster than the turn motor can. Driving the simulator with it on, direction changes lagged: stick circles were 43° behind instead of 6°. At the current limits the wheels can't slip anyway (the slip-current test finds about 79 A on COMP against a 45 A limit), so it isn't worth the lag. Turn it on for a robot whose current limit is near its slip current. Path following never uses it, and `stop()` is always immediate.
- **Steering feedforward:** each module works out how fast its target angle is moving and adds `steerFeedforward × steerKv()` volts per rad/s on top of the turn PID, so modules turn with the motion instead of lagging behind it. A jump faster than the turn motor can follow, like a 180° flip, gets none. Tunable as `Turn PID/Steer FF`.

The shared bindings (slow mode, aim, heading reset) are listed under
[Controls]({{ '/architecture/robot-state/' | relative_url }}#controls).

## Logging

- `Drive/Gyro` and `Drive/Module0`…`Module3` — the gyro and module inputs.
- `SwerveStates/Setpoints` (empty while disabled) and `SwerveStates/Measured`; `SwerveChassisSpeeds/Setpoints` (after the setpoint generator), `SwerveChassisSpeeds/Requested` (before it) and `SwerveChassisSpeeds/Measured`.
- `Odometry/Robot` — the estimated pose. `Odometry/TrajectorySetpoint` — PathPlanner's target pose. The active path, `Odometry/Trajectory`, is logged by `DashboardManager`.
- `Drive/AimTarget` — the point `getAimRotationForHub()` aims at, with the goal heading.

Each loop, outside simulation, the drive also hands `RobotState` its
gyro rates and its measured, desired and fused chassis speeds. The
desired speeds are the discretized speeds from the last `runVelocity`
call.

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

- The **Measure:** autos (wheel radius, drive feedforward, slip current) measure the real robot and put the results on the Tune tab — see [Autos]({{ '/commands/autos/' | relative_url }}#measuring-autos). The wheel radius is tunable as `Drive/Wheel Radius` and applies after a restart.
- `DriveCommands.wheelRadiusCharacterization(drive)` and
  `DriveCommands.feedforwardCharacterization(drive)` — see
  [Drive Commands]({{ '/commands/drive-commands/' | relative_url }}).
- `drive.sysIdQuasistatic(direction)` / `drive.sysIdDynamic(direction)` —
  WPILib SysId routines.

## Common pitfalls

- **A module points the wrong way at zero.** Check that module's
  `zeroRotation` in the robot's `DriveConfig`.
- **Wheels skip in autos.** Lower `autoMaxAcceleration` or
  `driveCurrentLimit` for that robot, then rerun the diagnostic autos.
- **Vision fusion overpowers odometry.** Raise the camera's
  `stdDevFactor` or the baselines in
  [`VisionConstants`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/subsystems/vision/VisionConstants.java).
