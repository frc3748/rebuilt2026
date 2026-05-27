---
layout: default
title: Drive
eyebrow: Subsystem
description: Four-module swerve drive with high-rate odometry, vision fusion, and PathPlanner integration.
permalink: /subsystems/drive/
---

The drive subsystem is the most complex and the most foundational.
Every other subsystem that needs a pose asks `Drive` (via `RobotState`)
for it.

| | |
| --- | --- |
| **Source** | `src/main/java/frc/robot/subsystems/drive/` |
| **Public class** | [`Drive`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/subsystems/drive/Drive.java) extends `StateMachine<Drive.State>` |
| **Children** | Four `Module` wrappers (FL, FR, BL, BR) |
| **Constants** | [`DriveConstants`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/subsystems/drive/DriveConstants.java) |

## States

```java
public enum State {
  UNDETERMINED,
  IDLE,
  TRAVERSING,             // open-loop, joystick-driven
  SLOW,                   // reduced max velocity for fine alignment
  TRAVERSING_AT_ANGLE     // joystick translation, locked heading
}
```

`UNDETERMINED` → `IDLE` happens automatically once odometry has at
least one valid sample.

## Geometry

| Parameter | Value |
| --- | --- |
| Track width | 0.71 m (28″) |
| Wheel base | 0.71 m (28″) |
| Wheel radius | 0.0508 m (2″) |
| Max translation speed | 5.27 m/s |
| Slow-mode max | 0.5 m/s |
| Drive reduction | 6.48 : 1 (L2 SDS) |
| Turn reduction | 12.1 : 1 |

Module translations are computed from track width and wheel base in
`DriveConstants` and passed to `SwerveDriveKinematics`.

## Module layout

The four modules are indexed 0–3:

| Index | Position | Drive CAN | Turn CAN | Encoder CAN |
| --- | --- | --- | --- | --- |
| 0 | Front-Left | 8 | 3 | 4 |
| 1 | Front-Right | 2 | 9 | 1 |
| 2 | Back-Left | 4 | 7 | 2 |
| 3 | Back-Right | 6 | 5 | 3 |

Drive motors are NEOs; turn motors are NEO 550s with absolute
CANcoders. The Pigeon 2 gyro lives on CAN ID 50.

## Hardware abstraction

Three IO interfaces:

- **`GyroIO`** / `GyroIOPigeon2` — yaw, pitch, roll, angular velocities.
- **`ModuleIO`** / `ModuleIOSpark` / `ModuleIOSim` — per-wheel I/O.
- **`DriveIO`** — chassis-level (used in some legacy paths).

The real impl runs a **`SparkOdometryThread`** at 250 Hz to capture
sub-loop wheel positions, giving more accurate dead-reckoning between
the 50 Hz `periodic()` ticks.

## Pose estimation

A WPILib `SwerveDrivePoseEstimator` fuses three inputs:

1. **Module odometry** — every loop, the 250 Hz samples are batched
   and replayed in time order.
2. **Vision** — `Drive#addVisionMeasurement(Pose2d, double timestamp,
   Matrix stdDevs)` is called by `VisionSubsystem` whenever a Megatag
   estimate arrives.
3. **Gyro** — the Pigeon 2 yaw provides absolute heading reference.

The result is read by everyone via:

```java
robotState.getLatestFieldToRobot();    // pose now
robotState.getPredictedFieldToRobot(); // pose extrapolated forward
```

## PathPlanner integration

`Drive` registers a `PPHolonomicDriveController` with PathPlanner. Auto
routines and dynamic paths from
[`DynamicPathGenerator`]({{ '/utilities/path-generation/' | relative_url }})
both feed through it.

Path-following tolerances and controller PID gains live in
`DriveConstants` and are tunable via
[`GetTuned`]({{ '/utilities/get-tuned/' | relative_url }}).

## Driver control

`DriveCommands.joystickDrive(Drive, …)` is the standard binding:

- Left stick → translation, scaled to max velocity.
- Right stick → rotation (or held heading in `TRAVERSING_AT_ANGLE`).
- A deadband + cubed response is applied for fine control.

When the driver releases the sticks, the modules **brake-park** (X
pattern) — implemented by holding `IDLE` and requesting an X module
state.

## Public API (selected)

```java
Pose2d getPose();
void   setPose(Pose2d pose);                            // teleport — usually only auto init
void   addVisionMeasurement(Pose2d, double t, Matrix);  // from VisionSubsystem
void   runSetpoint(ChassisSpeeds speeds);               // field-relative
void   runCharacterization(double volts);               // for SysId
Rotation2d getAimRotationForHub();                      // shortest-path heading toward hub
Field2d getField();                                     // for Shuffleboard
```

## Characterization

`DriveCommands` exposes:

- **Wheel-radius characterization** — spins the robot in place and
  computes the effective wheel radius from gyro vs. odometry.
- **Feedforward characterization** — sweeps voltage and records
  velocity to fit `kS`, `kV`, `kA`.
- **SysId routines** — quasistatic and dynamic, forward and reverse.

Run via dashboard chooser entries; outputs land in AdvantageKit logs
for offline analysis.

## Tuning knobs

Every PID gain and tolerance is wrapped in
[`GetTuned`]({{ '/utilities/get-tuned/' | relative_url }}). The most
common to touch:

- `Drive/Module/Drive/kP`, `kI`, `kD`
- `Drive/Module/Turn/kP`, `kI`, `kD`
- `Drive/HeadingController/kP`, `kI`, `kD`
- `Drive/MaxLinearSpeed`, `Drive/MaxAngularSpeed`

## Common pitfalls

- **`UNDETERMINED` forever.** Means odometry never delivered a valid
  sample. Usually a CANcoder offset issue — verify the offsets in
  `DriveConstants` against the values in the Spark client.
- **Wheels skip in autos.** Reduce `Drive/Module/Drive/kP` or lower
  the auto's max acceleration.
- **Vision fusion overpowers odometry.** Increase the vision standard
  deviation factors in
  [`VisionConstants`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/subsystems/vision/VisionConstants.java).
