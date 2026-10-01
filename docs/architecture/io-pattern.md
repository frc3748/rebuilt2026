---
layout: default
title: The IO Layer Pattern
eyebrow: Architecture
description: Hardware lives behind two small interfaces, MotorIO and CameraIO. Subsystems never see a vendor class.
permalink: /architecture/io-pattern/
---

The codebase follows the **AdvantageKit IO pattern**, but instead of an
IO interface, a real implementation, and a sim implementation for every
mechanism, there is one set for motors and one for cameras. Subsystems
compose those.

## Motors

| File | What it is |
| --- | --- |
| `util/motor/MotorConfig.java` | Fluent description of one motor: CAN ID, controller (`SPARK_MAX`, `SPARK_FLEX` or `TALON_FX`), followers, gains, MAXMotion, soft limits, tunables. |
| `util/motor/Gains.java` | PID, feedforward, gravity and MAXMotion values. `MotorConfig` holds one per closed-loop slot. |
| `util/motor/MotorIO.java` | The interface. `@AutoLog` inputs (position, velocity, volts, current, temperature, follower volts and current) and setters. |
| `util/motor/MotorIOSpark.java` | Real hardware for `SPARK_MAX` and `SPARK_FLEX`. Configures the Spark from the `MotorConfig`, reads inputs with `ifOk`, wires [`SparkUtil.tune`]({{ '/utilities/tunable-number/' | relative_url }}#motor-gains). |
| `util/motor/MotorIOTalonFX.java` | Real hardware for `TALON_FX` (a Kraken or Falcon) through Phoenix 6. See below. |
| `util/motor/MotorIOSim.java` | Kinematic simulation. Velocity goals are reached with a short lag, position goals move at the MAXMotion cruise velocity. |
| `util/motor/SpinMotor.java` | A velocity-controlled motor: `set(speed)`, `isAtGoal(tolerance)`. |
| `util/motor/PosMotor.java` | A position-controlled motor: `set(position)`, `set(position, ff, slot)`, `resetPosition(position)`. |
| `util/motor/Motor.java` | Shared base: picks the IO for `Constants.kMode` (and, on the real robot, the config's controller), logs inputs under `Motors/<name>`, buffers the command, sends it once per loop and logs its `Goal` and `Mode`. |

A constants class declares the motors as public fields:

```java
public MotorConfig extension = new MotorConfig("Intake Extension", 46, Controller.SPARK_MAX)
        .follower(47, true)
        .currentLimit(60)
        .conversion(360.0 / 23.0, 360.0 / 23.0 / 60.0)
        .pid(0.09, 0, 0)
        .feedforward(0.17, 0.00131, 0)
        .cosineGravity(0.24)
        .maxMotion(600, 130, 2);
```

And the subsystem builds them from the constants object it's given and
says what each state does:

```java
rollers = new SpinMotor(constants.rollers);
extension = new PosMotor(constants.extension);

@Override
protected void applyState(State state) {
    switch (state) {
        case STOW -> goTo(stowSetpoint.get(), 0);
        case INTAKE -> goTo(intakeSetpoint.get(), intakeRollerSpeed.get());
        ...
    }
}
```

Each robot can hand its subsystems different constants; see
[Per-robot constants]({{ '/architecture/robots/' | relative_url }}#per-robot-constants).

The state machine reads every registered motor before `applyState` and
writes every motor after `applyConstraints`, so each motor is logged and
sent exactly once per loop.

Any `MotorConfig` can run a Kraken by passing `Controller.TALON_FX`.
`MotorIOTalonFX` keeps the same units as `MotorIOSpark`:

- Positions are in the `conversion()` position units, applied as the
  Talon's `SensorToMechanismRatio`.
- Velocities are scaled to match the Spark convention (motor RPM times
  the velocity factor).
- `maxMotion(...)` becomes Motion Magic.
- Followers run with `MotorAlignmentValue.Aligned`, or `Opposed` when
  inverted.
- Every gain written in the `MotorConfig` is tunable live, like on a
  Spark (no `kDeviationErr`). See [Tuning]({{ '/utilities/tunable-number/' | relative_url }}).

## Cameras

Every camera, whatever the vendor, reports the same three things:

- **Pose observations.** A robot pose, its timestamp, tag count, average tag distance, ambiguity, and a `PoseSource` (MegaTag 1, MegaTag 2, multi-tag, single-tag, trig solve). The source decides how much the translation and heading are trusted.
- **Object observations.** Class, yaw, pitch, area, and confidence from a detection pipeline.
- **Tag IDs** seen this loop.

| File | What it is |
| --- | --- |
| `CameraConfig.java` | Name, NetworkTables name, vendor, robot→camera transform (fixed or a supplier for a moving camera), std-dev factor, pipelines, object height. |
| `CameraIO.java` | The interface and the observation records. |
| `CameraIOLimelight.java` / `CameraIOPhoton.java` / `CameraIOPhotonSim.java` | Vendor implementations. |
| `Camera.java` | Vendor-neutral processing: filters poses, weights them, and turns object observations into field positions. |
| `Vision.java` | The state machine that owns the cameras. |

Adding a camera is one `CameraConfig` returned from the robot's
`RobotDefinition.cameras()`. Adding a vendor is one `CameraIO` class and
one constant in `CameraConfig.Type`.

## Drive

The swerve drive keeps the AdvantageKit template's own interfaces:

| Interface | Implementations |
| --- | --- |
| `GyroIO` | `GyroIOPigeon2`, `GyroIONavX` |
| `ModuleIO` | `ModuleIOSpark` (Spark Flex or MAX drive; CANcoder or Spark absolute-encoder turn), `ModuleIOSim` |

`Drive` picks them from `Constants.kMode` and the robot's `DriveConfig`
(`gyro`, `driveController`, `turnSensor`), and passes the same config to
each one. `ModuleIOSpark` only drives Sparks and throws if
`driveController` is `TALON_FX`; Kraken swerve needs its own `ModuleIO`.
See [Drive]({{ '/subsystems/drive/' | relative_url }}).

## How the mode is chosen

`Constants.kMode` is `REAL` on the roboRIO and `SIM` on a laptop; set
`kSimMode` to `REPLAY` to replay a log. `Motor`, `Camera.of` and `Drive`
read it, so no mechanism switches on the mode.

## Caveats

- **Inputs fields must be `public`** for the annotation processor.
- **The sim is kinematic.** It proves state-machine logic, not gains.
- **Limelight offsets live on the Limelight.** `CameraConfig.robotToCamera`
  is used for logging, simulation, and PhotonVision.
