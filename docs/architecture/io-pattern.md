---
layout: default
title: The IO Layer Pattern
eyebrow: Architecture
description: Hardware lives behind two small interfaces, MotorIO and CameraIO. Subsystems never see a vendor class.
permalink: /architecture/io-pattern/
---

The codebase follows the **AdvantageKit IO pattern**, but instead of an
IO interface, a Spark implementation, and a sim implementation for every
mechanism, there is exactly one of each for motors and one of each for
cameras. Subsystems compose those.

## Motors

| File | What it is |
| --- | --- |
| `util/motor/MotorConfig.java` | Fluent description of one motor: CAN ID, Spark MAX or Flex, followers, gains, MAXMotion, soft limits, tunables. |
| `util/motor/MotorIO.java` | The interface. `@AutoLog` inputs and setters. |
| `util/motor/MotorIOSpark.java` | Real hardware. Configures the Spark from the `MotorConfig`, reads inputs with `ifOk`, wires `SparkUtil.tunePID`. |
| `util/motor/MotorIOSim.java` | Kinematic simulation. Velocity goals are reached with a short lag, position goals move at the MAXMotion cruise velocity. |
| `util/motor/SpinMotor.java` | A velocity-controlled motor: `set(speed)`, `isAtGoal(tolerance)`. |
| `util/motor/PosMotor.java` | A position-controlled motor: `set(position)`, `set(position, ff, slot)`, `resetPosition(position)`. |
| `util/motor/Motor.java` | Shared base: picks the IO for `Constants.kMode`, logs inputs, buffers the command and sends it once per loop. |

A constants file declares the motors:

```java
public static final MotorConfig kExtension = new MotorConfig("Intake Extension", 46, Controller.SPARK_MAX)
        .follower(47, true)
        .currentLimit(60)
        .conversion(360.0 / 23.0, 360.0 / 23.0 / 60.0)
        .pid(0.09, 0, 0)
        .feedforward(0.17, 0.00131, 0)
        .cosineGravity(0.24)
        .maxMotion(600, 130, 2)
        .tunable(true, true);
```

And the subsystem holds them and says what each state does:

```java
private final SpinMotor rollers = new SpinMotor(kRollers);
private final PosMotor extension = new PosMotor(kExtension);

@Override
protected void applyState(State state) {
    switch (state) {
        case STOW -> goTo(kStowSetpoint.get(), 0);
        case INTAKE -> goTo(kIntakeSetpoint.get(), kIntakeRollerSpeed.get());
        ...
    }
}
```

The state machine reads every registered motor before `applyState` and
writes every motor after `applyConstraints`, so each motor is logged and
sent exactly once per loop.

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

Adding a camera is one `CameraConfig` in `VisionConstants.kCameras`.
Adding a vendor is one `CameraIO` class and one constant in `CameraConfig.Type`.

## How the mode is chosen

`Constants.kMode` is `REAL` on the roboRIO and `SIM` on a laptop; set
`kSimMode` to `REPLAY` to replay a log. `Motor` and `Camera.of` read it,
so no subsystem switches on the mode.

## Caveats

- **Inputs fields must be `public`** for the annotation processor.
- **The sim is kinematic.** It proves state-machine logic, not gains.
- **Limelight offsets live on the Limelight.** `CameraConfig.robotToCamera`
  is used for logging, simulation, and PhotonVision.
