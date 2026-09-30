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
| `util/motor/MotorIO.java` | The interface. `@AutoLog` inputs and setters (voltage, output, velocity, position, stop). |
| `util/motor/MotorIOSpark.java` | Real hardware. Configures the Spark from the `MotorConfig`, reads inputs with `ifOk`, wires `SparkUtil.tunePID`. |
| `util/motor/MotorIOSim.java` | Kinematic simulation. Velocity setpoints are reached with a short lag, position setpoints move at the MAXMotion cruise velocity. |
| `util/motor/Motor.java` | What a subsystem holds. Picks the IO for `Constants.kMode`, owns the logged inputs, remembers the last setpoint. |

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

And the subsystem uses them:

```java
public class Intake extends StateMachine<Intake.State> {
    private final Motor rollers = new Motor(IntakeConstants.kRollers);
    private final Motor extension = new Motor(IntakeConstants.kExtension);

    @Override
    protected void update() {
        rollers.update();
        extension.update();

        switch (getState()) {
            case STOW -> stow();
            case INTAKE -> intake();
            default -> stop();
        }
    }

    public void intake() {
        extension.setPosition(IntakeConstants.kIntakeSetpoint.get());
        rollers.setVelocity(IntakeConstants.kIntakeRollerSpeed.get());
    }
}
```

`Motor.update()` reads the IO and calls `Logger.processInputs` under the
motor's name, so every motor is logged and replayable.

## Cameras

| File | What it is |
| --- | --- |
| `CameraConfig.java` | Name, NetworkTables name, robot→camera transform, std-dev factor, vendor type. |
| `CameraIO.java` | `@AutoLog` inputs plus `setRobotOrientation`. |
| `CameraIOLimelight.java` | Limelight MegaTag 1 and 2. |
| `CameraIOPhoton.java` | PhotonVision multi-tag solve plus a heading-seeded solve. |
| `CameraIOPhotonSim.java` | `CameraIOPhoton` with a simulated camera attached. |
| `Camera.java` | Config + IO + inputs; filters the estimate and computes standard deviations. |
| `Vision.java` | Loops over its cameras and hands estimates to `RobotState`. |

Adding a camera is one line in `VisionConstants.kCameras`. Adding a
vendor is one `CameraIO` class and one line in `Camera.of`.

## How the mode is chosen

`Constants.kMode` is `REAL` on the roboRIO and `SIM` on a laptop; set
`kSimMode` to `REPLAY` to replay a log. `Motor` and `Camera.of` read it,
so no subsystem switches on the mode.

## Caveats

- **Inputs fields must be `public`** for the annotation processor.
- **The sim is kinematic.** It proves state-machine logic, not gains.
- **Limelight offsets live on the Limelight.** `CameraConfig.robotToCamera`
  is used for logging, simulation, and PhotonVision.
