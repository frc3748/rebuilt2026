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
| `util/motor/MotorIOSim.java` | Physics simulation. Each motor is a DC-motor plant (kV from the motor's free speed and conversion, kA from `simVelocityLag`, kS and gravity from the config) driven by a copy of the Spark's onboard PID, feedforward and MAXMotion at 1 kHz, with the current limit. Auto-tune and the gains behave in the simulator the way they do on the robot. |
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
        .cosineGravity(-0.24, 360.0)
        .maxMotion(600, 130, 10)
        .quadratureFilter(2, 10);
```

Every motor has a current limit. `currentLimit` (and the drive's
`driveCurrentLimit` / `turnCurrentLimit`) defaults to 40 A, and anything
under 1 A also falls back to 40 A (`MotorConfig.safeCurrentLimit`), so a
missing or zero limit can't leave a motor unprotected.

`cosineGravity(kCos, unitsPerRotation)` takes how many position units make
one mechanism rotation (360 for degrees), so the position turns into an
angle for the cosine term. The sign matters: kCos pushes the same way as
positive output, so an arm whose positive direction falls with gravity
(the intake, 0° deployed and level) needs a negative kCos. Auto-tune
measures it with the right sign. On a Spark the roboRIO adds
`kCos × cos(angle)` to each position command's feedforward from the last
measured position, instead of using the Spark's own kCos: the Spark
rejects a negative kCos ("Invalid parameter value"), which left the
intake pivot with no settings and no output at all.

`maxMotion(maxAccel, cruiseVel, allowedError)`: the Spark regenerates the
profile from wherever the mechanism is once it falls more than
`allowedError` behind. Keep it big enough that P can build real effort on
a loaded arm; at 2° the intake could only ask for about 2 V while lifting.

`uvwFilter(periodMs, depth)` and `quadratureFilter(periodMs, depth)` set
how the Spark averages velocity. A NEO on a SPARK MAX uses UVW (its hall
sensor). A Vortex on a SPARK Flex uses quadrature: REV only applies the
UVW settings to a Flex with a Flex Dock. Without the right one the Spark's
defaults apply, and on a Flex that's a 100 ms window averaged over 64
samples, so the speed reading lags far behind the motor. The intake
roller had `uvwFilter` and its auto-tune couldn't fit the lagging data;
every Flex now uses `quadratureFilter(2, 10)`, and `MotorAutoTuneTest`
fails if one doesn't.

If REVLib can't create a mechanism's Spark at all (it throws "Error (N)
creating SPARK #ID" when the device answers with an error at startup, or
REV's message when the Spark rejects a setting at boot), that
motor runs as a no-op and a "<name> motor didn't start" error with REV's
message shows under Devices, instead of the whole robot program failing to
start. Drive modules still fail loudly, since the robot can't drive on three.

The Spark IO retries its boot configuration and encoder zero five times.
If one still fails, a "<name> motor didn't take its settings" error shows
under Devices, because that Spark may be running without its soft limits,
zero or follower setting.

A position-controlled motor that pushes into something for a second stops
pushing: still more than 1% of its travel from the goal, moving less than
2% of its travel per second, and drawing at least 80% of its current
limit. It stays off, resting where it is, until it gets a goal more than
1% of its travel away, and shows "<name> was pushing into a hard stop, so
it stopped. Check its zero". It's logged as `Motors/<name>/Stalled`. The
usual cause is a zero that's off, so the setpoint sits past the hard stop.
It only applies to motors with a travel range (soft limits or
`tuneRange`).

Once a second the IO also checks its Sparks. If one that missed its
settings at boot shows up, or one reboots (REV's sticky "has reset"
warning, from a power blip or a loose wire), it sends the settings again
and puts the encoder back at the starting position. Without that the hood
came back reading 0° at its bottom stop and drove 25° past its top. The
settings error clears once they go through, and "<name> motor rebooted,
check its power and CAN wires" stays up for the rest of the session.
Changes while running (tuning, current limits) use `configureAsync`, so a
bad tuned value can't stall the loop or stop the program.

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
