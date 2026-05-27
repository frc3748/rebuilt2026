---
layout: default
title: The IO Layer Pattern
eyebrow: Architecture
description: Every subsystem talks to hardware through an interface. Here is why, and how it is structured.
permalink: /architecture/io-pattern/
---

The codebase follows the **AdvantageKit IO pattern**: every subsystem
sees hardware through an *interface*, not a concrete implementation.

## The three layers

For any mechanism — call it `Foo` — you will see three files:

| File | What it is |
| --- | --- |
| `FooIO.java` | An **interface** declaring `updateInputs()` and any setters. Contains an `@AutoLog`-annotated `Inputs` inner class. |
| `FooIOSpark.java` | A **real-hardware implementation** that talks to actual motors and sensors. |
| `FooIOSim.java` | A **simulation implementation** that runs physics in software. |

The subsystem class (`Foo.java`) holds a reference of type `FooIO` — it
neither knows nor cares which concrete class is on the other side.

## A concrete example: `FlywheelIO`

```java
public interface FlywheelIO {
  @AutoLog
  class FlywheelIOInputs {
    public double velocityRPS = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}
  default void setVelocity(double rps, double ff) {}
  default void setVoltage(double volts) {}
  default void stop() {}
}
```

Three things to notice:

1. **`@AutoLog` on the inner class.** The annotation processor
   generates `FlywheelIOInputsAutoLogged`, which `Logger.processInputs`
   knows how to read/write/replay. You never write that class by hand.
2. **Every method is `default`-empty.** The stub implementation —
   `new FlywheelIO() {}` — is a valid no-op, useful in replay mode
   where inputs come from the log file.
3. **No leak of vendor types.** Nothing about `SparkMax`, `CANcoder`,
   `PhotonCamera`, or `LimelightHelpers` appears here. Replacing the
   motor controller doesn't change the interface.

## How a subsystem uses its IO

The pattern inside the subsystem is always the same:

```java
public class Flywheel extends StateMachine<State> {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public Flywheel(FlywheelIO io) {
    super(State.UNDETERMINED, …);
    this.io = io;
    registerStateCommands();
  }

  @Override
  public void update() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
    super.update();  // run pending transitions, state commands
  }

  public boolean isReady() {
    return Math.abs(inputs.velocityRPS - setpointRPS) < tolerance;
  }
}
```

Two periodic calls do the heavy lifting:

- **`io.updateInputs(inputs)`** — read sensors into the inputs object.
- **`Logger.processInputs("Flywheel", inputs)`** — in **live** mode this
  records the inputs to the log; in **replay** mode it overwrites
  `inputs` with the values from the log. Either way, downstream code
  reads the same fields.

## How `RobotState` chooses implementations

In `RobotState`'s constructor:

```java
shooter = new Shooter(
  switch (robotState) {
    case 1 -> new TurretIOSpark();
    case 2 -> new TurretIOSim();
    default -> new TurretIO() {};
  },
  switch (robotState) {
    case 1 -> new HoodIOSpark();
    case 2 -> new HoodIOSim();
    default -> new HoodIO() {};
  },
  switch (robotState) {
    case 1 -> new FlywheelIOSpark();
    case 2 -> new FlywheelIOSim();
    default -> new FlywheelIO() {};
  }
);
```

- **`robotState == 1`** — real robot. Use the Spark/Pigeon/Limelight implementations.
- **`robotState == 2`** — simulator. Use the physics-based sim implementations.
- **anything else** — replay. Use the empty default; `Logger.processInputs` will populate from the log.

## Why this pattern is worth the file count

It costs three files per subsystem instead of one. In return:

- **Replay-driven debugging.** Hit a weird bug at competition? Pull the
  WPILog off the USB drive, replay it locally, set breakpoints anywhere
  in the state-machine logic. The IO returns exactly the values it
  returned on the field.
- **Sim parity.** Adding a new mechanism means writing a sim
  implementation too. That sounds like work, but it forces you to
  understand the physics — and now everyone can develop without a
  robot on the bench.
- **Vendor isolation.** When CTRE renames an API or REV ships a
  breaking driver update, the blast radius is one file.
- **Unit testability.** You can construct a subsystem with a mock IO
  and a JUnit test — no WPILib HAL required.

## When to write each implementation

| Mode | When you need to update which IO |
| --- | --- |
| Adding a sensor | Add a field to the `Inputs` class. Populate it in Spark + Sim. The default is `0.0`, which the replay default also returns. |
| Adding a setter | Add to the interface as a `default` empty method. Override in Spark; usually no-op in Sim unless you need it for physics. |
| New mechanism | Three files: interface, Spark impl, Sim impl. Then a `Constants` file. Then the subsystem class. |

## Caveats

- **Default-empty stub means silent failure.** If you forget to override
  a setter in `FlywheelIOSpark`, the motor will just sit there. There's
  no compile-time check. Convention is to wire every interface method
  in the real impl even if it's a TODO.
- **AutoLog doesn't see private fields.** Inputs fields must be
  `public` for the annotation processor to find them.
- **`@AutoLog` only works for primitives + a few WPILib types**
  (`Pose2d`, `Translation2d`, `Rotation2d`, arrays of those, …). For
  custom types you have to log manually with `Logger.recordOutput`.
