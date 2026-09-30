---
layout: default
title: Tuning with TunableNumber
eyebrow: Utilities
description: Dashboard-tunable constants and live motor gains, both built on TunableNumber.
permalink: /utilities/tunable-number/
---

Anything you'd want to tune at a competition without redeploying goes
through a `TunableNumber`. You either read it where the value is used,
or give it a callback. Motor gains are wired up for you by
`SparkUtil.tune` and `MotorIOTalonFX`.

> **Nothing is saved.** A tuned value applies live and is gone on
> reboot. When you find a good number, copy it into the code.

## `TunableNumber`

[`TunableNumber`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/TunableNumber.java)
wraps AdvantageKit's `LoggedNetworkNumber` at `/Tunable/<key>`:

```java
public TunableNumber(String key, double defaultValue);
public double get();
public TunableNumber onChange(DoubleConsumer listener);
public static void pollAll();
```

Mechanism defaults live in the robot's constants object as plain
fields. The subsystem builds the `TunableNumber` from that default in
its constructor, and calls `get()` where the value is used:

```java
// IntakeConstants
public double stowSetpoint = -93;

// IntakeComp constructor
stowSetpoint = new TunableNumber("Intake/Extension Stow Setpoint", constants.stowSetpoint);

// IntakeComp.applyState
case STOW -> goTo(stowSetpoint.get(), 0);
```

The key is fixed in the subsystem, so it's the same on every robot;
only the default can differ. See
[Per-robot constants]({{ '/architecture/robots/' | relative_url }}#per-robot-constants).
Code that isn't per robot, such as `ActionCommands`' `FixedPos/…`
values, declares a `static final TunableNumber` instead.

Calling `get()` every loop is what picks up dashboard edits; don't copy
the value into a plain `double` at construction.

For code that needs a callback instead of polling, use `onChange`:

```java
new TunableNumber("Drive/Heading Lock kP", config.headingLockP).onChange(controller::setP);
```

`Robot.robotPeriodic()` calls `TunableNumber.pollAll()` before the
scheduler runs, so listeners fire on the main loop, once per change.
`SparkUtil.tune`, `MotorIOTalonFX`, `HeadingLock`, the `Auto Turn` gain
in `DriveCommands` and `ShooterComp`'s `TOF Tuning/…` values all work
this way.

`AutoAlignToPoseCommand` creates its `Auto Align/…` tunables once, in
a static `Tuning` shared by every instance, and reads them in
`initialize()`, so an edit applies to the next align.

## Motor gains

[`SparkUtil.tune`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/SparkUtil.java)
is the one helper for live Spark gains:

```java
public static void tune(String key, SparkBase spark, SparkBaseConfig config,
                        Gains gains, boolean feedforward, boolean maxMotion);
```

It publishes `<key>/kP`, `kI` and `kD`; with `feedforward`, also `kS`,
`kV`, `kA` and `kG` (or `kCos` for cosine gravity); with `maxMotion`,
also `kMaxAccel`, `kCruiseVel` and `kDeviationErr`. Each edit updates
the config and reconfigures the Spark without resetting or persisting
parameters.

You rarely call it directly. A `MotorConfig` opts in with
`.tunable(feedforward, maxMotion)`, and `MotorIOSpark` calls
`SparkUtil.tune(config.name(), …)` with that config's
[`Gains`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/motor/Gains.java):

```java
public MotorConfig extension = new MotorConfig("Intake Extension", 46, Controller.SPARK_MAX)
        ...
        .tunable(true, true);   // Intake Extension/kP … kDeviationErr
```

`ModuleIOSpark` does the same for the swerve modules as `Drive PID/…`
and `Turn PID/…`.

A `TALON_FX` motor gets the same `<name>/…` keys from `MotorIOTalonFX`,
except `kDeviationErr`. Each edit is applied to the Talon's slot 0 or
Motion Magic config.

## Key naming

A `/`-separated path, starting with the subsystem or feature:

- `Intake/Extension Stow Setpoint`
- `Shooter/Gravity Funnel InchesPerSec2`
- `FixedPos/RPS`
- `TOF Tuning/<distance>`

Keep names stable; the key is how you find the value on the dashboard.

## Where values show up

Tunables live under `/Tunable` in NetworkTables, so any NT client
(Elastic, AdvantageScope, OutlineViewer) can edit them. Each value is
also recorded in the log under `NetworkInputs/Tunable/…`, so replay
uses the same numbers and robotTools can list every edit made mid-run.
While the FMS is attached, `get()` returns the default and edits are
ignored.

## What to tune (and what not to)

**Yes:** PID gains, tolerances, setpoints, shot and time-of-flight
values.

**No:** CAN IDs, gear ratios, geometry — they only change with the
hardware. And nothing safety-related: the robot must be safe at the
default value.
