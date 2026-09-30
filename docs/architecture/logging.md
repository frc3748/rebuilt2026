---
layout: default
title: Logging & Telemetry
eyebrow: Architecture
description: AdvantageKit, DogLog, Elastic — how data leaves the robot.
permalink: /architecture/logging/
---

Three channels carry data off the robot. They serve different
audiences.

| Channel | Audience | Purpose |
| --- | --- | --- |
| **AdvantageKit `Logger`** | Developers | Replayable, deterministic recording of everything. |
| **DogLog / NetworkTables** | Developers, tuners | Live numbers and tunable values during a match. |
| **Elastic notifications** | Drivers | Big, glanceable toasts during a match. |

## AdvantageKit

`Robot` configures `Logger` once at startup:

```java
Logger.addDataReceiver(new WPILOGWriter());   // writes to USB on real robot
Logger.addDataReceiver(new NT4Publisher());   // streams to AdvantageScope
Logger.start();
```

After that, every input every subsystem reads gets recorded. The two
methods you use day to day:

```java
io.updateInputs(inputs);                       // pull from hardware
Logger.processInputs("Drive/Module0", inputs); // record (live) or restore (replay)

Logger.recordOutput("Drive/Pose",     getPose());
Logger.recordOutput("Drive/Setpoint", lastSetpoint);
```

Replay is the killer feature: open a `.wpilog` file in AdvantageScope,
or run `./gradlew simulateJava` against it, and the robot code
re-executes with identical inputs.

### What gets logged automatically

- All `@AutoLog` inputs from every IO.
- State-machine state, desired state, transitioning flag.
- Subsystem flags.

### What you log by hand

Outputs (anything *computed*): poses, setpoints, error values,
3D mechanism poses for AdvantageScope visualization. Convention: use
the subsystem name as the prefix (`"Shooter/DistanceToTarget"`).

## DogLog

[DogLog](https://doglog.dev) provides the dashboard-tunable values.
The codebase uses it through
[`TunableNumber`]({{ '/utilities/tunable-number/' | relative_url }}):

```java
public static final TunableNumber kStowSetpoint = new TunableNumber("Intake/Extension Stow Setpoint", -93);
...
extension.set(kStowSetpoint.get());
```

The value shows up under `/Tunable` in NetworkTables and is editable
live. It is not saved: every reboot starts from the value in code.

Use it for **anything you'd want to tune at a competition without a
rebuild** — PID gains, tolerances, fixed-pose targets, flywheel speeds.

## Elastic notifications

[`Elastic`]({{ '/utilities/elastic/' | relative_url }}) wraps the
[Elastic dashboard's](https://frc-elastic.gitbook.io/docs) toast
notification API:

```java
Elastic.sendNotification(new Notification()
    .withTitle("Path Error")
    .withDescription("Unable to set pose")
    .withLevel(NotificationLevel.ERROR));
```

Use it sparingly — every toast competes for the driver's attention.
Good candidates:

- Successful homing of a mechanism
- Vision dropout / restoration
- Alliance-color mismatch
- An auto routine that couldn't load

Bad candidates: anything that fires every loop.

## Visualization conventions

For AdvantageScope's 3D field view:

- **`Odometry/Robot`** (a `Pose2d`) shows the robot on the field.
- **Pose3d outputs** show mechanisms (`Intake/Pose`, `Hopper/Pose`, `Hood/Pose`).
- In simulation, `ShotVisualizer` logs the predicted shot under `Shooter/Trajectory`. Each
  `Camera` logs `Vision/<name>/CameraPose` so you can verify its transform.

For SmartDashboard / Shuffleboard:

- Numbers, booleans, and the drive's `Field2d` (`Drive.fieldPose`)
  are published.

> **Don't double-publish.** If a value already goes to AdvantageKit
> (via `recordOutput`), don't separately `SmartDashboard.putNumber` it
> — the `NT4Publisher` data receiver already exposes everything under
> the `AdvantageKit/RealOutputs` table.

## Reading logs after the fact

USB-stick workflow:

1. Plug a USB stick into the roboRIO. `WPILOGWriter` writes to it automatically.
2. After the match, pull the stick.
3. Open the most recent `.wpilog` in AdvantageScope.
4. To **replay**, point a local sim at the log: it'll re-run the robot code with the same inputs and you can step through anything.

This is the single most valuable debugging tool in the codebase.
