---
layout: default
title: Logging & Telemetry
eyebrow: Architecture
description: AdvantageKit, tunables, Elastic — how data leaves the robot.
permalink: /architecture/logging/
---

Three channels carry data off the robot. They serve different
audiences.

| Channel | Audience | Purpose |
| --- | --- | --- |
| **AdvantageKit `Logger`** | Developers | Replayable, deterministic recording of everything. |
| **Tunables (NetworkTables)** | Developers, tuners | Values you can edit live, recorded in the log. |
| **Elastic notifications** | Drivers | Big, glanceable toasts during a match. |

## AdvantageKit

`Robot` configures `Logger` once at startup:

```java
Logger.recordMetadata("GIT_SHA", BuildInfo.GIT_SHA);
...
Logger.addDataReceiver(new WPILOGWriter());   // USB stick on the roboRIO, logs/ in sim
Logger.addDataReceiver(new NT4Publisher());   // streams to AdvantageScope
Logger.start();
```

The metadata is `ROBOT`, `MODE`, `ROBOT_TYPE`, and the build info:
`GIT_SHA`, `GIT_BRANCH`, `GIT_DIRTY` and `BUILD_DATE`. The
`generateBuildInfo` task in `build.gradle` writes those into
`build/generated/buildinfo/frc/robot/BuildInfo.java` before every
compile, so each log says which commit it came from and whether the
tree had uncommitted changes.

After that, every input every subsystem reads gets recorded. The two
methods you use day to day:

```java
io.updateInputs(inputs);                       // pull from hardware
Logger.processInputs("Drive/Module0", inputs); // record (live) or restore (replay)

Logger.recordOutput("Drive/AimTarget", new Pose2d(targetTrans, goal));
Logger.recordOutput("Shooter/Setpoint/Speed", setpoint.getShooterRPS());
```

Replay is the killer feature: open a `.wpilog` file in AdvantageScope,
or run `./gradlew simulateJava` against it, and the robot code
re-executes with identical inputs.

### What gets logged automatically

- All `@AutoLog` inputs from every IO. Mechanism motors log under
  `Motors/<name>`, including `TempCelsius`, and `Motor` adds its `Goal`
  and `Mode` as outputs.
- State-machine state, desired state, transitioning flag, flags.
- AdvantageKit's own `PowerDistribution/*`, `SystemStats/*` and
  `DriverStation/*` tables.
- Every tunable, under `NetworkInputs/Tunable/…`.

### What you log by hand

Outputs (anything *computed*): goals, setpoints, targets, error
values. Use the subsystem name as the prefix (`"Shooter/Setpoint/Speed"`).

Poses that only exist to draw the robot in AdvantageScope go through
`Visuals.record` instead, which only records in simulation. The full
key list and the reasoning are on
[Logging & robotTools]({{ '/architecture/logging-and-robottools/' | relative_url }}).

## Tunables

[`TunableNumber`]({{ '/utilities/tunable-number/' | relative_url }})
wraps AdvantageKit's `LoggedNetworkNumber`:

```java
stowSetpoint = new TunableNumber("Intake/Extension Stow Setpoint", constants.stowSetpoint);
...
case STOW -> goTo(stowSetpoint.get(), 0);
```

The value shows up under `/Tunable` in NetworkTables and is editable
live. Every value is recorded under `NetworkInputs/Tunable/…`, so replay
sees the same edits. It is not saved: every reboot starts from the
value in code.

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

- `Odometry/Robot` (a `Pose2d`) shows the robot on the field.
- In simulation only, Pose3d outputs show mechanisms (`Intake/Pose`,
  `Intake/ExtensionPose`, `Hopper/Pose`, `Hood/Pose`), each `Camera`
  logs `Vision/<name>/CameraPose` and `Vision/<name>/Tags`, and
  `ShotVisualizer` logs the predicted shot under `Shooter/Trajectory`
  while the shooter is firing.

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
3. Open the most recent `.wpilog` in AdvantageScope, or add the stick
   as a log folder in
   [robotTools]({{ '/architecture/logging-and-robottools/' | relative_url }}#opening-logs-in-robottools).
4. To **replay**, point a local sim at the log: it'll re-run the robot code with the same inputs and you can step through anything.

This is the single most valuable debugging tool in the codebase.
