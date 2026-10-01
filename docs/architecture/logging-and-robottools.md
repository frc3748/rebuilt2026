---
layout: default
title: Logging & robotTools
eyebrow: Architecture
description: What the robot logs and why, the battery workflow, and how to read logs in robotTools.
permalink: /architecture/logging-and-robottools/
---

robotTools is the team's log app, kept in its own `robotTools` repo. It
runs locally, reads a folder of AdvantageKit `.wpilog` files, and
answers questions AdvantageScope can't because it knows our keys:
energy per match phase and subsystem, battery resistance over time,
goal vs actual for every motor, path and auto-align tracking, camera
uptime, state timelines, and which tunables changed mid-run. Its
README covers setup and every analysis.

## Real robot vs simulation

Measurements and targets are logged on the real robot. Visuals are
logged only in simulation.

Everything needed to judge a match is logged on every robot: inputs,
motor goals, setpoints, targets, states and tunables. Values that only
exist to draw the robot in AdvantageScope go through
[`Visuals`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/Visuals.java),
whose `record(key, value)` only writes when `Constants.kMode` is `SIM`:

- `Intake/Pose`, `Intake/ExtensionPose`, `Hood/Pose`, `Hopper/Pose`
- `Vision/<camera>/CameraPose` and `Vision/<camera>/Tags`

On the roboRIO they would only cost loop time and log space. Guard any work that only builds a visual with `Visuals.enabled()`, as
`Camera` does for the tag poses. `Shooter/Trajectory` comes from
`ShotVisualizer`, which only runs in simulation and only logs while the
shooter is firing.

## What the robot logs

Outputs sit under `RealOutputs/`, inputs at the root, and metadata
under `RealMetadata/`. robotTools' analyses look for these keys:

| What | Keys | Used for |
| --- | --- | --- |
| Robot, code version | Metadata `ROBOT_TYPE`, `GIT_SHA`, `GIT_BRANCH`, `GIT_DIRTY`, `BUILD_DATE` | Which robot and which commit a log came from. |
| Battery | `Battery/Id`, `Battery/BootVoltage` | Per-battery history. |
| Power | `PowerDistribution/*`, `SystemStats/BatteryVoltage`, `SystemStats/BrownedOut` (logged by AdvantageKit) | Energy, voltage sag, brownouts. |
| Mechanism motors | `Motors/<name>/…` inputs (`Position`, `Velocity`, `AppliedVolts`, `CurrentAmps`, `TempCelsius`), `Motors/<name>/Goal` and `Mode` outputs | Goal vs actual, settle time, current and temperature. |
| Swerve | `Drive/Module<n>/…` inputs, `SwerveStates/{Setpoints,Measured}`, `SwerveChassisSpeeds/{Setpoints,Measured}` | Module setpoints vs what the modules did. |
| Paths | `Odometry/Robot`, `Odometry/Trajectory`, `Odometry/TrajectorySetpoint`, `DriveToPose/Target`, `DriveToPose/Active` | Path tracking error, how close auto-align lands. |
| Heading lock | `Drive/HeadingLock/Target`, `ErrorDegrees`, `Locked` | Heading drift, and the heading-lock expectation. |
| Vision | `Vision/<camera>/…` inputs, `Vision/<camera>/AcceptedPoses`, `RejectedPoses`, `RejectReasons`, `Objects`, and `Vision/Objects` | Camera uptime, vision vs odometry, why poses were rejected, fuel detection accuracy. |
| States | `<machine>/state`, `desired`, `overridden`, `rejectedRequests`, `lastRejected` | Timelines, slow or stuck transitions, rejected requests. |
| Connections and faults | `Motors/<name>/Connected`, `Faults`, `StickyFaults`, `StickyWarnings`; the swerve modules' `DriveConnected`, `TurnConnected`, `EncoderConnected`, `DriveFaults`, `TurnFaults`, `DriveStickyWarnings`, `TurnStickyWarnings` | The health checklist, including Sparks that reboot mid-match. |
| Loop time | `LoopTimes/<machine>`, logged by `StateMachine` | Which subsystem makes the loop slow. |
| Tuning | `NetworkInputs/Tunable/…` and `TunableDefaults/…` | Every tunable edited mid-run, and tunables left away from their code default. |
| Self-test | `SelfTest/Active`, `Step`, `Device`, `Kind`, `Target` | The pit self-test. |
| Sim fuel | `FuelSim/Launched`, `FuelSim/Acquired` (sim only) | Shots and pickups in sim logs. |

The swerve module inputs include `DriveTempCelsius` and
`TurnTempCelsius`. The build info comes from the `generateBuildInfo`
task in `build.gradle`; see
[Logging & Telemetry]({{ '/architecture/logging/' | relative_url }}).

Anything else still shows up in robotTools' explorer and in
AdvantageScope. That includes `Shooter/Setpoint/Speed`, `HoodAngle` and
`AimError`, logged by `ShooterComp` while the shooter isn't idle, and
`Game/Phase`, `Game/HubActive`, `Game/WonAuto` and `Game/DistanceToHub`
from `DashboardManager`.

## Batteries

[`BatteryTracker`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/BatteryTracker.java)
is built by `RobotState` and updated every loop. When the code starts
it reads
`src/main/deploy/batteries.json`:

```json
{
  "batteries": [
    { "id": "B1" },
    { "id": "B2" }
  ]
}
```

and fills a **Battery** chooser (a `LoggedDashboardChooser`) on the
dashboard, defaulting to `Unknown`. It logs `Battery/BootVoltage` once
when the code starts and `Battery/Id` every loop.

1. Before each match, pick the battery that's in the robot.
2. If the robot is enabled with `Unknown` still picked, a warning alert
   says "Pick the battery on the dashboard".
3. Keep the list in robotTools' **Batteries** page. **Send list to robot
   code** rewrites `batteries.json`; deploy to update the chooser.

## Self-test

Enabling **Test** mode on the Driver Station runs `SelfTest`
(`commands/SelfTest.java`). Put the robot on a cart first. The test
overrides every state machine that owns motors, then works through:

1. Driving all four modules forward at 1.5 V, then turning them to 90°
   and back.
2. Spinning each `SpinMotor` at 2 V.
3. Moving each `PosMotor` to its test position and back.

A `PosMotor`'s test position is the middle of its soft limits, or
`selfTestPosition` in its `MotorConfig`. The intake extension uses −45.
A `PosMotor` with neither is only checked for a connection. Each step is
logged under `SelfTest/`, and robotTools' Self-test tab checks direction,
movement, encoder agreement, current and connection. Cancelling Test
mode clears the overrides.

## Opening logs in robotTools

On the roboRIO, `WPILOGWriter` writes to the USB stick (`/U/logs`). In
simulation it writes to `logs/` in this repo, which git ignores.

1. Set up robotTools once (Python 3.10+ and Node 20+), following its README.
2. From the robotTools folder, run
   `backend/.venv/bin/python -m robottools serve --open`. It opens
   <http://127.0.0.1:8000>.
3. In **Settings**, add your log folders (the USB stick, this repo's
   `logs/`) and the path to this repo. robotTools checks the folders
   every 10 seconds and reads new logs on its own. You can also drag
   `.wpilog` files onto the Runs page.

No robot? **Settings → Record a practice match in the simulator**, or
`backend/.venv/bin/python -m robottools simdrive --repo <path to this repo>`,
builds the code, starts the simulator without a window, picks a battery
and an auto, and runs 15 seconds of auto and a scripted teleop. It
drives the simulator through the Sim Websockets Server extension, which
`build.gradle` declares with `defaultEnabled = false`, so a normal
**Simulate Robot Code** leaves it off. Sim logs have no real current or
temperature for the mechanisms. Add `--self-test` to run the self-test
in Test mode first.

`.github/workflows/sim-check.yml` does this on every push and pull request.
It runs the tests, records a sim match with the self-test, and runs
`robottools check` on the log. The job fails when the health checklist or
an expectation fails, and the report goes in the job summary.
