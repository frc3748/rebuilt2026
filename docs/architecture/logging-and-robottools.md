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
| Heading lock | `Drive/HeadingLock/Target`, `Drive/HeadingLock/ErrorDegrees` | Heading drift. |
| Vision | `Vision/<camera>/…` inputs, `Vision/<camera>/AcceptedPoses` and `RejectedPoses` | Camera uptime, vision vs odometry. |
| States | `<machine>/state` | State-machine timelines. |
| Tuning | `NetworkInputs/Tunable/…` | Every tunable edited mid-run. |

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
temperature for the mechanisms.
