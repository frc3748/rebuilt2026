---
layout: default
title: Driver Dashboard (Cockpit)
eyebrow: Utilities
description: Buttons, pre-match checks and gauges that robotTools shows on the driver dashboard.
permalink: /utilities/cockpit/
---

The driver dashboard is the **Drive** page in robotTools. It replaced
Elastic. It reads everything over NetworkTables, so the robot decides
what's on it through
[`Cockpit`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/cockpit/Cockpit.java).
Next season's robot registers its own buttons, checks and gauges and the
dashboard shows them with no changes to robotTools.

`Robot.robotPeriodic()` calls `Cockpit.update()` right after the
command scheduler runs.

## Buttons

```java
Cockpit.button("stow", "Clear overrides & stow", Tab.TELEOP, command);
Cockpit.toggleButton("hubShot", "Hub-front shot", Tab.TELEOP, command, shooter::isHoldingShot);
Cockpit.confirmButton("zeroHood", "Zero hood", Tab.TEST, command);
```

Each button is a `LoggedNetworkBoolean` at `/Cockpit/Buttons/<id>`. The
dashboard sets it to true. `update()` schedules the command and sets it
back to false. A toggle button's supplier lights the button while it's
on. A confirm button needs a second press within 3 seconds. Give a
command `ignoringDisable(true)` if it should run while disabled.

The tab (`PREMATCH`, `AUTO`, `TELEOP`, `TEST`, `POSTMATCH`) picks where
the button shows. The button list goes out as JSON in
`Cockpit/Manifest`.

## Pre-match checks

```java
Cockpit.check("battery", "Battery voltage", () -> volts >= 12.3
        ? Check.pass(text) : Check.fail(text + ", swap it"));
```

A check returns `Check.pass`, `Check.warn` or `Check.fail` with a short
detail that tells the drive team what to do. Any `fail` turns the
go/no-go red. `Cockpit.isReady()` is true when nothing fails, so LEDs
can show it too. The results are logged as `Cockpit/Checks` and
`Cockpit/Ready`.

`DashboardManager` registers this robot's checks:

| Check | Fails when |
| --- | --- |
| Battery voltage | Under 12.3 V before the first enable. |
| Battery picked | Warns when no battery is picked. |
| Auto picked | Warns when the auto is None or not a "(GAME)" auto. |
| On the start pose | More than 5 cm or 2° off, with directions from the drivers' view. |
| Heading confirmed | MegaTag 1 hasn't confirmed the heading. |
| Everything connected | Any motor, encoder, gyro or camera is offline. |
| Controllers | Warns when the driver (port 0) or operator (port 1) controller is unplugged. |
| Self-test | It hasn't passed, until the first enable. A pass is saved on the roboRIO (`/home/lvuser/selftest.txt`), so on the field the check shows "Passed in the pits 40 min ago" for 12 hours on the same code. With the FMS attached it only warns, since Test mode can't run on the field. |

## Notifications

```java
Cockpit.toast(Level.INFO, "Heading fixed by MegaTag 1", "Corrected 2.4°");
Cockpit.toast(Level.ERROR, "Self-test failed", String.join(", ", failures));
```

A toast pops up on the dashboard (blue for info, amber for warning, red
for error) and goes into the post-match timeline. Use them for one-off
events the drive team should know about. Use an `Alert` for something
that stays wrong. New alerts also pop up as toasts. Confirm and toggle
buttons toast when they finish ("Zero hood done", "Hub-front shot on").
The last 30 toasts are logged as `Cockpit/Toasts`, so robotTools sees
them too.

`DisconnectNotifier` toasts in red when anything drops out: a drive or
turn motor, an encoder, a mechanism motor, the gyro (NavX or Pigeon) or
a camera. It names the device and says what to check, and toasts again
when it comes back. It checks every 0.1 s, after the first 2 s of boot.
It logs `Disconnects/Devices` and `Disconnects/Events`. The devices'
own alerts live in the `Devices` alert group, so they show in the
problems bar without toasting twice.

## Gauges

```java
Cockpit.gauge("flywheel", "Flywheel", "rps", flywheel::getSpeed, flywheel::getGoal, flywheel::isReady, this::isWorking);
Cockpit.gauge("multiplier", "Multiplier", "×", flywheel::getMultiplier, () -> flywheel.getMultiplier() != 1.0);
```

A gauge is a readout in the Shot card on the Auto and Teleop tabs, with
an optional goal (drawn as a bar) and an optional in-tolerance light. The
last argument says when it's worth showing, so the dashboard only shows
what matters right now: the shooter's gauges while it's working, the
multiplier only when someone changed it. Visible gauges are logged
together as `Cockpit/Gauges`.

## Field markers

```java
Cockpit.marker("driveTo", "Drive to", Marker.TARGET, AutoAlignToPoseCommand::activeTarget);
Cockpit.marker("hub", "Hub", Marker.AIM, () -> Optional.of(state.getDriveAnglePos()));
Cockpit.marker("fuel", "Fuel", Marker.POINT, () -> state.getVision().getClosestObjectPose());
Cockpit.marker("aimPoint", "Aiming here", Marker.CROSSHAIR, () -> state.getDrive().getRecentAimTarget().map(...));
Cockpit.zone("trench", "Hood down", () -> TrenchZone.hoodLowerRequired(state) ? Optional.of(...) : Optional.empty(), TrenchZone.hoodLowerRadius());
```

Markers are drawn on the dashboard's live field while the supplier
returns a pose. `TARGET` draws a dashed robot where a command is driving
to, with a line from the robot and the distance. `AIM` draws a line from
the robot to a point, green when `Shooter/Ready/Aim` is true. `POINT`
rings a spot. `CROSSHAIR` marks where the drive is actually aiming (the
lead point while moving), and the hub's aim line ends there. `ZONE`
draws an orange circle of the given radius with its label on top; the
trench zone shows while the hood has to stay down. They're logged as
`Cockpit/Markers` (`id`, kind, label, x, y, degrees, radius). The auto
path only shows while auto is running; after that the field shows
markers, the robot's trail and velocity, and vision fixes.

## Assists

```java
Cockpit.assist("align", "Auto-align", () -> AutoAlignToPoseCommand.activeTarget().isPresent());
Cockpit.assist("heading", "Heading lock", () -> HeadingLock.isEngaged() && DriverStation.isTeleopEnabled());
```

While an assist's supplier is true its label shows as a yellow chip
under the robot on the live field, so the driver knows when the robot is
steering for them. `DashboardManager` registers auto-align, aim lock
(`TRAVERSING_AT_ANGLE`), path following and heading lock. The active
labels are logged as `Cockpit/Assists`. Chips are hidden during auto.

## Controller rumble

The driver and operator controllers rumble for half a second when the
hub flips, pulse for the 3 seconds before a flip, and the driver's
controller gives a short buzz when the shot becomes ready.

## Cameras

```java
Cockpit.camera("limelight-fuel", "Fuel Camera", "limelight-fuel", "http://limelight-fuel.local:5800/stream.mjpg", camera::isConnected);
```

`Vision` registers every camera. The dashboard looks up the stream
under `/CameraPublisher/<stream>/streams`, then tries the fallback URL.
A camera tile only appears once its stream sends a frame, and disappears
when the robot says the camera is offline or the dashboard loses the
robot. The field shrinks to make room. In simulation the stream is
`<networkName>-processed`, which `CameraIOPhotonSim` leaves off to save
CPU.

Each Limelight stream costs field bandwidth (the field caps the robot at
4 to 7 Mbps). Set the Limelight's stream to low bandwidth mode if the
radio gets busy.

## Effects

In teleop, the screen edge glows amber at 30 s, red at 15 s, and flashes
with a big countdown for the last 5 s. Banners pop up at 30, 15 and 10
seconds. Before a shift that flips the hub, a callout counts down
"Shoot in 5…" (green) or "Stop shooting in 5…" (red), and the whole
screen flashes that color for the last 3 seconds. When the hub flips, a
"Shoot!" or "Stop shooting" banner washes the screen. If the shooter
fires while the hub is inactive, the screen strobes red with "Stop
shooting · hub inactive". There are no sounds.

## What else the dashboard reads

| Topic | Shown as |
| --- | --- |
| `Cockpit/Autos` | Every auto's name, mode, start pose and path (JSON), for the mode chips and tap-to-pick field. |
| `Cockpit/Auto/Path`, `Start` | The selected auto's path and start pose. |
| `Cockpit/Auto/Playback/*` | The auto's PathPlanner trajectory, timed, for the play button. |
| `Cockpit/Auto/Step`, `DriftMeters` | What auto is doing and how far off the path it is. |
| `Cockpit/Vision` | One line per pose camera: name, connected, tags seen, seconds since its last accepted fix. The teleop vision card appears when a camera is offline or no camera has had a fix for 3 s. |
| `Game/*` | Hub state, whether the next shift flips it (`HubActiveNext`), the countdown, and `Timeline`: each teleop segment's match-time span and whether our hub is active, inactive, both or unknown, drawn as the strip in the clock card. |
| `Shooter/Ready/*` | The shot tile. |
| `SwerveStates/*`, `SwerveChassisSpeeds/Measured`, `Drive/Module*` inputs | The swerve tile. |
| `SystemStats/*`, `PowerDistribution/TotalCurrent`, `LoggedRobot/FullCycleMS` | The battery tile. |
| `Alerts/errors`, `Alerts/warnings` | The red and amber problems row. |
| `<subsystem>/state` | The states tile. |

## Recorded by the dashboard

The dashboard keeps its own record of each match while it's open. The
Shot card shows the time since the last shot, the last and average
cycle, and the volley count once teleop's first volley lands. During
auto it records every robot pose it receives, so Post-match → Auto
replay plays the plan (dashed) against what the robot drove (solid) and
marks the spot where it was furthest off. Post-match → Event shows
volleys, shots into an inactive hub, lowest voltage and auto drift for
each match at the event, newest in yellow. These live in the
dashboard's browser storage, not on the robot.

Raise driver-facing problems with WPILib `Alert`s. Errors show as a red
bar on every tab and warnings in amber. Set them only while the problem
is real, never every loop as a message.
