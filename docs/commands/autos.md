---
layout: default
title: Autos
eyebrow: Commands
description: How autonomous routines are written, shared by every robot, and picked from the dashboard.
permalink: /commands/autos/
---

Every autonomous routine is an
[`AutoRoutine`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/commands/autos/AutoRoutine.java).
[`Autos.all(state)`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/commands/autos/Autos.java)
lists the shared routines. Every robot gets them through
`RobotDefinition.autos(state)`, which returns `Autos.all(state)` unless
the robot overrides it, and `DashboardManager` puts them in the
**Auto Choices** chooser.

## `AutoRoutine`

```java
public abstract class AutoRoutine {
    protected AutoRoutine(String name, String... pathNames);

    public String name();
    public abstract Command build();
    public List<PathPlannerPath> previewPaths();   // loads pathNames, for the dashboard preview

    public static AutoRoutine none();                  // does nothing
    public static AutoRoutine pathPlanner(String name); // wraps a PathPlanner .auto file
}
```

`pathNames` lists every PathPlanner path the routine uses. It drives the
field preview and lets a subclass load all paths up front.

## Picking an auto

The **Auto Choices** chooser holds, in order:

1. `None` (the default).
2. Every `.auto` file in `src/main/deploy/pathplanner/autos/`, wrapped with `AutoRoutine.pathPlanner`.
3. The robot's `autos(state)`.

Drivers pick autos on the robotTools **Drive** page (Pre-match tab).
`DashboardManager` publishes every auto's mode, start pose and path as
`Cockpit/Autos`, and the selected one's timed trajectory for the play
button. Tap a mode, then tap where the robot is on the field, and the
auto from that mode and start drops in. With **Auto-pick from robot** on,
once vision has confirmed the heading, the dashboard picks the auto
whose start the robot is sitting on, and switching modes keeps the
robot's spot (or the same side). A tap always wins until the robot is
moved to another start. While a path runs, PathPlanner's active path is logged as
`Odometry/Trajectory`. The **Auto picked** pre-match check warns while
the auto is None or isn't a "(GAME)" auto.

When autonomous starts, `RobotState` calls `build()` on the selection,
registers the result as the `AUTO` state's command, and enters `AUTO`.

## `PathAuto`

Each auto is one file in `commands/autos/` that extends `PathAuto`:

```java
public class DepotSideQuickShoot extends PathAuto {
    public DepotSideQuickShoot(RobotState state) {
        super(state, "Depot Side Quick Shoot (GAME)",
                "Start Depot Side to Mid Intake",
                "Mid Intake to Start Depot Side",
                ...);
    }

    @Override
    protected Command routine() {
        return Commands.sequence(
                shooter(Shooter.State.HUB_TRACKING),
                intake(Intake.State.INTAKE),
                follow("Start Depot Side to Mid Intake"),
                ...);
    }
}
```

`PathAuto.build()` loads every path, resets the pose to the first
path's start (flipped for red), then runs `routine()`. If a path fails
to load, it returns a print command named `<name> (FAILED)` instead.

Helpers available inside `routine()`:

| Helper | What it does |
| --- | --- |
| `follow(path)` | Follows a preloaded path with `FollowPath` (see below). |
| `intake(state)` / `shooter(state)` | Transition and wait for it to land. |
| `requestIntake(state)` / `requestShooter(state)` | Transition without waiting. |
| `spinUp()` | The shooter's `spinUp()` command. |
| `shake(seconds)` | Waits `seconds` while running `ActionCommands.shakeIntake`. |
| `aim()` / `turn()` | `ActionCommands.aimAtHub` / `turnToHub`. |
| `nudge(meters)` | Auto-align straight forward by a distance. |
| `shootFromStart(path, shakeSeconds)` | Follow the path while tracking, aim, shoot, shake. |

The mechanism helpers go through the robot's
[`Superstructure`]({{ '/architecture/robots/' | relative_url }}#superstructure),
so on a robot without that mechanism they do nothing. `shake(seconds)`
still waits its time, so a drivetrain-only robot drives every auto's
paths.

## Path following

PathPlanner's own follower runs on a clock. If the robot gets held up,
say by scraping a wall, the target keeps moving. The robot then cuts
straight across to catch it, which is how a delayed auto ends up
clipping the trench or hitting fuel. It also ends the path when the
clock runs out, even if the robot isn't there yet. `FollowPath`
(`commands/FollowPath.java`) mixes that with BLine's position-based
following:

- **Speed:** it keeps PathPlanner's time-optimal speed and acceleration profile.
- **Steering:** every loop it finds the closest point on the path. It drives along the path's direction there, with one correction that pulls the robot back onto the line (`Path/Translation kP`) and one that closes the distance to the target along the path.
- **No running away:** the profile's clock only runs while the robot keeps up. The target never gets more than `Path/Max Lead` (0.5 m) ahead, so it can't pull the robot across a corner.
- **Ending:** a path ends when the robot is within 2 cm and 2° of the end. If it's pushing against something, like a wall, with less than 5 cm left along the path, it ends there. Otherwise it gives up after `Path/Settle Seconds` (0.75 s). A path that ends moving hands off as soon as its profile ends.
- **Turning near walls:** `PathReference` checks the bumper footprint (`DriveConfig#bumperLength`, `bumperWidth`) against the field walls, the trench walls and the hubs (`game/FieldObstacles`). Where a turned robot wouldn't fit but a square one would, like inside the trench, it holds the nearest 90° heading. The turn is spread out at the auto turn rate, so the robot squares up before it reaches the tight spot.
- **Paths drawn past the wall:** some paths are drawn past the field wall on purpose, to make up for slip. Where that happens the robot rides 3 cm off the wall at full speed instead of grinding into it, with the join rounded off over about ±40 cm.

It logs `Auto/Path/CrossTrackMeters`, `AlongTrackMeters`, `Held`, `Time`
and the followed line as `Auto/Path/Reference`. robotTools' match grade
scores "Followed the path" from the cross-track error.

`BlairAutoTest` runs both Blair autos on the full REBUILT field in the
simulator. Every path has to finish, staying within 15 cm of the
reachable path. They stay within about 8–11 cm. With PathPlanner's
follower they cut up to 85 cm.

## The catalog

`Autos.all(state)` returns 14 routines. Each gets a mode with
`.mode("...")`, which groups the same strategy from different starts on
the dashboard:

| Name | Mode | What it does |
| --- | --- | --- |
| **Center Only Starting 8 (GAME)** | Starting 8 | Drive to Home Center and shoot the preload. |
| **Depot Only Starting 8 (GAME)** | Starting 8 | Same, from the depot side. |
| **HP Only Starting 8 (GAME)** | Starting 8 | Same, from the human-player side. |
| **Depot Side To Depot (GAME)** | To station | Home Depot, then intake at the depot and shoot. |
| **Depot Side To Depot End at Mid (GAME)** | To station, end mid | Shoot, intake at the depot, go under the trench and sweep mid while pass-tracking. |
| **HP Side To HP (GAME)** | To station | Shoot the preload, drive to the HP station, shoot again. |
| **HP Side To HP End at Mid (GAME)** | To station, end mid | Shoot at the HP station, then sweep mid while pass-tracking. |
| **Depot Side Depot Mid Half Sweep (GAME)** | Half sweep | Intake at the depot and shoot, sweep mid, return to Home HP and shoot. |
| **Depot Side Quick Shoot (GAME)** | Quick shoot | Two runs to mid, coming back to the start to shoot after each. |
| **HP Side Quick Shoot (GAME)** | Quick shoot | Same, from the HP side. |
| **Depot Side Circut Shoot (GAME)** | Circuit | Two circuits through mid, shooting after each. |
| **Depot Side Blair (GAME)** | Blair | Two intake runs from the depot side, shooting after each. |
| **HP Side Blair (GAME)** | Blair | Same, from the HP side. |
| **Depot Side Bump (GAME)** | Bump | Like Depot Side Blair, coming back over the bump. |

The dashboard labels each start by the first word of the name before
"Side" or "Only" (Depot, HP, Center), so keep that naming.

The source file is authoritative; the paths each auto uses are listed
in its constructor.

## Diagnostic autos

Six autos named **Diagnostic: …** check that the drive, odometry and path following are good enough for real autos. They're in `Autos.all`, so every robot has them. Each drives from wherever the robot is sitting, with on-the-fly PathPlanner paths and the same path controller as the match autos, so they work in the pit.

| Auto | Moves | Room needed |
| --- | --- | --- |
| Forward 2 ft and back | Straight out and back | 1 m |
| Left 2 ft and back | Sideways out and back | 1 m |
| 1 m square | Four sides, holding heading | 1.5 m square |
| Full spin | Four 90° turns in place | Robot width |
| 2 m turning 180 and back | Drives while turning, then back | 2.5 m |
| 3 m at auto speed and back | At 4 m/s and 3.5 m/s² (your fastest autos), capped at the robot's top speed | 4 m |

To run one, tape the floor at the robot's corners, pick it in the auto chooser and enable Autonomous. It passes if it ends within 5 cm and 3° of where it should (`DiagnosticAuto.kPassMeters`, `kPassDegrees`).
- **Results:** each one toasts its result and logs `Diagnostics/<test>/` (error, heading error, the worst distance from the path while following it, and the expected and actual poses). The Test tab lists every result.
- **Tape check:** the pass/fail compares against odometry. If the robot passes but isn't back on the tape, odometry is off. Check the wheel radius, then the module offsets.
- **Simulator:** `CompDiagnosticsTest`, `SecondaryDiagnosticsTest` and `PracticeDiagnosticsTest` run all six in the simulator for every robot, with stepped time. A change that breaks path following fails the build.

The pre-match check warns while a diagnostic is selected, since it isn't a match auto.

To add one, extend `DiagnosticAuto`, list its moves relative to the start with `drive(forward, left, degrees)`, `turn(degrees)` or `atAutoSpeed(...)`, and add it to `Autos.all`.

## Measuring autos

Four autos named **Measure: …** measure the real robot instead of trusting the numbers in the code. Turn on tuning mode first. Each one puts what it found on the Tune tab, so **Save** keeps it and the next deploy writes it into the code. Without tuning mode they only report.

| Auto | Setup | Measures | Puts on the Tune tab |
| --- | --- | --- | --- |
| Wheel radius | Room to spin | Spins 1.5 turns at 1 rad/s and compares how far the wheels rolled with how far the gyro turned | `Drive/Wheel Radius` (applies after a restart) |
| Drive feedforward | 2 m of open floor in front | Ramps the drive voltage at 1 V/s until it's gone 2 m, stops, then steps 3 V back to the start. Fits volts = kS + kV·speed + kA·acceleration to every sample, ignoring any near the current limit | `Drive PID/kS`, `kV`, `kA`, a starting `Drive PID/kP` from the fit, and `Drive Sim/kP`; also reports the top speed at 12 V |
| Steering | Wheels on the floor, room for them to turn | Ramps every turn motor to ±3 V at 1 V/s and steps ±2 V, then fits kS, kV and kA per module and averages them | `Turn PID/kP`, `kD`, `Steer FF`, and `Turn Sim/kP`, `kD`; also reports how far apart the modules are |
| Slip current | Front bumper flat against a wall | Lifts the drive current limit to 80 A and ramps the voltage until the wheels spin (up to 10 V), then puts the limit back | `Drive/Current Limit` at 90% of the current where the wheels broke loose; also reports the wheel grip (μ) |

- **Results:** each one toasts its result and logs `Measure/<test>/`. The Test tab lists them under **Measurements**.
- **Matches the code:** a measurement within 1% (wheel radius), 0.05 V and 5% (feedforward) or 3 A (slip current) of the code says so and changes nothing.
- **Drive feedforward:** if it pushes at 2 V or more without moving for 0.3 s, it stops, backs off, and says how far it got instead of grinding into the wall.
- **Slip current:** if the wheels hold all the way, the limit can't make them slip and nothing changes. If the wheels spin before the current builds up, the robot wasn't against a wall, and it says so.
- **From the Tune tab:** **Auto-tune** on the Drive PID or Drive Sim group runs Drive feedforward, and on Turn PID or Turn Sim runs Steering. See [Tuning]({{ '/utilities/tunable-number/' | relative_url }}#auto-tune).
- **Simulator:** `CompMeasureTest` and `PracticeMeasureTest` run them in maple-sim. The wheel radius has to come out within 2%. The fitted feedforward has to predict a real 2 V step within 5%. The slip current has to match the wheel grip in the drive config.

## Adding a new auto

1. Draw the path(s) in the PathPlanner GUI; they save to `src/main/deploy/pathplanner/paths/`.
2. Add a class in `commands/autos/` that extends `PathAuto`. Pass the name and every path name to `super`; the first path sets the starting pose.
3. Write `routine()` with the helpers above.
4. Add `new MyAuto(state).mode("...")` to `Autos.all`. Reuse an existing mode when it's the same strategy from another start. `CockpitTest` fails if an auto has no mode.
5. Bump the count in `CompRobotTest.everyAutoFindsItsPaths` and run the tests; the test fails if any auto can't load its paths or a path name's case doesn't match its file. `PracticeRobotTest` also builds every auto on the practice robot.

## Pitfalls

- **Path names must match the file exactly, including case.** macOS
  and Windows ignore case; the roboRIO does not. `everyAutoFindsItsPaths`
  compares every path name against the files on disk, so it catches this
  on any computer.
- **An auto does nothing.** One of its paths failed to load, so
  `build()` returned the `(FAILED)` print command. The load error is
  printed to the console when it runs.
