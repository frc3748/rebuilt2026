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
| `follow(path)` | `AutoBuilder.followPath` on a preloaded path. |
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
