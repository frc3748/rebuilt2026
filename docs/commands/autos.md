---
layout: default
title: Autos
eyebrow: Commands
description: How autonomous routines are written, registered with a robot, and picked from the dashboard.
permalink: /commands/autos/
---

Every autonomous routine is an
[`AutoRoutine`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/commands/autos/AutoRoutine.java).
A robot's `Superstructure.autos()` returns its routines, and
`DashboardManager` puts them in the **Auto Choices** chooser.

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
3. The robot's own routines from `superstructure.autos()`.

While the Driver Station is in autonomous mode, the selected routine's
`previewPaths()` are drawn on the dashboard field (alliance-flipped) and
logged as `Auto Trajectory 3D`. `Robot/AutoChoosed` turns true when the
selected name contains "GAME", so drivers can see a real auto is picked.

When autonomous starts, `RobotState` calls `build()` on the selection,
registers the result as the `AUTO` state's command, and enters `AUTO`.

## Competition autos

Each competition auto is one file in `robots/competition/autos/` that
extends `CompetitionAuto`:

```java
public class DepotSideQuickShoot extends CompetitionAuto {
    public DepotSideQuickShoot(CompetitionSuperstructure robot) {
        super(robot, "Depot Side Quick Shoot (GAME)",
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

`CompetitionAuto.build()` loads every path, resets the pose to the first
path's start (flipped for red), then runs `routine()`. If a path fails
to load, it returns a print command named `<name> (FAILED)` instead.

Helpers available inside `routine()`:

| Helper | What it does |
| --- | --- |
| `follow(path)` | `AutoBuilder.followPath` on a preloaded path. |
| `intake(state)` / `shooter(state)` / `flywheel(state)` | Transition and wait for it to land. |
| `requestIntake(state)` / `requestShooter(state)` | Transition without waiting. |
| `shake(seconds)` | `ActionCommands.shakeIntake` for a fixed time. |
| `aim()` / `turn()` | `ActionCommands.aimAtHub` / `turnToHub`. |
| `nudge(meters)` | Auto-align straight forward by a distance. |
| `shootFromStart(path, shakeSeconds)` | Follow the path while tracking, aim, shoot, shake. |

## The competition catalog

`CompetitionSuperstructure.autos()` registers 15 routines:

| Name | What it does |
| --- | --- |
| **Center Only Starting 8 (GAME)** | Drive to Home Center and shoot the preload. |
| **Depot Only Starting 8 (GAME)** | Same, from the depot side. |
| **HP Only Starting 8 (GAME)** | Same, from the human-player side. |
| **Depot Side To Depot (GAME)** | Home Depot, then intake at the depot and shoot. |
| **Depot Side To Depot End at Mid (GAME)** | Shoot, intake at the depot, go under the trench and sweep mid while pass-tracking. |
| **HP Side To HP (GAME)** | Shoot the preload, drive to the HP station, shoot again. |
| **HP Side To HP End at Mid (GAME)** | Shoot at the HP station, then sweep mid while pass-tracking. |
| **Depot Side Depot Mid Half Sweep (GAME)** | Intake at the depot and shoot, sweep mid, return to Home HP and shoot. |
| **Depot Side Quick Shoot (GAME)** | Two runs to mid, coming back to the start to shoot after each. |
| **HP Side Quick Shoot (GAME)** | Same, from the HP side. |
| **Depot Side Circut Shoot (GAME)** | Two circuits through mid, shooting after each. |
| **Depot Side Blair (GAME)** | Two intake runs from the depot side, shooting after each. |
| **HP Side Blair (GAME)** | Same, from the HP side. |
| **Depot Side Bump (GAME)** | Like Depot Side Blair, coming back over the bump. |
| **CUSTOM AUTO (GAME)** | Built on the dashboard; see below. |

The source file is authoritative; the paths each auto uses are listed
in its constructor.

## `CustomAuto`

`CustomAuto` builds an auto from the dashboard, with no deploy. It
publishes 100 choosers, `Auto Parallel 0` to `Auto Parallel 99`, in
10 lanes of 10 steps: lane *n* is choosers *10n* to *10n + 9*. Each
chooser offers `None` plus every public static method in
[`ActionCommands`]({{ '/commands/action-commands/' | relative_url }})
that takes a `CompetitionSuperstructure` and returns a `Command`, found
by reflection.

When the auto starts, each lane runs its chosen steps in sequence and
all lanes run in parallel. The choosers are read at that moment, so
they can change until the match starts.

## Adding a new auto

1. Draw the path(s) in the PathPlanner GUI; they save to `src/main/deploy/pathplanner/paths/`.
2. Add a class in `robots/competition/autos/` that extends `CompetitionAuto`. Pass the name and every path name to `super`; the first path sets the starting pose.
3. Write `routine()` with the helpers above.
4. Add `new MyAuto(this)` to `CompetitionSuperstructure.autos()`.
5. Bump the count in `CompetitionRobotTest.everyAutoFindsItsPaths` and run the tests; the test fails if any auto can't load its paths or a path name's case doesn't match its file.

A robot without mechanisms can still return `AutoRoutine.pathPlanner("…")`
or its own `AutoRoutine` subclass from its superstructure.

## Pitfalls

- **Path names must match the file exactly, including case.** macOS
  and Windows ignore case; the roboRIO does not. `everyAutoFindsItsPaths`
  compares every path name against the files on disk, so it catches this
  on any computer.
- **An auto does nothing.** One of its paths failed to load, so
  `build()` returned the `(FAILED)` print command. The load error is
  printed to the console when it runs.
