---
layout: default
title: Auto Commands
eyebrow: Commands
description: The registered autonomous routines and how they are selected.
permalink: /commands/auto-commands/
---

[`AutoCommands`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/commands/AutoCommands.java)
holds the catalog of autonomous routines and the data classes that
describe them.

## The `AutoClass`

Every auto routine is described by a small data object:

```java
public static class AutoClass {
  public final String name;
  public final List<String> sequentialPathStrings; // PathPlanner path names
  public final boolean climbAtEnd;

  public Command getCommand(RobotState state)            { … }
  public List<PathPlannerPath> getAutoDisplayList()      { … }
}
```

- `sequentialPathStrings` is a list of PathPlanner path filenames to
  run in order.
- `climbAtEnd` decides whether to chain `ActionCommands.autoClimb` at
  the very end.
- `getCommand` builds the actual `Command` — usually a sequence of
  follow-path + shoot.

## Registered autos

The current catalog (see `AutoCommands` source for the authoritative
list):

| Name | What it does |
| --- | --- |
| **Center Only Starting 8 (GAME)** | Score the 8 preload from a center start. |
| **Depot Only Starting 8 (GAME)** | Start from the depot, score 8. |
| **HP Only Starting 8 (GAME)** | Start from human-player side, score 8. |
| Variants with `+ climb` | Same routine, plus `autoClimb` at the end. |
| Circuit / sweep / Blair variants | Alternative path orderings for specific match strategies. |

Each routine has a "GAME" suffix that distinguishes it from
characterization/test autos.

## How they get into the chooser

`RobotState` calls into `AutoCommands` to populate a
`LoggedDashboardChooser<Command>`:

```java
chooser.addOption("Center Only Starting 8 (GAME)",
                  AutoCommands.byName("Center Only Starting 8").getCommand(this));
```

The selected `Command` is returned by `RobotState#getAutoCommand()`
and run from `Robot.autonomousInit()`.

## How they execute

When an `AutoClass.getCommand()` is invoked:

1. **Reset pose** — the first path's start pose becomes the assumed
   field origin for this match. (Vision will correct over the first
   few hundred ms anyway.)
2. **For each path string** — load the `PathPlannerPath` via
   PathPlannerLib, build a follow command with the registered
   `PPHolonomicDriveController`, and chain `shoot` commands at the
   appropriate event markers.
3. **If `climbAtEnd`** — append `ActionCommands.autoClimb(state)`.

The whole thing is one composed `Command`. The state machines do
their normal thing throughout — auto routines are *just* a different
binding to the same primitives.

## Adding a new auto

1. Build the path(s) in PathPlanner GUI; place them in
   `src/main/deploy/pathplanner/paths/`.
2. Add an entry in `AutoCommands` with the path filename(s).
3. Add the chooser line in `RobotState`'s auto-selection block.
4. Deploy and test in sim before trusting it on real hardware.

For paths that need to be generated at *runtime* — say, "drive to the
nearest scoring location and shoot" — use
[`DynamicPathGenerator`]({{ '/utilities/path-generation/' | relative_url }})
instead.

## See also

- [`AutosConstants`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/commands/autos/AutosConstants.java)
  — fixed waypoints used by multiple autos.
- [`Autos`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/commands/autos/Autos.java)
  — helpers for repeated patterns (e.g. "drive then shoot").
