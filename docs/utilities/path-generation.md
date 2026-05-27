---
layout: default
title: Path Generation
eyebrow: Utilities
description: DynamicPathGenerator and CustomAutoBuilder — runtime PathPlanner paths.
permalink: /utilities/path-generation/
---

PathPlanner paths are normally authored in the GUI and stored as JSON.
But sometimes you need a path at runtime — "drive to the nearest
scoring location" or "drive to wherever the dashboard says". Two
utilities support that.

## `DynamicPathGenerator`

[`DynamicPathGenerator`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/DynamicPathGenerator.java)
is the thin wrapper around PathPlannerLib's `PathPlannerPath`
constructor.

### Primary API

```java
PathPlannerPath getPathFromWaypoints(
    List<Pose2d> waypoints,
    PathConstraints constraints,
    GoalEndState endState);
```

You hand it a list of waypoints (each a `Pose2d` — translation +
heading), the global velocity/accel constraints, and the desired end
state (velocity + heading at the last waypoint). It returns a path
you can feed to the standard PathPlanner follow command.

### Usage example

```java
PathPlannerPath path = DynamicPathGenerator.getPathFromWaypoints(
    List.of(
      robotState.getLatestFieldToRobot(),
      new Pose2d(4.5, 6.0, Rotation2d.fromDegrees(0)),
      new Pose2d(7.0, 6.0, Rotation2d.fromDegrees(0))
    ),
    new PathConstraints(4.0, 3.0, Math.PI, Math.PI),
    new GoalEndState(0.0, Rotation2d.fromDegrees(0)));

Command follow = AutoBuilder.followPath(path);
```

The first waypoint should usually be the robot's current pose, so the
spline starts smoothly.

### When to use

- **Field-relative actions.** Drive to the closest scoring location
  from wherever you are.
- **Composite alignments.** Stitch a few intermediate waypoints
  together to avoid obstacles.
- **Anything that depends on driver-chosen targets** without re-doing
  the path in the GUI.

### When *not* to use

- For static, planned autos — author them in PathPlanner GUI and load
  them via [`AutoCommands`]({{ '/commands/auto-commands/' | relative_url }}).
- For point-to-point movements where simple
  [`AutoAlignToPoseCommand`]({{ '/commands/auto-align/' | relative_url }})
  would do — no need for a spline when you just want to drive to a
  pose.

## `CustomAutoBuilder`

[`CustomAutoBuilder`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/CustomAutoBuilder.java)
is the higher-level cousin. It lets drivers/strategists assemble an
auto from a list of scoring locations via the dashboard.

The flow:

1. Dashboard exposes a multi-select widget of locations.
2. Driver picks the order.
3. `CustomAutoBuilder.build(selectedLocations)` returns a `Command`.

Internally it chains `DynamicPathGenerator` segments plus
`ActionCommands.aimAndShoot` at each waypoint.

This is the mechanism behind "the strategists drew a path on a
whiteboard 30 minutes ago and we need it tonight."

## `IPathCallback`

[`IPathCallback`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/IPathCallback.java)
is a single-method interface for registering callbacks on PathPlanner
events:

```java
interface IPathCallback {
  void onEvent(String eventName, PathPlannerTrajectoryState state);
}
```

Useful for triggering subsystem actions at path waypoints without
embedding them in the path's event markers.

## Pitfalls

- **Discontinuous heading.** PathPlanner expects pose headings to be
  the *robot heading*, not the path-tangent heading. Mixing them up
  gives a path that swings sideways unexpectedly.
- **Tight constraints.** Setting `MaxAccel` low makes paths take
  forever to plan and execute. Default to vehicle limits and only
  tighten where required.
- **First waypoint isn't the current pose.** PathPlanner will jump to
  the first waypoint at full velocity. Start with `getLatestFieldToRobot()`.
