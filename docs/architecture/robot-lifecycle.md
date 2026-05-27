---
layout: default
title: Robot Lifecycle
eyebrow: Architecture
description: How Main, Robot, and RobotState collaborate to start, run, and shut down the robot.
permalink: /architecture/robot-lifecycle/
---

The lifecycle splits across three files:

| File | Role |
| --- | --- |
| [`Main.java`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/Main.java) | JVM entry point — a one-liner. |
| [`Robot.java`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/Robot.java) | Extends `LoggedRobot`. Owns the logger and the lifecycle hooks. |
| [`RobotState.java`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/RobotState.java) | Owns every subsystem. Wires controller bindings. |

## `Main` — the entry point

`Main.main(String[])` does exactly one thing:

```java
public static void main(String... args) {
  RobotBase.startRobot(Robot::new);
}
```

WPILib takes over from here. It instantiates a `Robot`, calls
`robotInit()`, then drives the periodic loop at ~50 Hz.

## `Robot` — logging and hooks

`Robot` extends `LoggedRobot` rather than `TimedRobot`. That's the
AdvantageKit subclass that wraps the loop with input recording.

The constructor:

1. Configures the AdvantageKit `Logger` with `WPILOGWriter` (writes to USB on the roboRIO) and `NT4Publisher` (streams to AdvantageScope).
2. Disables REV's built-in auto-logging to avoid double-recording.
3. Constructs the [`RobotState`]({{ '/architecture/robot-state/' | relative_url }}) singleton.

Each periodic hook is a one-liner that delegates:

```java
@Override public void robotPeriodic() { CommandScheduler.getInstance().run(); }
@Override public void autonomousInit() { SubsystemManagerFactory.getInstance().notifyAutonomousStart(); }
@Override public void teleopInit()     { SubsystemManagerFactory.getInstance().notifyTeleopStart(); }
@Override public void disabledInit()   { SubsystemManagerFactory.getInstance().disableAllSubsystems(); }
```

The pattern: **`Robot` doesn't make decisions, it announces transitions**.
The [`SubsystemManager`]({{ '/architecture/subsystem-manager/' | relative_url }})
broadcasts each announcement to every registered subsystem, and each
subsystem decides what to do.

## `RobotState` — wiring

`RobotState` is constructed once, from `Robot`'s constructor. It does
five things:

### 1. Choose IO implementations

A single `robotState` field — `1` for real hardware, `2` for sim,
anything else for replay — decides which IO subclass each subsystem
gets:

```java
switch (robotState) {
  case 1 -> new Drive(new GyroIOPigeon2(), new ModuleIOSpark(0), new ModuleIOSpark(1), …);
  case 2 -> new Drive(new GyroIO(){},      new ModuleIOSim(),    new ModuleIOSim(),    …);
  default -> new Drive(new GyroIO(){},     new ModuleIO(){},     new ModuleIO(){},     …);
}
```

See [The IO Layer Pattern]({{ '/architecture/io-pattern/' | relative_url }}) for the full story.

### 2. Register subsystems

Every subsystem registers itself with the [`SubsystemManager`]({{ '/architecture/subsystem-manager/' | relative_url }})
in its constructor. After `RobotState`'s constructor returns, the
manager knows about all of them.

### 3. Bind controllers

Driver and operator Xbox controllers get bound to subsystem transition
requests:

```java
controller.a().onTrue(shooter.transitionCommand(Shooter.State.HUB_TRACKING));
controller.b().onTrue(shooter.transitionCommand(Shooter.State.IDLE));
```

Notice there are no `runOnce(() -> motor.set(...))` calls. Driver intent
is *always* expressed as a state-machine request.

### 4. Populate the auto chooser

A `LoggedDashboardChooser<Command>` is filled from
[`AutoCommands`]({{ '/commands/auto-commands/' | relative_url }}). The
selected command runs in `autonomousInit()`.

### 5. Hold the kinematic buffers

`RobotState` maintains
[`ConcurrentTimeInterpolatableBuffer`]({{ '/utilities/time-buffers/' | relative_url }})
instances for pose, turret angle, and drive velocity history. These
buffers let the shooter aim at where the target *was* when a vision
frame was taken, not where the camera currently points.

## The 20 ms loop, end to end

For one tick of `robotPeriodic`:

1. **CommandScheduler** runs.
2. Each registered subsystem's `periodic()` is called.
   1. The IO reads inputs (`io.updateInputs(inputs)`).
   2. `Logger.processInputs(prefix, inputs)` records them (and replaces them with logged values in replay mode).
   3. The state machine processes pending transitions and runs the current state's command.
3. Outputs from any state-machine action propagate back to motors via the IO.
4. The logger flushes the frame.

The whole loop is deterministic and replayable — point AdvantageScope
at a `wpilog` file and you can step through it.
