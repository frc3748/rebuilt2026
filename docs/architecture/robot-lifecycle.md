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
| [`Robot.java`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/Robot.java) | Extends `LoggedRobot`. Picks the robot, owns the logger and the lifecycle hooks. |
| [`RobotState.java`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/RobotState.java) | Builds the drive, vision and the robot's superstructure from a `RobotDefinition`. |

## `Main` — the entry point

`Main.main(String[])` does exactly one thing:

```java
public static void main(String... args) {
  RobotBase.startRobot(Robot::new);
}
```

WPILib takes over from here. It instantiates a `Robot` and drives the
periodic loop at ~50 Hz.

## `Robot` — logging and hooks

`Robot` extends `LoggedRobot` rather than `TimedRobot`. That's the
AdvantageKit subclass that wraps the loop with input recording.

The constructor:

1. Reads `Constants.kRobot` to know which robot it is running on. See [Multiple Robots]({{ '/architecture/robots/' | relative_url }}).
2. Records `ROBOT`, `MODE`, `ROBOT_TYPE` and build-info (`GIT_SHA`, `GIT_BRANCH`, `GIT_DIRTY`, `BUILD_DATE`) metadata, adds `WPILOGWriter` (writes to the USB stick on the roboRIO, or `/home/lvuser/logs` without one; see `LogFolder`) and `NT4Publisher` (streams to AdvantageScope), disables REV's `StatusLogger` auto-logging, and starts the `Logger`.
3. Constructs `new RobotState(Constants.kRobot.create())` and registers it with the [`SubsystemManager`]({{ '/architecture/subsystem-manager/' | relative_url }}).

Each periodic hook is a one-liner that delegates:

```java
@Override public void robotPeriodic()      { TunableNumber.pollAll(); CommandScheduler.getInstance().run(); }
@Override public void simulationPeriodic() { robotState.updateSimulation(); }
@Override public void autonomousInit()     { SubsystemManagerFactory.getInstance().notifyAutonomousStart(); }
@Override public void teleopInit()         { SubsystemManagerFactory.getInstance().notifyTeleopStart(); }
@Override public void disabledInit()       { SubsystemManagerFactory.getInstance().disableAllSubsystems(); }
```

`testInit()` also cancels every scheduled command.

The pattern: **`Robot` doesn't make decisions, it announces transitions**.
The [`SubsystemManager`]({{ '/architecture/subsystem-manager/' | relative_url }})
broadcasts each announcement to every registered subsystem, and each
subsystem decides what to do.

## `RobotState` — wiring

`RobotState` is constructed once, from `Robot`'s constructor, with the
`RobotDefinition` of the robot `Constants.kRobot` picks. It does five
things.

### 1. Build the robot

```java
controls = definition.createControls();
shooterConstants = definition.shooter();
shotCalculator = new ShotCalculator(shooterConstants);
drive = new Drive(definition.drive(), this);
vision = new Vision(this, definition.cameras());
superstructure = definition.createSuperstructure(this);
dashboard = new DashboardManager(this, gameState, definition.name(), definition.autos(this));
```

Each piece picks its own IO from `Constants.kMode`. `Drive` does it for
the gyro and modules:

```java
return switch (Constants.kMode) {
  case REAL   -> new ModuleIOSpark(config, index);
  case SIM    -> new ModuleIOSim(config);
  case REPLAY -> new ModuleIO() {};
};
```

`Motor` and `Camera.of` do the same. See [The IO Layer Pattern]({{ '/architecture/io-pattern/' | relative_url }}).

### 2. Register subsystems

`RobotState` adds `vision`, `drive` and every state machine from
`superstructure.subsystems()` as child subsystems. When `Robot`
registers `RobotState`, the manager walks the children recursively, so
every machine receives the lifecycle broadcasts.

### 3. Bind controllers

```java
controls.bind(this);
```

`Controls` holds every binding, shared by every robot unless its
definition returns a subclass from `createControls()`. Driver buttons
for the intake or shooter are bound only if the robot has one. Each
binding is a state-machine request, never a raw motor write.

### 4. Pick the auto

`DashboardManager` fills the **Auto Choices** chooser, including the
robot's `autos(state)`, which is `Autos.all(state)` unless the robot
overrides it. When autonomous starts, `RobotState`
registers the selected auto's `build()` command as the `AUTO` state's
command and switches to `AUTO`. See
[Autos]({{ '/commands/autos/' | relative_url }}).

### 5. Hold the kinematic buffers

`RobotState` keeps
[`ConcurrentTimeInterpolatableBuffer`]({{ '/utilities/time-buffers/' | relative_url }})
histories of the robot pose and gyro rates, so vision and object
detection can look up where the robot was when a frame was captured.

## The 20 ms loop, end to end

For one tick of `robotPeriodic`:

1. `TunableNumber.pollAll()` runs the `onChange` listeners of any [tunable]({{ '/utilities/tunable-number/' | relative_url }}) edited since the last loop.
2. **CommandScheduler** runs every subsystem's `periodic()`. For each `StateMachine`:
   1. Registered motors and sensors read their inputs, and `Logger.processInputs` records them (or replaces them with logged values in replay).
   2. `update()` runs: telemetry and self-requested transitions.
   3. The override or `applyState(state)` sets motor goals, then `applyConstraints()` can overrule them.
   4. Each motor sends its final command once and logs it as `Motors/<name>/Goal` and `Mode`.
3. Scheduled commands (bindings, autos) run.

The whole loop is deterministic and replayable — point AdvantageScope
at a `wpilog` file and you can step through it.
