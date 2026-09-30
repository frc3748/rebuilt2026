---
layout: default
title: Subsystem Manager
eyebrow: Architecture
description: A singleton registry that broadcasts lifecycle events to every state machine.
permalink: /architecture/subsystem-manager/
---

The [`SubsystemManager`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/state/SubsystemManager.java)
is the glue between `Robot`'s lifecycle hooks and the many subsystems
that want to react to them.

## The contract

`Robot` registers one machine, the root:

```java
SubsystemManagerFactory.getInstance().registerSubsystem(robotState);
```

Registration walks `getChildSubsystems()` recursively, so every machine
added with `addChildSubsystem` (vision, drive, the superstructure's
subsystems, and their own children such as the shooter's hood,
flywheel, hopper and kicker) is registered too. After registration,
each one receives broadcasts whenever the robot's mode changes.

## What it broadcasts

| Method on `SubsystemManager` | Called from | Effect on each subsystem |
| --- | --- | --- |
| `notifyAutonomousStart()` | `Robot.autonomousInit()` | `prepSubsystems()`, then `onAutonomousStart()` |
| `notifyTeleopStart()` | `Robot.teleopInit()` | `prepSubsystems()`, then `onTeleopStart()` |
| `notifyTestStart()` | `Robot.testInit()` | `prepSubsystems()`, then `onTestStart()` |
| `disableAllSubsystems()` | `Robot.disabledInit()` | `disable()` |
| `prepSubsystems()` | Each `notify…` method | `enableAllSubsystems()` (`enable()`), then `determineAllSubsystems()` (`determineState()`) |

## Why a registry?

A few reasons:

- **Order independence.** Subsystems can be constructed in any order;
  the manager finds them all afterward.
- **One-line lifecycle for new mechanisms.** Add a new subsystem as a
  child of a registered machine, as `ShooterComp` does with its hopper
  and kicker, and it gets disabled-on-disable for free, without
  touching `Robot.java`.
- **Dashboard chooser.** When a subsystem registers, the manager
  publishes a `SendableChooser` to SmartDashboard for forcing states
  during testing.

## `SubsystemManagerFactory`

Holds the singleton. `getInstance()` creates it on first use, and
`setInstance(manager)` replaces it:

```java
SubsystemManagerFactory.getInstance().registerSubsystem(robotState);
```

## Implementation detail: how subsystems get ticked

The manager **doesn't** tick subsystems. That's the
`CommandScheduler`'s job (because each `StateMachine` extends
`SubsystemBase`). The manager is purely for lifecycle and registry.

This is worth knowing if you ever wonder *"who calls
`Subsystem.periodic()`?"* — the answer is always the WPILib command
scheduler, never the manager.
