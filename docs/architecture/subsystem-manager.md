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

Every subsystem registers itself in its constructor:

```java
SubsystemManagerFactory.getInstance().registerSubsystem(this);
```

After registration, the subsystem will receive broadcasts whenever the
robot's mode changes.

## What it broadcasts

| Method on `SubsystemManager` | Called from | Effect on each subsystem |
| --- | --- | --- |
| `notifyAutonomousStart()` | `Robot.autonomousInit()` | Calls `onAutonomousStart()` |
| `notifyTeleopStart()` | `Robot.teleopInit()` | Calls `onTeleopStart()` |
| `notifyTestStart()` | `Robot.testInit()` | Calls `onTestStart()` |
| `enableAllSubsystems()` | `Robot.enabledInit()` (implicit) | `enable()` + transition to a sane state |
| `disableAllSubsystems()` | `Robot.disabledInit()` | `disable()` on every machine |
| `prepSubsystems()` | Custom auto setup | Enable + force `requestDetermine()` |

## Why a registry?

A few reasons:

- **Order independence.** Subsystems can be constructed in any order;
  the manager finds them all afterward.
- **One-line lifecycle for new mechanisms.** Add a new subsystem and it
  gets disabled-on-disable for free, without touching `Robot.java`.
- **Dashboard chooser.** When a subsystem registers, the manager
  publishes a `SendableChooser` to SmartDashboard for forcing states
  during testing.

## `SubsystemManagerFactory`

A thin wrapper around the singleton plus a few convenience methods:

```java
SubsystemManagerFactory.getInstance().registerSubsystem(this);
SubsystemManagerFactory.disableAllSubsystems();   // static shortcut
```

There's nothing surprising in there — it just keeps the call sites
short.

## Implementation detail: how subsystems get ticked

The manager **doesn't** tick subsystems. That's the
`CommandScheduler`'s job (because each `StateMachine` extends
`SubsystemBase`). The manager is purely for lifecycle and registry.

This is worth knowing if you ever wonder *"who calls
`Subsystem.periodic()`?"* — the answer is always the WPILib command
scheduler, never the manager.
