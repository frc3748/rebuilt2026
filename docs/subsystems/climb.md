---
layout: default
title: Climb
eyebrow: Subsystem
description: Single-motor elevator that pulls the robot to the rung.
permalink: /subsystems/climb/
---

| | |
| --- | --- |
| **Source** | `src/main/java/frc/robot/subsystems/climb/` |
| **Public class** | [`Climb`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/subsystems/climb/Climb.java) extends `StateMachine<Climb.State>` |
| **Constants** | `ClimbConstants` |

The climb is a one-motor elevator. It extends up to grab the rung,
then retracts to pull the robot up. No sensors beyond the motor's
own encoder — current spikes and encoder position do the rest.

## States

| State | Behavior |
| --- | --- |
| `IDLE` | Motor off. |
| `STOW` | Hold the stow setpoint. |
| `UP` | Extend to reach the bar. |
| `DOWN` | Pull the robot up, using the slower closed-loop slot 1. |
| `ZEROING` | Drive gently down into the hard stop with a reduced current limit. |

## Zeroing

Zeroing is a state, not a separate command. Entering `ZEROING` lowers the
current limit and runs the motor down at `Climb/Lower Motor Output`. When
the current passes `Climb/Zero Current Threshold`, the climb resets its
encoder to zero and requests `STOW`. Leaving `ZEROING` for any reason
restores the normal current limit.

The robot zeroes the climb once, at the first autonomous or teleop start.
The "Climb Zero" dashboard button and the operator Y + left trigger chord
request `ZEROING` again.

## Mechanism

- **One motor** — NEO in brake mode when the elevator is stationary.
- **Hard stops** — top and bottom of travel, used for zeroing.

## Auto-climb

[`ActionCommands.autoClimb(RobotState)`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/commands/ActionCommands.java)
composes drive + climb:

1. Drive auto-aligns to the configured climb pose.
2. Climb transitions `STOW → UP`.
3. Drive holds position while operator manually triggers `UP → DOWN`.
4. The climb subsystem auto-completes `DOWN → CLIMB` when the stall
   current threshold is exceeded.

## Logging

The climb publishes a 3D pose for the elevator stage to AdvantageScope,
so you can see the climb deployment over time.

## Pitfalls

- **`DOWN → CLIMB` never fires.** Stall threshold is too high, or the
  motor isn't actually loaded. Plot `Climb/current` in AdvantageScope
  during a real attempt; pick a number that's clearly above unloaded
  draw and clearly below the breaker trip.
- **Zeroing never finishes.** The stall threshold is above what the motor
  draws at the hard stop. Lower `Climb/Zero Current Threshold`.
- **Robot sags after climbing.** The climb motor idles in brake mode by
  default (`MotorConfig`); confirm nothing calls `.coast()` on `kClimb`.
