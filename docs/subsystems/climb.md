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

```java
enum State {
  UNDETERMINED,
  STOW,    // fully retracted
  UP,      // extended to reach height
  DOWN,    // pulling robot up to bar
  CLIMB    // holding final position, brake engaged
}
```

State flow during a climb:

```
   STOW ──▶ UP ──▶ (driver positions robot) ──▶ DOWN ──▶ CLIMB
```

`CLIMB` engages brake mode on the motor to hold the robot's weight
without consuming amps.

## Detecting the end of travel

There's no external sensor. Two cues tell the subsystem when the climb
is finished:

- **Encoder position.** `UP` ends when the motor reaches the extended
  setpoint (`ClimbConstants.kExtendedPositionRad`).
- **Current spike.** `DOWN → CLIMB` triggers when the motor stalls
  against the rung — current crosses `ClimbConstants.kStallCurrentAmps`
  for at least `kStallPersistenceSeconds`.

If you ever need to lock out the auto-completion (for testing or a
manual climb), call `disable()` on the subsystem.

## Zero calibration

A `zero()` command — usually bound to a long-press operator combo —
runs the motor slowly *down* into the hard stop until current spikes.
Once the motor stops moving (and stalls), the encoder zero is reset.

This is required after every code deploy because the motor uses a
relative encoder.

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
- **`DOWN → CLIMB` fires too early.** Stall persistence too short. Bump
  `kStallPersistenceSeconds` up — even 100 ms of confirmation prevents
  noise from latching the state.
- **Robot sags after `CLIMB`.** Brake mode isn't holding. Confirm the
  motor is set to brake in `ClimbIOSpark`.
