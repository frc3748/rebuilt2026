---
layout: default
title: Climb
eyebrow: Subsystem
description: One-motor elevator with time-of-flight rung sensing.
permalink: /subsystems/climb/
---

| | |
| --- | --- |
| **Source** | `src/main/java/frc/robot/subsystems/climb/` |
| **Public class** | [`Climb`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/subsystems/climb/Climb.java) extends `StateMachine<Climb.State>` |
| **Constants** | `ClimbConstants` |

The climb is the only mechanism with two independently controlled
motors (one per side) and is one of the few that uses an external
sensor — two time-of-flight beam-breakers to detect rung capture.

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

`CLIMB` engages brake mode on the motors to hold the robot's weight
without consuming amps.

## Beam-break sensors

Two TOF sensors detect whether the hooks have engaged the rung. The IO
interface is `BeamBreakerIO` with two implementations:

- **`BeamBreakerTOF`** — real hardware (Playing with Fusion or similar).
- **`BeamBreakerSim`** — sim, driven by `FuelSim`/`SimulatedRobotState`.

Both sensors must report tripped for the `DOWN → CLIMB` transition to
complete.

## Zero calibration

A `zero()` command — usually bound to a long-press operator combo —
runs both motors slowly *down* into the hard stop until current
spikes. Once both motors stop moving, the encoder zero is reset.

This is required after every code deploy because the motors are
relative encoders.

## Mechanism

- NEOs in brake mode when the elevator is stationary.
- **No common shaft** — each side moves independently; the
  state-machine sets matched setpoints, but mechanical binding ensures
  they stay aligned.
- **Hard stops** — top and bottom of travel, used for zeroing.

## Auto-climb

[`ActionCommands.autoClimb(RobotState)`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/commands/ActionCommands.java)
composes drive + climb:

1. Drive auto-aligns to the configured climb pose.
2. Climb transitions `STOW → UP`.
3. Drive holds position while operator manually triggers `UP → DOWN`.
4. Climb auto-completes `DOWN → CLIMB` when both beam-breaks trip.

## Logging

The climb publishes a 3D pose for the elevator extent and a separate
pose per side, so AdvantageScope can render the climb deployment over
time.

## Pitfalls

- **One side runs ahead.** Mechanical bind. Don't fight it in
  software — fix the linkage.
- **`DOWN → CLIMB` never fires.** A beam-break isn't triggering. Plot
  `Climb/leftBreak` and `Climb/rightBreak` in AdvantageScope; check
  power and CAN to the TOF sensors.
- **Robot sags after `CLIMB`.** Brake mode isn't holding. Confirm both
  motors are set to brake in `ClimbIOSpark`.
