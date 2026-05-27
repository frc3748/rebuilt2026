---
layout: default
title: Hopper
eyebrow: Subsystem
description: Internal conveyor that carries pieces from intake to shooter. Jam-aware.
permalink: /subsystems/hopper/
---

| | |
| --- | --- |
| **Source** | `src/main/java/frc/robot/subsystems/hopper/` |
| **Public class** | [`Hopper`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/subsystems/hopper/Hopper.java) extends `StateMachine<Hopper.State>` |
| **Constants** | `HopperConstants` |

## States

```java
enum State { UNDETERMINED, IDLE, SHOOT, OUTAKE }
```

- `IDLE` — motor off, just holds the piece.
- `SHOOT` — runs forward, feeding the kicker.
- `OUTAKE` — runs in reverse to eject.

## Jam detection

The hopper watches its motor current. If current stays above
`HopperConstants.kJamCurrentAmps` for longer than
`kJamPersistenceSeconds`, the subsystem briefly transitions to
`OUTAKE` to clear, then back to `SHOOT`.

This logic lives in `Hopper#periodic()`:

```java
if (state == State.SHOOT &&
    inputs.currentAmps > kJamCurrentAmps &&
    jamTimer.hasElapsed(kJamPersistenceSeconds)) {
  requestTransition(State.OUTAKE);
  recoveryTimer.reset();
}
```

The auto-recovery means the operator usually doesn't need to think
about jams — they just hear the motor stutter and the piece comes
through.

## Coordination with the shooter

`Hopper` doesn't decide when to shoot — it just exposes the `SHOOT`
state. The `Shooter` parent enters `SHOOTING` only once turret, hood,
and flywheel report ready, *then* requests `Hopper.SHOOT`. The two
machines run independently and the parent orchestrates the timing.

## Mechanism

- Single NEO 550 driving a polycord conveyor.
- No sensor on the piece itself — jam detection relies on current.

## Logging

The hopper publishes its conveyor angle (cumulative spin) for
mechanism visualization in AdvantageScope.

## Pitfalls

- **`SHOOT` reverses for no reason.** Jam detection threshold is too
  low. Bump `HopperConstants.kJamCurrentAmps`.
- **Real jams don't recover.** Threshold is too high or persistence
  too long. Watch the `Hopper/current` plot in AdvantageScope during a
  real jam to find the right number.
