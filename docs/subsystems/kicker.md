---
layout: default
title: Kicker
eyebrow: Subsystem
description: A short final-stage roller that pushes the piece the last inch into the flywheel.
permalink: /subsystems/kicker/
---

| | |
| --- | --- |
| **Source** | `src/main/java/frc/robot/subsystems/kicker/` |
| **Public class** | [`Kicker`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/subsystems/kicker/Kicker.java) extends `StateMachine<Kicker.State>` |
| **Constants** | `KickerConstants` |

The kicker is the simplest powered mechanism on the robot: one motor,
three states, no PID. Its purpose is to decouple "piece ready" from
"piece launched" so the shooter can wait for spin-up without
prematurely feeding the flywheel.

## States

```java
enum State { IDLE, SHOOT, OUTAKE }
```

- `IDLE` — motor off. Piece sits at the kicker, ready.
- `SHOOT` — motor at full forward voltage; piece transfers into the flywheel.
- `OUTAKE` — motor reversed; piece backs out into the hopper or out the intake.

## Why it exists as a separate subsystem

You could absorb this into the hopper. It's separate because:

- **Timing.** The shooter needs to launch the piece on a specific
  frame, not "when the conveyor reaches it." Holding the piece at the
  kicker means launch latency is bounded by motor inertia, not
  conveyor travel time.
- **State clarity.** Logs show exactly when launch happened (`SHOOT`
  entered).
- **Independent testing.** The mechanism can be characterized and
  tuned without spinning the rest of the indexing chain.

## Mechanism

A single NEO 550 driving a short roller. Open-loop voltage in all
states.

## Pitfalls

- **Piece won't transfer.** Increase `KickerConstants.kShootVolts`.
  Check the roller isn't slipping on the piece.
- **Double-feeds.** `SHOOT` is held too long. The shooter should
  return the kicker to `IDLE` as soon as the shoot timeout fires. Verify the shooter's `SHOOTING` state command exits.
