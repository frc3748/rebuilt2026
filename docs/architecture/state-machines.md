---
layout: default
title: State Machines
eyebrow: Architecture
description: Every subsystem is a state machine. Transitions are commands in a directional graph.
permalink: /architecture/state-machines/
---

If the [IO pattern]({{ '/architecture/io-pattern/' | relative_url }})
isolates hardware, the **state-machine pattern** organizes behavior.
Every subsystem in the codebase extends
[`StateMachine<E extends Enum<E>>`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/state/StateMachine.java).

## The model in one paragraph

A subsystem has an `enum State`. Two states are connected by a
**transition**, which is a `Command` that runs once to move the
subsystem from one to the other. While the subsystem sits in a state,
its **state command** runs continuously. Driver code never sets
a state directly — it calls `requestTransition(State.SHOOTING)`, and
the machine handles the rest.

```
            transition Command (one-shot)
   STATE A ─────────────────────────────▶ STATE B
                                          │
                                          ▼
                              state command for STATE B
                              (runs while in this state)
```

## The vocabulary

<dl>
  <dt>State</dt>
  <dd>An enum value. Every machine has at least an <code>UNDETERMINED</code> state, which represents "we don't know where the mechanism physically is yet."</dd>

  <dt>Transition</dt>
  <dd>A directed edge in the state graph. Backed by a <code>Command</code> that runs once when the edge is traversed. Most transitions are instantaneous; some (like climb deployment) take seconds.</dd>

  <dt>State command</dt>
  <dd>A long-running <code>Command</code> registered for a state. Starts when the state is entered, ends when the state is exited.</dd>

  <dt>Omni-transition</dt>
  <dd>A transition from <em>every</em> state to a target. Used for "emergency" states like <code>IDLE</code> that you always want to be able to reach.</dd>

  <dt>Flag</dt>
  <dd>A boolean attached to the state machine that is independent of the primary state. Used for side-channels — e.g. "are we currently allowed to shoot?" — without proliferating states.</dd>
</dl>

## The minimum viable state machine

Building one looks like this:

```java
public class Kicker extends StateMachine<Kicker.State> {
  public enum State { UNDETERMINED, IDLE, SHOOT, OUTAKE }

  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  public Kicker(KickerIO io) {
    super("Kicker", State.UNDETERMINED, State.class);
    this.io = io;

    registerStateCommand(State.IDLE,    run(() -> io.setVoltage(0)));
    registerStateCommand(State.SHOOT,   run(() -> io.setVoltage(KickerConstants.kShootVolts)));
    registerStateCommand(State.OUTAKE,  run(() -> io.setVoltage(-KickerConstants.kShootVolts)));

    addOmniTransitions(State.IDLE, State.SHOOT, State.OUTAKE);
  }

  @Override
  public void update() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
    super.update();
  }
}
```

That's it. To use it:

```java
kicker.requestTransition(Kicker.State.SHOOT);
```

## Public API of `StateMachine`

The members you'll touch from outside a subsystem:

| Method | What it does |
| --- | --- |
| `requestTransition(State)` | Request a transition; runs on the next periodic. |
| `transitionCommand(State)` | Returns a `Command` that requests the transition and waits until it lands. Useful for `onTrue()` bindings. |
| `getState()` | Current state. |
| `isDetermined()` | `false` only while in `UNDETERMINED`. |
| `isTransitioning()` | `true` while a transition command is mid-flight. |
| `enable()` / `disable()` | Globally gate transitions. `disabledInit()` calls `disable()` on all. |
| `setFlag(name)` / `getFlag(name)` | Side-channel booleans. |

And the ones a subclass uses when *building* its graph:

| Method | What it does |
| --- | --- |
| `addTransition(from, to, Command)` | One edge. |
| `addOmniTransitions(states…)` | Edges from *every* state into each given state. |
| `registerStateCommand(state, Command)` | Long-running behavior while in `state`. |
| `addChildSubsystem(StateMachine)` | Hierarchical composition. Parent's `update()` ticks children. |

## The transition graph

Internally, edges live in a
[`DirectionalEnumGraph<E, TransitionBase<E>>`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/state/graph/DirectionalEnumGraph.java).
When you call `requestTransition(target)`, the machine asks the graph
for an edge from the current state to the target. If it exists, the
edge's command is scheduled; if not, the request is logged and dropped.

This means an invalid transition is **safe** — it doesn't crash, it
just doesn't happen. You'll see the rejected request in the log.

> **Why a graph instead of nested if/else?**
> The graph form makes the entire state machine inspectable. The
> dashboard exposes a state chooser per subsystem, so during testing
> you can force the machine into any state for which a transition is
> defined — no code change needed.

## Hierarchical composition

The shooter is the showcase example. `Shooter` extends `StateMachine<Shooter.State>`
and owns three child machines: `Turret`, `Hood`, `Flywheel`. Each
child is added with `addChildSubsystem()`. When `Shooter` enters
`HUB_TRACKING`, its state command requests `HUB_TRACKING` on each
child. Each child runs its own state command independently.

This means the shooter's state machine doesn't have to know about
turret PID — it just orchestrates intent.

## Logging

Every machine auto-logs to AdvantageKit:

- `…/CurrentState`
- `…/DesiredState`
- `…/IsTransitioning`
- `…/Flags/<flag-name>`

In AdvantageScope, plot these and you'll see exactly what the
subsystem was trying to do, frame by frame.

## Patterns you'll see across subsystems

- **`UNDETERMINED` at boot.** Most subsystems wait until they have a
  zero or a homing position before allowing transitions. Until then
  they refuse them.
- **`IDLE` is always an omni-target.** You can always stop a mechanism.
- **State commands hold setpoints.** A "tracking" state's command is
  often `run(() -> io.setPosition(supplier.get()))` — the supplier is
  the actual control loop.
- **Transition commands wait on physical conditions.** A climb
  deployment transition might `waitUntil(beamBreak::isTripped)` before
  declaring victory.
