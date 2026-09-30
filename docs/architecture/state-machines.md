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
  <dd>A directed edge in the state graph. Backed by a <code>Command</code> that runs once when the edge is traversed. Most transitions are instantaneous.</dd>

  <dt>State command</dt>
  <dd>A long-running <code>Command</code> registered for a state. Starts when the state is entered, ends when the state is exited.</dd>

  <dt>Omni-transition</dt>
  <dd>A transition from <em>every</em> state to a target. Used for "emergency" states like <code>IDLE</code> that you always want to be able to reach.</dd>

  <dt>Flag</dt>
  <dd>A boolean attached to the state machine that is independent of the primary state. Used for side-channels — e.g. "are we currently allowed to shoot?" — without proliferating states.</dd>
</dl>

## The periodic loop

Every `StateMachine` runs the same loop each robot cycle, in this order:

1. **Read.** Every device registered with `addHardware(...)` reads its sensors and logs its inputs.
2. **Update.** `update()` runs for anything that is not motor output: telemetry, stall detection, deciding to request another state.
3. **Apply the state.** If an override is set it runs; otherwise `applyState(getState())` runs. This is the only place a subsystem commands its motors.
4. **Apply constraints.** `applyConstraints()` runs last and wins over both the state and any override. Safety rules such as "lower the intake under the trench" live here.
5. **Write.** Every registered device sends its final command to the hardware once.

Because motors only send in step 5, a constraint can replace a state's command without two conflicting CAN writes in the same loop.

## A whole subsystem

```java
public class Intake extends StateMachine<Intake.State> {
    public enum State { UNDETERMINED, STOW, IDLE, INTAKE, OUTAKE, SHAKE }

    private final SpinMotor rollers = new SpinMotor(kRollers);
    private final PosMotor extension = new PosMotor(kExtension);

    public Intake(RobotState robotState) {
        super("Intake", State.UNDETERMINED, State.class);
        addHardware(rollers, extension);
        allowAllTransitions();
        enable();
    }

    @Override
    protected void applyState(State state) {
        switch (state) {
            case STOW -> goTo(kStowSetpoint.get(), 0);
            case IDLE -> goTo(kIntakeSetpoint.get(), 0);
            case INTAKE -> goTo(kIntakeSetpoint.get(), kIntakeRollerSpeed.get());
            ...
        }
    }

    @Override
    protected void applyConstraints() {
        if (TrenchZone.intakeLowerRequired(robotState)) {
            extension.set(kIntakeSetpoint.get());
        }
    }
}
```

To use it:

```java
intake.requestTransition(Intake.State.INTAKE);
```

## Overrides

Any machine can be overridden from the operator controller:

```java
intake.setOverride(Intake.State.STOW);   // behave like STOW no matter what is requested
intake.setOverride(intake::rollIn);      // run a custom action instead of applyState
intake.clearOverride();
```

The requested state keeps updating underneath, so clearing the override
drops straight back into whatever the drivers and autos asked for.
Constraints still apply while overridden.

## Public API of `StateMachine`

| Method | What it does |
| --- | --- |
| `requestTransition(State)` | Request a transition; lands on a following loop. |
| `transitionCommand(State)` | A `Command` that requests the transition and waits until it lands. |
| `transitionCommand(State, false)` | Requests the transition and finishes immediately. Autos use it to overlap steps. |
| `getState()` | Current state. |
| `isDetermined()` / `isTransitioning()` | Lifecycle checks. |
| `setOverride(State)` / `setOverride(Runnable)` / `clearOverride()` | Manual control. |

Used inside a subclass:

| Method | What it does |
| --- | --- |
| `addHardware(devices…)` | Register motors and sensors for the read and write steps. |
| `allowAllTransitions()` | Every state can be reached from every other state. |
| `addTransition(from, to, Runnable)` | One edge with an action that runs when it is traversed. |
| `registerStateCommand(state, Runnable)` | Runs once on entering a state. |
| `applyState(state)` | What the mechanism does in each state. |
| `applyConstraints()` | Rules that always win. |
| `update()` | Telemetry and self-transitions. |
| `addChildSubsystem(StateMachine)` | Hierarchical composition. |

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
and owns two child machines: `Hood` and `Flywheel`. Each
child is added with `addChildSubsystem()`. When `Shooter` enters
`HUB_TRACKING`, its state command requests `HUB_TRACKING` on each
child. Each child runs its own state command independently.

This means the shooter's state machine doesn't have to know about
hood PID — it just orchestrates intent.

## Logging

Every machine auto-logs to AdvantageKit:

- `<Name>/state` and `<Name>/desired`
- `<Name>/transitioning`
- `<Name>/flags`
- `<Name>/enabled` and `<Name>/overridden`

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
- **Autos chain transition commands.** `transitionCommand(state)`
  waits for the state to land, so `Commands.sequence(...)` of them
  runs step by step; pass `false` to fire and move on.
