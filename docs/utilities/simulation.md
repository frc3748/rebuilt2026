---
layout: default
title: Simulation
eyebrow: Utilities
description: SimulatedRobotState, FuelSim — game-piece physics in the desktop simulator.
permalink: /utilities/simulation/
---

The simulator can do more than run motors. With these two utilities,
it also simulates *game pieces*: the balls spawn at intake locations,
fall under gravity, interact with the simulated intake geometry, and
get logged as 3D poses for AdvantageScope.

## `SimulatedRobotState`

[`SimulatedRobotState`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/SimulatedRobotState.java)
mirrors the live `RobotState` in simulation. It tracks:

- Simulated game pieces in the world (position, velocity, rotation).
- Whether the intake "sees" a piece (used by `IntakeIOSim` to fake
  the beam-break input).
- Whether the hopper currently holds a piece.

This is consulted by every `*IOSim` implementation that needs to know
about pieces — e.g., `IntakeIOSim.updateInputs` sets `inputs.pieceDetected`
from `SimulatedRobotState.intakeHasPiece()`.

## `FuelSim`

[`FuelSim`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/FuelSim.java)
is the physics engine for game pieces. It runs in
`Robot#simulationPeriodic` and does the following each loop:

1. **Spawn** pieces at field intake locations on a configurable rate.
2. **Integrate** their motion under gravity + drag.
3. **Test for pickup** by the simulated intake (geometric overlap with
   the intake bounding volume).
4. **Test for ejection** when the shooter fires (a launched piece
   gets velocity from the flywheel + hood angle).
5. **Publish** every piece's 3D pose to AdvantageKit for visualization.

### Registering the robot

Once at startup:

```java
FuelSim.getInstance().registerRobot(robotState);
FuelSim.getInstance().registerIntakeGeometry(intakeSim);
```

After registration, the simulator knows enough geometry to detect
interactions automatically.

## Running the simulator

```bash
./gradlew simulateJava
```

Hot-reload doesn't work (it's a JVM relaunch each time) but the
turnaround is fast — most logic changes are 5-10 second iterations.

## What you see in AdvantageScope

With the right layout:

- The robot as a 3D model.
- Each game piece as a yellow sphere.
- Pieces inside the hopper follow the robot.
- Pieces in flight trace a parabolic arc to (hopefully) the target.

This is enough to debug almost any indexing or shooting bug without
ever touching a real ball.

## Tuning the physics

A few `GetTuned` knobs:

| Key | Default | What it does |
| --- | --- | --- |
| `Sim/Gravity` | -9.81 m/s² | Vertical gravity. |
| `Sim/Drag` | small | Linear drag coefficient. |
| `Sim/SpawnRate` | 1 Hz | How often new pieces appear at intake stations. |
| `Sim/MaxPieces` | 20 | Cap on simultaneous pieces. |

Don't expect bit-for-bit accuracy — it's a sim. But it's a sim that
exercises the *codepath* of a shot, which is what matters for finding
bugs.

## Pitfalls

- **Pieces fall through the floor.** Drag/gravity mismatch with the
  step size. Reset and try again — usually transient.
- **Intake never sees a piece.** Intake geometry isn't registered, or
  the bounding volume is wrong. Check `FuelSim.registerIntakeGeometry`
  args.
- **Shots always miss.** Verify the turret-to-robot transform matches
  what real hardware uses — the sim uses the same `VisionConstants`.
