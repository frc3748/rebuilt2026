---
layout: default
title: Simulation
eyebrow: Utilities
description: SimulatedRobotState, FuelSimulation and FuelSim — ground truth and game-piece physics in the desktop simulator.
permalink: /utilities/simulation/
---

The simulator can do more than run motors. Mechanisms simulate through
`MotorIOSim`, the drive through `ModuleIOSim`, cameras through
`CameraIOPhotonSim`, and on a robot with a shooter or intake the fuel
itself is simulated: pieces sit on the field, get picked up by the
intake, and fly when the shooter fires.

## Which robot?

The simulator runs `Constants.kRobot` (see
[Multiple Robots]({{ '/architecture/robots/' | relative_url }})). The
practice robot simulates a drivetrain only.

## `SimulatedRobotState`

[`SimulatedRobotState`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/SimulatedRobotState.java)
holds the ground-truth robot pose. `Drive` feeds it every loop in
simulation, and `CameraIOPhotonSim` renders AprilTags from it.
`RobotState#getSimRobot()` returns it, or `null` on a real robot.

## `FuelSimulation`

[`FuelSimulation`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/game/FuelSimulation.java)
is the robot-facing wrapper. The
[`Superstructure`]({{ '/architecture/robots/' | relative_url }}#superstructure)
creates one in `SIM` mode when the first shooter or intake is added,
from the drive's `DriveConfig` (frame size and bumper height), the
shooter position in the robot's `ShooterConstants`, and an intake box in
front of the robot. The field starts with its usual fuel and the robot
starts holding 8 pieces.

- **Pickup** — while the intake is in `INTAKE`, pieces inside the intake box are counted as held.
- **Launch** — while the shooter `isFiring()` (`SHOOTING` or `PASSING`), the `Superstructure` calls `fuel.launch(exitVelocity, launchAngle)` every `ShooterConstants.simSecondsBetweenShots`, from the pass setpoint if `isPassing()` and the hub setpoint otherwise. It returns `false` when the robot holds no fuel.
- **Update** — `Superstructure.simulationPeriodic()` runs the sim shots and the `ShotVisualizer`, then `fuel.update()`. It is reached through `Robot.simulationPeriodic()` → `RobotState.updateSimulation()`. `ShotVisualizer` logs `Shooter/Trajectory` while the shooter is firing and clears it when it stops.

A **Reset Fuel** dashboard button clears the field and respawns the
starting fuel.

## `FuelSim`

[`FuelSim`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/game/FuelSim.java)
is the physics engine underneath: gravity, air resistance, field and
hub collisions, hub scoring, and intake boxes. It publishes
every piece as a `Translation3d` array to `/Fuel Simulation/Fuels` in
NetworkTables for AdvantageScope.

## Running the simulator

Use **WPILib: Simulate Robot Code** from the command palette (see
[Getting Started]({{ '/getting-started/' | relative_url }})),
or:

```bash
./gradlew simulateJava
```

`build.gradle` also adds the **Sim Websockets Server** extension with
`defaultEnabled = false`, so it starts unticked in the Simulate Robot
Code dialog and `simulateJava` leaves it off. robotTools' sim driver
loads it to drive the simulator without a window; see
[Logging & robotTools]({{ '/architecture/logging-and-robottools/' | relative_url }}#opening-logs-in-robottools).
Sim logs go to `logs/` in the repo.

## What you see in AdvantageScope

With the right layout:

- The robot as a 3D model, with the intake, hopper and hood poses.
- Each fuel piece from `/Fuel Simulation/Fuels`.
- The predicted shot from `Shooter/Trajectory`.
- Each camera's pose and the tags it sees, under `Vision/<name>/CameraPose` and `Vision/<name>/Tags`.

This is enough to debug almost any indexing or shooting bug without
ever touching a real ball.

The mechanism poses and camera visuals go through `Visuals.record`, so
they only exist in simulation logs. See
[Logging & robotTools]({{ '/architecture/logging-and-robottools/' | relative_url }}#real-robot-vs-simulation).

## Pitfalls

- **Shooter never launches.** The robot holds no fuel. Drive over
  pieces with the intake in `INTAKE`, or press **Reset Fuel**.
- **Intake never picks up.** Check the intake box built in
  `Superstructure.createFuelSimulation()`.
- **Shots always miss.** Check `shooterToRobotCenter` in the robot's
  `ShooterConstants`; the sim launches from there.
