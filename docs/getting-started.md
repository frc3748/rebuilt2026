---
layout: default
title: Getting Started
eyebrow: Tutorial
description: Clone the repo, set up WPILib, deploy to the roboRIO, and run the simulator.
permalink: /getting-started/
---

This page walks you from a fresh checkout to a robot you can drive — both in
the simulator and on the real hardware.

## Prerequisites

You will need:

- **WPILib 2026** — install the [WPILib release](https://github.com/wpilibsuite/allwpilib/releases) for your platform. This installs a managed JDK 17, VS Code, and the FRC tooling. Do not try to mix in a system JDK; the GradleRIO toolchain expects WPILib's bundled one.
- **Git** — any recent version.
- **AdvantageScope** — for replaying logs and visualizing the field. Ships with WPILib but also available [standalone](https://github.com/Mechanical-Advantage/AdvantageScope).
- *(optional)* **Elastic** — driver-facing dashboard. The robot pushes toasts to it via [`Elastic`]({{ '/utilities/elastic/' | relative_url }}).

## Clone the repository

```bash
git clone https://github.com/frc3748/rebuilt2026.git
cd rebuilt2026
```

Open the folder in **WPILib VS Code**. The first time you open it, the
`Java` and `Gradle for Java` extensions will index — wait for the bottom
status bar to settle before doing anything else, otherwise IntelliSense
will give wrong answers about AdvantageKit's generated classes.

## Build

```bash
./gradlew build
```

The first build downloads:

1. The WPILib 2026 SDK
2. Every vendor dep listed in [`vendordeps/`]({{ '/reference/vendor-deps/' | relative_url }}) — PathPlanner, AdvantageKit, REVLib, Phoenix 5+6, PhotonLib, DogLog, MapleSim, etc.
3. The AdvantageKit annotation processor — this generates the `*AutoLogged` classes you'll see referenced everywhere (e.g. `ModuleIOInputsAutoLogged`).

If the build fails on the first run, run it again — vendor downloads
sometimes time out.

## Deploy to the robot

With a roboRIO connected (USB, tethered Ethernet, or radio):

```bash
./gradlew deploy
```

GradleRIO handles the artifact, the `/home/lvuser/deploy` static files,
and a restart of the robot program.

## Run the simulator

```bash
./gradlew simulateJava
```

This launches the desktop sim. You should see:

- A **Sim GUI** window with joystick mappings and a field display.
- An **AdvantageScope-compatible NetworkTables stream** — connect
  AdvantageScope to `localhost` to watch state and 3D field telemetry
  in real time.
- A simulated game-piece physics layer powered by
  [`FuelSim`]({{ '/utilities/simulation/' | relative_url }}). Balls
  spawn at the field intake locations and interact with the simulated
  intake/hopper geometry.

The `robotState` field in [`RobotState`]({{ '/architecture/robot-state/' | relative_url }})
controls which IO implementations are wired up:

- `1` — real hardware (only valid on the roboRIO).
- `2` — simulator. Hardware calls are replaced by physics-based sim implementations.
- Anything else — replay mode. IO does nothing, expecting inputs to be replayed from a log.

## Your first change

The fastest "did I install everything right?" check:

1. Open `src/main/java/frc/robot/Robot.java`.
2. Find `robotPeriodic()`.
3. Add a `Logger.recordOutput("HelloWorld", 42);` line.
4. Run `./gradlew simulateJava`.
5. In AdvantageScope, find `HelloWorld` under `NT/AdvantageKit/RealOutputs`.

If you see the value, you're good — your toolchain is wired correctly.

## Next steps

- Read [Robot Lifecycle]({{ '/architecture/robot-lifecycle/' | relative_url }}) — how `Main`, `Robot`, and `RobotState` fit together.
- Skim [The IO Layer Pattern]({{ '/architecture/io-pattern/' | relative_url }}) — the single most important pattern in the codebase.
- Skim [State Machines]({{ '/architecture/state-machines/' | relative_url }}) — the second most important.
- Then pick a subsystem and read it: [Drive]({{ '/subsystems/drive/' | relative_url }}) is a good first one because every other subsystem mirrors its structure.
