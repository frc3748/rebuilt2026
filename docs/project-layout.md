---
layout: default
title: Project Layout
eyebrow: Tutorial
description: A map of the repository — what lives where and why.
permalink: /project-layout/
---

A bird's-eye view of where everything lives.

## Top level

```
rebuilt2026/
├── build.gradle                ← GradleRIO build, AdvantageKit annotation processor
├── settings.gradle             ← Project name
├── gradle/, gradlew(.bat)      ← Gradle wrapper
├── vendordeps/                 ← External library JSONs (Phoenix, REVLib, …)
├── src/
│   └── main/
│       ├── deploy/             ← Static files copied to /home/lvuser/deploy
│       └── java/frc/robot/     ← All robot code (see below)
├── .wpilib/                    ← WPILib team-number / language preferences
└── docs/                       ← This documentation site
```

## `src/main/java/frc/robot/`

```
frc/robot/
├── Main.java                   ← JVM entry point; calls RobotBase.startRobot(Robot::new)
├── Robot.java                  ← Extends LoggedRobot; sets up logging and lifecycle hooks
├── RobotState.java             ← Top-level state machine; owns every subsystem and the controller bindings
├── Constants.java              ← Mode (REAL / SIM / REPLAY), picked automatically from RobotBase.isReal()
│
├── subsystems/
│   ├── drive/                  ← Swerve drive (AdvantageKit template, per-module IO)
│   ├── vision/                 ← Any number of cameras feeding the pose estimator
│   │   ├── Vision.java               ← StateMachine; loops over its cameras
│   │   ├── Camera.java               ← Vendor-neutral filtering, weighting, and object projection
│   │   ├── CameraConfig.java         ← Name, network name, robot→camera transform, std-dev factor
│   │   ├── CameraIO.java             ← Interface plus PoseObservation / ObjectObservation records
│   │   ├── CameraIOLimelight.java    ← Limelight MegaTag 1 + 2
│   │   ├── CameraIOPhoton.java       ← PhotonVision multi-tag + heading-seeded solve
│   │   ├── CameraIOPhotonSim.java    ← PhotonVision simulation on top of CameraIOPhoton
│   │   └── VisionConstants.java      ← Camera list, std-dev tuning, field geometry
│   ├── shooter/                ← Composite: hood + flywheel (the shooter is fixed to the chassis)
│   │   ├── Shooter.java              ← Orchestrates hood, flywheel, hopper, kicker
│   │   ├── ShooterConstants.java     ← Distance → shot map and time-of-flight map
│   │   ├── hood/                     ← {Hood, HoodConstants}
│   │   └── flywheel/                 ← {Flywheel, FlywheelConstants}
│   ├── intake/                 ← {Intake, IntakeConstants}
│   ├── hopper/                 ← {Hopper, HopperConstants}
│   ├── kicker/                 ← {Kicker, KickerConstants}
│   └── climb/                  ← {Climb, ClimbConstants, BeamBreakerIO, BeamBreakerTOF}
│
├── commands/
│   ├── DriveCommands.java            ← Default drive command + characterization
│   ├── ActionCommands.java           ← High-level "do the thing" composites (aim, shoot, climb)
│   ├── AutoCommands.java             ← AutoClass base (build, afterAuto) and the auto registry
│   ├── AutoAlignToPoseCommand.java   ← Profiled-PID pose alignment
│   └── autos/Autos.java              ← Every chooser auto, written with small step helpers
│
└── util/
    ├── motor/                  ← One motor abstraction shared by every mechanism
    │   ├── MotorConfig.java          ← CAN ID, controller type, gains, limits (fluent)
    │   ├── SpinMotor.java            ← Velocity-controlled motor: set(speed)
    │   ├── PosMotor.java             ← Position-controlled motor: set(position)
    │   ├── Motor.java                ← Shared base: picks the IO for the current Mode, logs, sends once per loop
    │   ├── MotorIO.java              ← Interface (@AutoLog inputs)
    │   ├── MotorIOSpark.java         ← Spark MAX / Spark Flex
    │   └── MotorIOSim.java           ← Kinematic sim (tracks setpoints)
    ├── state/                  ← The state-machine framework
    ├── TunableNumber.java            ← Dashboard-tunable constant (used in *Constants files)
    ├── GetTuned.java                 ← Ad-hoc dashboard-tunable lookups
    ├── ShooterSetpoint.java          ← Distance-aware shooter solutions
    ├── ShotCalculator.java           ← Projectile-motion shot math
    ├── ShotVisualizer.java           ← 3D trajectory logging
    ├── BallTargetFactory.java, PassTargetFactory.java  ← Hub and pass targets
    ├── TrenchZone.java               ← Trench proximity checks
    ├── DynamicPathGenerator.java, CustomAutoBuilder.java
    ├── SimulatedRobotState.java, FuelSim.java
    ├── LimelightHelpers.java, Elastic.java, SparkUtil.java
    └── ConcurrentTimeInterpolatableBuffer.java, GeomUtil.java, MathHelpers.java, RobotTime.java, Util.java
```

## The mental model

Three boxes nest inside each other:

```
┌─────────────────────────────────────────────────────────────┐
│  Robot          (lifecycle hooks, logger setup)             │
│  ┌───────────────────────────────────────────────────────┐  │
│  │  RobotState   (top-level state machine, ownership)    │  │
│  │  ┌─────────────────────────────────────────────────┐  │  │
│  │  │  Subsystems (Drive, Vision, Shooter, …)        │  │  │
│  │  │  ─ each extends StateMachine<E>                │  │  │
│  │  │  ─ each owns Motors (Spark, Sim, or Stub IO)   │  │  │
│  │  └─────────────────────────────────────────────────┘  │  │
│  └───────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

- `Robot` runs the WPILib loop and the AdvantageKit `Logger`.
- `RobotState` is the only object that knows about *every* subsystem.
- A subsystem only knows about its own state, its own IO, and (sometimes) its child subsystems.

## File conventions

- **`<Thing>.java`** — the `StateMachine` subclass. Holds a `SpinMotor` or `PosMotor` per physical motor and says what each state does in `applyState()`.
- **`<Thing>Constants.java`** — a `MotorConfig` per motor, `TunableNumber` setpoints, and geometry.
- **`util/motor/`** — the only place that talks to REV hardware or the simulator.
- **`Camera*.java`** — the vision equivalent: a `CameraConfig` per camera and one `CameraIO` per vendor.

Adding a mechanism is two files: a constants file with its motors, and a state machine that says what each state does.
