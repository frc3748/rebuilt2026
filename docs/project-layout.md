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
├── RobotState.java             ← Top-level state machine; owns every subsystem
│
├── subsystems/
│   ├── drive/                  ← Swerve drive
│   │   ├── Drive.java                ← StateMachine — public API
│   │   ├── DriveConstants.java       ← Geometry, gear ratios, CAN IDs, PID
│   │   ├── DriveIO.java              ← (drive-level IO interface)
│   │   ├── Module.java               ← Per-wheel wrapper
│   │   ├── ModuleIO.java             ← Per-wheel IO interface
│   │   ├── ModuleIOSpark.java        ← Real Spark Max impl
│   │   ├── ModuleIOSim.java          ← Physics sim impl
│   │   ├── GyroIO.java               ← Gyro IO interface
│   │   ├── GyroIOPigeon2.java        ← Real Pigeon 2 impl
│   │   └── SparkOdometryThread.java  ← High-rate odometry reader
│   │
│   ├── vision/                 ← AprilTag vision
│   ├── shooter/                ← Composite: turret + hood + flywheel
│   │   ├── Shooter.java
│   │   ├── turret/             ← {Turret, TurretIO, TurretIOSpark, TurretIOSim, TurretConstants}
│   │   ├── hood/               ← {Hood, HoodIO, HoodIOSpark, HoodIOSim, HoodConstants}
│   │   └── flywheel/           ← {Flywheel, FlywheelIO, FlywheelIOSpark, FlywheelIOSim, FlywheelConstants}
│   │
│   ├── intake/                 ← Pivoting intake
│   ├── hopper/                 ← Internal conveyor
│   ├── kicker/                 ← Final push to flywheel
│   └── climb/                  ← One-motor elevator climb
│
├── commands/
│   ├── DriveCommands.java            ← Characterization + driver-stick mapping
│   ├── ActionCommands.java           ← High-level "do the thing" composites
│   ├── AutoCommands.java             ← Registered autos
│   ├── AutoAlignToPoseCommand.java   ← Profiled-PID pose alignment
│   └── autos/
│       ├── Autos.java                ← Auto helper functions
│       └── AutosConstants.java       ← Path names, fixed waypoints
│
└── util/
    ├── state/                  ← The state-machine framework
    │   ├── StateMachine.java         ← Base class every subsystem extends
    │   ├── SubsystemManager.java     ← Registry; coordinates lifecycle
    │   ├── SubsystemManagerFactory.java
    │   ├── graph/
    │   │   ├── DirectionalEnumGraph.java
    │   │   └── EdgeType.java
    │   └── transitions/
    │       ├── TransitionBase.java
    │       └── CommandTransition.java
    │
    ├── GeomUtil.java                 ← Pose/Rotation helpers
    ├── MathHelpers.java              ← Zero constants, angle wrapping
    ├── RobotTime.java                ← Wraps Timer.getFPGATimestamp()
    ├── ConcurrentTimeInterpolatableBuffer.java  ← Thread-safe pose history
    ├── ShooterSetpoint.java          ← Distance-aware shooter solutions
    ├── TurretCalculator.java         ← Projectile-motion turret math
    ├── BallTargetFactory.java        ← Hub target 3D coords
    ├── PassTargetFactory.java        ← Teammate-pass target 3D coords
    ├── TrenchZone.java               ← Auto-intake zone detection
    ├── DynamicPathGenerator.java     ← On-the-fly PathPlanner paths
    ├── CustomAutoBuilder.java        ← Dashboard auto builder
    ├── SimulatedRobotState.java      ← Sim-only state mirror
    ├── FuelSim.java                  ← Game-piece physics
    ├── TurretVisualizer.java         ← 3D pose logging
    ├── LimelightHelpers.java         ← Limelight NT wrapper
    ├── GetTuned.java                 ← Dashboard-tunable constants
    ├── Elastic.java                  ← Driver notifications
    ├── SparkUtil.java                ← Spark Max config helpers
    ├── LatchedBoolean.java           ← Rising-edge detector
    ├── IPathCallback.java            ← Path-event callback interface
    └── Util.java                     ← Misc helpers
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
│  │  │  ─ each owns an IO (Spark or Sim or Stub)      │  │  │
│  │  └─────────────────────────────────────────────────┘  │  │
│  └───────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

- `Robot` runs the WPILib loop and the AdvantageKit `Logger`.
- `RobotState` is the only object that knows about *every* subsystem.
- A subsystem only knows about its own state, its own IO, and (sometimes) its child subsystems.

## File conventions

A few conventions appear over and over:

- **`<Thing>.java`** — the `StateMachine` subclass. Public API.
- **`<Thing>IO.java`** — interface defining hardware reads/writes.
- **`<Thing>IOSpark.java`** — real REV implementation.
- **`<Thing>IOSim.java`** — physics-based simulator implementation.
- **`<Thing>Constants.java`** — CAN IDs, gear ratios, PID gains, geometry.
- **`<Thing>IOInputs` (inner class, `@AutoLog`)** — fields auto-logged by AdvantageKit. The annotation processor generates `<Thing>IOInputsAutoLogged`.

Once you internalize this, the codebase becomes very predictable: every
subsystem has the same five files in the same order.
