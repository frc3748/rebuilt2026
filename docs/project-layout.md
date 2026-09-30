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
├── build.gradle                ← GradleRIO build, AdvantageKit annotation processor, JUnit (forkEvery = 1)
├── settings.gradle             ← Project name
├── gradle/, gradlew(.bat)      ← Gradle wrapper
├── vendordeps/                 ← External library JSONs (Phoenix, REVLib, Studica, …)
├── src/
│   ├── main/
│   │   ├── deploy/             ← Static files copied to /home/lvuser/deploy (PathPlanner paths)
│   │   └── java/frc/robot/     ← All robot code (see below)
│   └── test/java/frc/robot/    ← CompetitionRobotTest, PracticeRobotTest
├── .wpilib/                    ← WPILib team-number / language preferences
└── docs/                       ← This documentation site
```

## `src/main/java/frc/robot/`

```
frc/robot/
├── Main.java                   ← JVM entry point; calls RobotBase.startRobot(Robot::new)
├── Robot.java                  ← Extends LoggedRobot; detects the robot, sets up logging and lifecycle hooks
├── RobotState.java             ← Top-level state machine; builds drive, vision and superstructure, holds pose history
├── Controls.java               ← Driver/operator controllers, shared drive bindings, rumble
├── Constants.java              ← Mode (REAL / SIM / REPLAY) and kDefaultRobot
│
├── robots/                     ← Everything robot-specific (see Multiple Robots)
│   ├── RobotType.java                ← COMPETITION, PRACTICE; roboRIO serial detection
│   ├── RobotDefinition.java          ← name(), drive(), cameras(), createSuperstructure()
│   ├── Superstructure.java           ← subsystems(), autos(), bindControls(), simulationPeriodic()
│   ├── competition/                  ← CompetitionRobot, CompetitionDrive, CompetitionSuperstructure,
│   │   │                               ActionCommands
│   │   └── autos/                    ← CompetitionAuto, one file per auto, CustomAuto
│   └── practice/                     ← PracticeRobot, PracticeDrive
│
├── game/                       ← 2026-game code
│   ├── FieldConstants.java           ← Tag layout, hub, funnel and trench geometry
│   ├── AllianceFlip.java             ← Red/blue mirroring
│   ├── GameState.java                ← Match phase, hub active, won auto
│   ├── DashboardManager.java         ← Auto chooser, auto path preview, Game/* values
│   ├── ShooterSetpoint.java          ← Distance-aware shooter solutions
│   ├── ShotCalculator.java           ← Projectile-motion shot math
│   ├── ShotVisualizer.java           ← 3D trajectory logging
│   ├── BallTargetFactory.java, PassTargetFactory.java  ← Hub and pass targets
│   ├── TrenchZone.java               ← Trench proximity checks
│   └── FuelSim.java, FuelSimulation.java  ← Game-piece physics and its robot wrapper
│
├── subsystems/
│   ├── drive/                  ← Swerve drive (AdvantageKit template, per-module IO)
│   │   ├── Drive.java                ← StateMachine; pose estimator, PathPlanner setup
│   │   ├── DriveConfig.java          ← Per-robot drivetrain constants (subclassed per robot)
│   │   ├── GyroIO*.java              ← GyroIOPigeon2, GyroIONavX
│   │   └── ModuleIO*.java            ← ModuleIOSpark, ModuleIOSim
│   ├── vision/                 ← Any number of cameras feeding the pose estimator
│   │   ├── Vision.java               ← StateMachine; loops over its cameras
│   │   ├── Camera.java               ← Vendor-neutral filtering, weighting, and object projection
│   │   ├── CameraConfig.java         ← Name, network name, type, robot→camera transform, std-dev factor
│   │   ├── CameraIO.java             ← Interface plus PoseObservation / ObjectObservation records
│   │   ├── CameraIOLimelight.java    ← Limelight MegaTag 1 + 2
│   │   ├── CameraIOPhoton.java       ← PhotonVision multi-tag + heading-seeded solve
│   │   ├── CameraIOPhotonSim.java    ← PhotonVision simulation on top of CameraIOPhoton
│   │   └── VisionConstants.java      ← Std-dev baselines and rejection thresholds
│   ├── shooter/                ← Composite: hood + flywheel (the shooter is fixed to the chassis)
│   │   ├── Shooter.java              ← Orchestrates hood, flywheel, hopper, kicker
│   │   ├── ShooterConstants.java     ← Distance → shot map and time-of-flight map
│   │   ├── hood/                     ← {Hood, HoodConstants}
│   │   └── flywheel/                 ← {Flywheel, FlywheelConstants}
│   ├── intake/                 ← {Intake, IntakeConstants}
│   ├── hopper/                 ← {Hopper, HopperConstants}
│   └── kicker/                 ← {Kicker, KickerConstants}
│
├── commands/
│   ├── DriveCommands.java            ← Default drive command + characterization
│   ├── AutoAlignToPoseCommand.java   ← Profiled-PID pose alignment
│   └── autos/AutoRoutine.java        ← Base class for every auto
│
└── util/                       ← Generic, not tied to a robot or a game
    ├── motor/                  ← One motor abstraction shared by every mechanism
    │   ├── MotorConfig.java          ← CAN ID, controller type, gains, limits (fluent)
    │   ├── Gains.java                ← PID, feedforward, gravity and MAXMotion values
    │   ├── SpinMotor.java            ← Velocity-controlled motor: set(speed)
    │   ├── PosMotor.java             ← Position-controlled motor: set(position)
    │   ├── Motor.java                ← Shared base: picks the IO for the current Mode, logs, sends once per loop
    │   ├── MotorIO.java              ← Interface (@AutoLog inputs)
    │   ├── MotorIOSpark.java         ← Spark MAX / Spark Flex
    │   └── MotorIOSim.java           ← Kinematic sim (tracks setpoints)
    ├── state/                  ← The state-machine framework
    ├── TunableNumber.java            ← Dashboard-tunable constant (DogLog)
    ├── SparkUtil.java                ← Spark error checks and tune(), the live-gains helper
    ├── ConcurrentTimeInterpolatableBuffer.java, RobotTime.java
    ├── SimulatedRobotState.java      ← Ground-truth pose in simulation
    └── LimelightHelpers.java, Elastic.java
```

## The mental model

Three boxes nest inside each other:

```
┌─────────────────────────────────────────────────────────────┐
│  Robot          (lifecycle hooks, logger setup)             │
│  ┌───────────────────────────────────────────────────────┐  │
│  │  RobotState   (built from a RobotDefinition)          │  │
│  │  ┌─────────────────────────────────────────────────┐  │  │
│  │  │  Drive, Vision + the Superstructure's machines │  │  │
│  │  │  ─ each extends StateMachine<E>                │  │  │
│  │  │  ─ each owns Motors (Spark, Sim, or Stub IO)   │  │  │
│  │  └─────────────────────────────────────────────────┘  │  │
│  └───────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

- `Robot` runs the WPILib loop and the AdvantageKit `Logger`.
- `RobotState` builds the shared drive and vision and asks the robot's `Superstructure` for the rest.
- Code that only one robot uses lives in `robots/<robot>/`; 2026-game code lives in `game/`; `util/` stays generic.
- A subsystem only knows about its own state, its own IO, and (sometimes) its child subsystems.

## File conventions

- **`<Thing>.java`** — the `StateMachine` subclass. Holds a `SpinMotor` or `PosMotor` per physical motor and says what each state does in `applyState()`.
- **`<Thing>Constants.java`** — a `MotorConfig` per motor, `TunableNumber` setpoints, and geometry.
- **`util/motor/`** — the only place mechanisms talk to REV hardware or the simulator (the drive has its own `ModuleIO`).
- **`Camera*.java`** — the vision equivalent: a `CameraConfig` per camera and one `CameraIO` per vendor.

Adding a mechanism is two files: a constants file with its motors, and a state machine that says what each state does. Then build it in a robot's `Superstructure` and return it from `subsystems()`.
