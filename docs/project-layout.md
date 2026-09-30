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
│   └── test/java/frc/robot/    ← CompRobotTest, SecondaryRobotTest, PracticeRobotTest
├── .wpilib/                    ← WPILib team-number / language preferences
└── docs/                       ← This documentation site
```

## `src/main/java/frc/robot/`

```
frc/robot/
├── Main.java                   ← JVM entry point; calls RobotBase.startRobot(Robot::new)
├── Robot.java                  ← Extends LoggedRobot; builds RobotState for Constants.kRobot, sets up logging and lifecycle hooks
├── RobotState.java             ← Top-level state machine; builds the robot from its definition, holds pose history
├── Superstructure.java         ← Holds the robot's shooter and intake (if it has them); runs the fuel sim
├── Controls.java               ← Driver/operator controllers, every binding (protected, overridable), rumble
├── Constants.java              ← Mode (REAL / SIM / REPLAY) and kRobot
│
├── robots/                     ← One definition per robot (see Multiple Robots)
│   ├── RobotType.java                ← COMP, SECONDARY, PRACTICE
│   ├── RobotDefinition.java          ← Abstract: name(), drive(); defaults for shooter(), cameras(), createSuperstructure(), createControls(), autos()
│   ├── comp/                         ← CompRobot, CompDrive (the new chassis, future main robot)
│   ├── secondary/                    ← SecondaryRobot, SecondaryDrive (the current robot, built on comp)
│   └── practice/                     ← PracticeRobot, PracticeDrive (drivetrain only)
│
├── game/                       ← 2026-game code
│   ├── FieldConstants.java           ← Tag layout, hub, funnel and trench geometry
│   ├── AllianceFlip.java             ← Red/blue mirroring
│   ├── GameState.java                ← Match phase, hub active, won auto
│   ├── DashboardManager.java         ← Auto chooser, auto path preview, Game/* values
│   ├── ShooterSetpoint.java          ← Distance-aware shooter solutions
│   ├── ShotCalculator.java           ← Projectile-motion shot math, built from a robot's ShooterConstants
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
│   ├── shooter/                ← Composite (the shooter is fixed to the chassis)
│   │   ├── Shooter.java              ← Abstract base: states and the calls shared code makes
│   │   ├── ShooterComp.java          ← Comp shooter; owns hood, flywheel, hopper, kicker
│   │   ├── ShooterConstants.java     ← Shooter position, shot map and time-of-flight map
│   │   ├── hood/                     ← {Hood, HoodConstants}
│   │   └── flywheel/                 ← {Flywheel, FlywheelConstants}
│   ├── intake/                 ← {Intake (abstract base), IntakeComp, IntakeConstants}
│   ├── hopper/                 ← {Hopper, HopperConstants}
│   └── kicker/                 ← {Kicker, KickerConstants}
│
├── commands/                   ← Shared by every robot
│   ├── DriveCommands.java            ← Default drive command + characterization
│   ├── AutoAlignToPoseCommand.java   ← Profiled-PID pose alignment
│   ├── ActionCommands.java           ← Composite commands for buttons and autos
│   └── autos/                        ← AutoRoutine, PathAuto, Autos.all, one file per auto
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
    │   ├── MotorIOTalonFX.java       ← TalonFX (Kraken, Falcon) through Phoenix 6
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
│  │  │  Drive, Vision + the Superstructure's machines  │  │  │
│  │  │  ─ each extends StateMachine<E>                 │  │  │
│  │  │  ─ each owns Motors (Spark, TalonFX, Sim, Stub) │  │  │
│  │  └─────────────────────────────────────────────────┘  │  │
│  └───────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

- `Robot` runs the WPILib loop and the AdvantageKit `Logger`.
- `RobotState` builds the shared drive and vision and gets the robot's `Superstructure`, which holds its mechanisms, from its definition.
- A robot's definition lives in `robots/<robot>/`, and mechanism logic that differs between robots lives in a subsystem variant such as `ShooterComp`; 2026-game code lives in `game/`; `util/` stays generic.
- A subsystem only knows about its own state, its own IO, and (sometimes) its child subsystems.

## File conventions

- **`<Thing>.java`** — the `StateMachine` subclass. Holds a `SpinMotor` or `PosMotor` per physical motor and says what each state does in `applyState()`.
- **`<Thing><Robot>.java`** — one robot's variant of an abstract `<Thing>`, named subsystem first (`IntakeComp`, `ShooterComp`). The base holds the states; the variant holds the motors.
- **`<Thing>Constants.java`** — a plain class of public fields: a `MotorConfig` per motor, setpoint defaults, and geometry. Defaults are the comp values; a robot changes fields [per robot]({{ '/architecture/robots/' | relative_url }}#per-robot-constants).
- **`util/motor/`** — the only place mechanisms talk to REV or CTRE hardware or the simulator (the drive has its own `ModuleIO`).
- **`Camera*.java`** — the vision equivalent: a `CameraConfig` per camera and one `CameraIO` per vendor.

Adding a mechanism is two files: a constants class with its motors, and a state machine that takes it in its constructor and says what each state does. Then add it as a child of the machine that drives it, as `ShooterComp` does with the hopper and kicker, and give `CompRobot` a protected method that returns its constants, like `hopper()`, so another robot can override it. `Superstructure` only takes a shooter and an intake, so a new top-level mechanism also needs its own `with…` method there.
