---
layout: default
title: Glossary
eyebrow: Reference
description: Terms that appear throughout the codebase and what they mean.
permalink: /reference/glossary/
---

<dl>

<dt>AdvantageKit</dt>
<dd>Littleton Robotics' logging library. Records every IO read so you can replay a match offline.</dd>

<dt>AdvantageScope</dt>
<dd>The visualization tool for AdvantageKit logs. 2D/3D field view, time-series plots, mechanism visualization.</dd>

<dt>Auto / Autonomous</dt>
<dd>The 15-second period at match start where the robot runs without driver input.</dd>

<dt>AutoLog</dt>
<dd>An annotation provided by AdvantageKit. Mark an inputs class with <code>@AutoLog</code> and the annotation processor generates a serializable subclass (<code>FooInputsAutoLogged</code>) that <code>Logger.processInputs</code> knows how to record and replay.</dd>

<dt>CAN</dt>
<dd>Controller Area Network. The bus that connects every motor controller and most sensors. The roboRIO has one bus; a CANivore adds a second.</dd>

<dt>CANcoder</dt>
<dd>CTRE's absolute magnetic encoder. Used on swerve modules for the turn axis.</dd>

<dt>CommandScheduler</dt>
<dd>WPILib's central loop runner. Calls every subsystem's <code>periodic()</code> and runs any active commands.</dd>

<dt>Controls</dt>
<dd>The class that owns the driver and operator Xbox controllers, every binding, and rumble. Shared by every robot; a robot can override one binding in a subclass. See <a href="{{ '/architecture/robot-state/' | relative_url }}#controls">RobotState</a>.</dd>

<dt>DogLog</dt>
<dd>A lightweight tunable-constants and live-telemetry library. Wrapped by <a href="{{ '/utilities/tunable-number/' | relative_url }}"><code>TunableNumber</code></a>.</dd>

<dt>DriveConfig</dt>
<dd>A robot's drivetrain constants: gyro, motor controllers, turn sensor, CAN IDs, geometry, gains. One subclass per robot (<code>CompDrive</code>, <code>SecondaryDrive</code>, <code>PracticeDrive</code>). See <a href="{{ '/subsystems/drive/' | relative_url }}#driveconfig">Drive</a>.</dd>

<dt>Elastic</dt>
<dd>A driver dashboard. The codebase pushes toast notifications to it via <a href="{{ '/utilities/elastic/' | relative_url }}">the Elastic helper</a>.</dd>

<dt>FMS</dt>
<dd>Field Management System. The match controller at competition. Provides alliance, match time, and other match data.</dd>

<dt>Feedforward (FF)</dt>
<dd>An open-loop prediction added to PID output to compensate for known dynamics. The shooter uses FF for chassis-velocity compensation; the drive uses <code>kS</code>/<code>kV</code>/<code>kA</code>.</dd>

<dt>Flag</dt>
<dd>A side-channel boolean on a <code>StateMachine</code>, independent of the primary state. Set/queried via <code>setFlag</code>/<code>isFlag</code>.</dd>

<dt>FRC</dt>
<dd>FIRST Robotics Competition.</dd>

<dt>GradleRIO</dt>
<dd>The Gradle plugin that handles deployment to the roboRIO.</dd>

<dt>Hub</dt>
<dd>The primary scoring target this season.</dd>

<dt>IO Layer</dt>
<dd>The interface-based hardware abstraction pattern used throughout the codebase. See <a href="{{ '/architecture/io-pattern/' | relative_url }}">The IO Layer Pattern</a>.</dd>

<dt>Kraken / TalonFX</dt>
<dd>Brushless motors (Kraken, Falcon) with a built-in CTRE Talon FX controller. Any <code>MotorConfig</code> with <code>Controller.TALON_FX</code> runs through <code>MotorIOTalonFX</code>; swerve modules don't support it yet. See <a href="{{ '/architecture/io-pattern/' | relative_url }}">The IO Layer Pattern</a>.</dd>

<dt>Limelight</dt>
<dd>A networked camera with built-in AprilTag detection. The comp and secondary robots have two Limelight 4s — one on the fixed shooter, one on the chassis.</dd>

<dt>LimelightHelpers</dt>
<dd>A small NetworkTables wrapper that exposes Limelight reads as Java methods. Lives in <code>util/</code>.</dd>

<dt>LoggedRobot</dt>
<dd>AdvantageKit's subclass of <code>TimedRobot</code>. <code>Robot.java</code> extends it.</dd>

<dt>Megatag / Megatag2</dt>
<dd>Limelight's multi-tag pose solve. Megatag2 is the gyro-fused successor; the codebase prefers it where available.</dd>

<dt>NavX</dt>
<dd>Studica's IMU. The practice robot's gyro, on USB.</dd>

<dt>NEO / NEO 550</dt>
<dd>REV brushless motors. NEOs drive the wheels and flywheel; NEO 550s are used for lower-torque mechanisms.</dd>

<dt>Odometry</dt>
<dd>Estimating position from wheel encoders. The drive runs a high-rate <a href="{{ '/subsystems/drive/' | relative_url }}">odometry thread</a> on real hardware.</dd>

<dt>Omni-transition</dt>
<dd>A transition from <em>every</em> state to a single target. Used for "always-reachable" states like <code>IDLE</code>.</dd>

<dt>PathPlanner</dt>
<dd>The trajectory generator used for autos. Paths are authored in a GUI and stored as JSON in <code>src/main/deploy/pathplanner/</code>.</dd>

<dt>Pigeon 2</dt>
<dd>CTRE's IMU. Provides yaw, pitch, roll, and angular velocities. The comp and secondary robots' gyro, CAN ID 50.</dd>

<dt>PhotonVision / PhotonLib</dt>
<dd>An alternative vision pipeline. PhotonLib simulates every camera, and <code>CameraConfig.Type.PHOTON</code> is intended for real object-detection cameras this year.</dd>

<dt>Pose / Pose2d / Pose3d</dt>
<dd>A translation + rotation. <code>Pose2d</code> is field-plane (x, y, yaw); <code>Pose3d</code> adds z, pitch, roll.</dd>

<dt>roboRIO</dt>
<dd>The NI single-board computer that runs the robot code.</dd>

<dt>RobotDefinition / RobotType</dt>
<dd>How one codebase runs several robots. A <code>RobotDefinition</code> subclass describes one robot, usually by extending the closest robot and overriding only what differs; <code>RobotType</code> lists them (<code>COMP</code>, <code>SECONDARY</code>, <code>PRACTICE</code>) and <code>Constants.kRobot</code> picks one. See <a href="{{ '/architecture/robots/' | relative_url }}">Multiple Robots</a>.</dd>

<dt>SendableChooser</dt>
<dd>A WPILib widget that exposes a dropdown to the dashboard. Used for auto selection and per-subsystem state overrides.</dd>

<dt>Spark Max / Spark Flex</dt>
<dd>REV brushless motor controllers. Almost every motor on the robot is driven by one.</dd>

<dt>State Machine</dt>
<dd>The codebase's central organizing pattern. Every subsystem extends <code>StateMachine&lt;E&gt;</code>. See <a href="{{ '/architecture/state-machines/' | relative_url }}">State Machines</a>.</dd>

<dt>Subsystem</dt>
<dd>A coherent piece of the robot (drive, intake, shooter, …). Every subsystem here extends <code>StateMachine</code>, which itself extends WPILib's <code>SubsystemBase</code>.</dd>

<dt>SubsystemManager</dt>
<dd>A singleton registry that broadcasts lifecycle events (auto start, teleop start, disable) to every registered subsystem. See <a href="{{ '/architecture/subsystem-manager/' | relative_url }}">Subsystem Manager</a>.</dd>

<dt>Superstructure</dt>
<dd>The shared class that holds a robot's mechanisms (a shooter and an intake, if it has them), built by its <code>RobotDefinition</code>. Autos and bindings reach mechanisms through it, so they do nothing on a robot without that mechanism. The practice robot's is empty. See <a href="{{ '/architecture/robots/' | relative_url }}#superstructure">Multiple Robots</a>.</dd>

<dt>SysId</dt>
<dd>WPILib's system-identification framework. The drive has SysId routines for characterization.</dd>

<dt>Transition</dt>
<dd>A directed edge in a state-machine graph, backed by a <code>Command</code>. See <a href="{{ '/architecture/state-machines/' | relative_url }}">State Machines</a>.</dd>

<dt>Trench Zone</dt>
<dd>A region of the field where the intake auto-deploys. See <a href="{{ '/reference/field-constants/' | relative_url }}">Field Constants</a>.</dd>

<dt>WPILib</dt>
<dd>The FRC standard library — units, geometry, command framework, hardware abstractions.</dd>

<dt>WPILog</dt>
<dd>WPILib's binary log format. AdvantageKit reads and writes it.</dd>

</dl>
