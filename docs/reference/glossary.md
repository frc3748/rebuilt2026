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

<dt>Beam-break</dt>
<dd>A sensor that detects when an object breaks an IR beam. Used by climb to detect rung capture. In sim, faked via <code>SimulatedRobotState</code>.</dd>

<dt>BeamBreakerIO</dt>
<dd>The IO interface for beam-breaks. Real: <code>BeamBreakerTOF</code> (time-of-flight). Sim: <code>BeamBreakerSim</code>.</dd>

<dt>CAN</dt>
<dd>Controller Area Network. The bus that connects every motor controller and most sensors. The roboRIO has one bus; a CANivore adds a second.</dd>

<dt>CANcoder</dt>
<dd>CTRE's absolute magnetic encoder. Used on swerve modules for the turn axis.</dd>

<dt>CommandScheduler</dt>
<dd>WPILib's central loop runner. Calls every subsystem's <code>periodic()</code> and runs any active commands.</dd>

<dt>DogLog</dt>
<dd>A lightweight tunable-constants and live-telemetry library. Wrapped by <a href="{{ '/utilities/get-tuned/' | relative_url }}"><code>GetTuned</code></a>.</dd>

<dt>Elastic</dt>
<dd>A driver dashboard. The codebase pushes toast notifications to it via <a href="{{ '/utilities/elastic/' | relative_url }}">the Elastic helper</a>.</dd>

<dt>FMS</dt>
<dd>Field Management System. The match controller at competition. Provides alliance, match time, and other match data.</dd>

<dt>Feedforward (FF)</dt>
<dd>An open-loop prediction added to PID output to compensate for known dynamics. The shooter uses FF for chassis-velocity compensation; the drive uses <code>kS</code>/<code>kV</code>/<code>kA</code>.</dd>

<dt>Flag</dt>
<dd>A side-channel boolean on a <code>StateMachine</code>, independent of the primary state. Set/queried via <code>setFlag</code>/<code>getFlag</code>.</dd>

<dt>FRC</dt>
<dd>FIRST Robotics Competition.</dd>

<dt>GradleRIO</dt>
<dd>The Gradle plugin that handles deployment to the roboRIO.</dd>

<dt>Hub</dt>
<dd>The primary scoring target this season.</dd>

<dt>IO Layer</dt>
<dd>The interface-based hardware abstraction pattern used throughout the codebase. See <a href="{{ '/architecture/io-pattern/' | relative_url }}">The IO Layer Pattern</a>.</dd>

<dt>Limelight</dt>
<dd>A networked camera with built-in AprilTag detection. The robot has two — one on the turret, one on the chassis.</dd>

<dt>LimelightHelpers</dt>
<dd>A small NetworkTables wrapper that exposes Limelight reads as Java methods. Lives in <code>util/</code>.</dd>

<dt>LoggedRobot</dt>
<dd>AdvantageKit's subclass of <code>TimedRobot</code>. <code>Robot.java</code> extends it.</dd>

<dt>Megatag / Megatag2</dt>
<dd>Limelight's multi-tag pose solve. Megatag2 is the gyro-fused successor; the codebase prefers it where available.</dd>

<dt>NEO / NEO 550</dt>
<dd>REV brushless motors. NEOs drive the wheels and flywheel; NEO 550s are used for lower-torque mechanisms.</dd>

<dt>Odometry</dt>
<dd>Estimating position from wheel encoders. The drive runs a high-rate <a href="{{ '/subsystems/drive/' | relative_url }}">odometry thread</a> on real hardware.</dd>

<dt>Omni-transition</dt>
<dd>A transition from <em>every</em> state to a single target. Used for "always-reachable" states like <code>IDLE</code>.</dd>

<dt>PathPlanner</dt>
<dd>The trajectory generator used for autos. Paths are authored in a GUI and stored as JSON in <code>src/main/deploy/pathplanner/</code>.</dd>

<dt>Pigeon 2</dt>
<dd>CTRE's IMU. Provides yaw, pitch, roll, and angular velocities. CAN ID 50.</dd>

<dt>PhotonVision / PhotonLib</dt>
<dd>An alternative vision pipeline. This codebase uses PhotonLib only in <em>simulation</em> — real cameras are Limelights.</dd>

<dt>Pose / Pose2d / Pose3d</dt>
<dd>A translation + rotation. <code>Pose2d</code> is field-plane (x, y, yaw); <code>Pose3d</code> adds z, pitch, roll.</dd>

<dt>roboRIO</dt>
<dd>The NI single-board computer that runs the robot code.</dd>

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

<dt>SysId</dt>
<dd>WPILib's system-identification framework. The drive has SysId routines for characterization.</dd>

<dt>TOF</dt>
<dd>Time-of-flight sensor. Used by climb beam-breaks.</dd>

<dt>Transition</dt>
<dd>A directed edge in a state-machine graph, backed by a <code>Command</code>. See <a href="{{ '/architecture/state-machines/' | relative_url }}">State Machines</a>.</dd>

<dt>Trench Zone</dt>
<dd>A region of the field where the intake auto-deploys. See <a href="{{ '/reference/field-constants/' | relative_url }}">Field Constants</a>.</dd>

<dt>WPILib</dt>
<dd>The FRC standard library — units, geometry, command framework, hardware abstractions.</dd>

<dt>WPILog</dt>
<dd>WPILib's binary log format. AdvantageKit reads and writes it.</dd>

</dl>
