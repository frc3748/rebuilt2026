---
layout: default
title: Rebuilt 2026
hide_title: true
description: The 2026 robot code for FRC Team 3748 — a state-machine driven, hardware-abstracted Java codebase built on WPILib and AdvantageKit.
---

<section class="hero">
  <div class="eyebrow">FRC Team 3748 · Season 2026</div>
  <h1>Rebuilt&nbsp;2026</h1>
  <p class="lede">
    A hardware-abstracted, state-machine driven Java codebase for FRC.
    Swerve drive, vision-fused odometry, a fixed shooter, automatic
    pathing, and a tunable simulator — all logged through AdvantageKit.
  </p>
  <div class="hero-actions">
    <a class="btn btn-primary" href="{{ '/getting-started/' | relative_url }}">Get started →</a>
    <a class="btn" href="{{ '/architecture/robot-lifecycle/' | relative_url }}">Read the architecture</a>
    <a class="btn" href="https://github.com/frc3748/rebuilt2026" target="_blank" rel="noopener">View source</a>
  </div>
</section>

<div class="article-body">

<h2 id="explore-the-docs">Explore the docs</h2>

<div class="card-grid">
  <a class="card" href="{{ '/getting-started/' | relative_url }}">
    <h3>Getting Started</h3>
    <p>Clone the repo, install WPILib, deploy to a roboRIO, run the sim.</p>
  </a>
  <a class="card" href="{{ '/architecture/robot-lifecycle/' | relative_url }}">
    <h3>Robot Lifecycle</h3>
    <p>How <code>Main</code>, <code>Robot</code>, and <code>RobotState</code> fit together.</p>
  </a>
  <a class="card" href="{{ '/architecture/robots/' | relative_url }}">
    <h3>Multiple Robots</h3>
    <p>One codebase for the comp, secondary and practice robots, and how to add another.</p>
  </a>
  <a class="card" href="{{ '/architecture/io-pattern/' | relative_url }}">
    <h3>The IO Layer Pattern</h3>
    <p>Why every subsystem talks to hardware through an interface with real and sim impls.</p>
  </a>
  <a class="card" href="{{ '/architecture/state-machines/' | relative_url }}">
    <h3>State Machines</h3>
    <p>The transition graph that drives every mechanism on the robot.</p>
  </a>
  <a class="card" href="{{ '/subsystems/drive/' | relative_url }}">
    <h3>Drive</h3>
    <p>Swerve modules, pose estimation, and PathPlanner integration.</p>
  </a>
  <a class="card" href="{{ '/subsystems/shooter/' | relative_url }}">
    <h3>Shooter</h3>
    <p>Flywheel, hood, hopper and kicker — a child-subsystem composite.</p>
  </a>
  <a class="card" href="{{ '/subsystems/vision/' | relative_url }}">
    <h3>Vision</h3>
    <p>Two Limelights, Megatag fusion, and the pose-estimator hand-off.</p>
  </a>
  <a class="card" href="{{ '/commands/action-commands/' | relative_url }}">
    <h3>Commands</h3>
    <p>The buttons, the autos, and how driver intent becomes motion.</p>
  </a>
</div>

<h2 id="what-makes-this-codebase">What makes this codebase</h2>

<p>
Three patterns shape almost every file:
</p>

<ol>
  <li>
    <strong>IO layering</strong> — every subsystem talks to hardware
    through an interface (<code>ModuleIO</code>, <code>MotorIO</code>,
    <code>CameraIO</code>). Real implementations live alongside
    simulator implementations; the subsystem code itself never knows
    the difference.
  </li>
  <li>
    <strong>State machines</strong> — every subsystem extends
    <code>StateMachine&lt;E&gt;</code> with its own state enum.
    Transitions are <code>Command</code>s registered in a directional
    graph. <code>requestTransition(state)</code> is the public verb.
  </li>
  <li>
    <strong>AdvantageKit-first logging</strong> — inputs are recorded
    on every loop, replayable in AdvantageScope, and surfaced live on
    the dashboard via DogLog and Elastic.
  </li>
</ol>

<blockquote class="callout">
  <div class="callout-title">New to the codebase?</div>
  <p>
    Read <a href="{{ '/architecture/robot-lifecycle/' | relative_url }}">Robot Lifecycle</a>
    first to see how a single button press travels from the controller
    all the way to a motor.
  </p>
</blockquote>

<h2 id="quick-tour">Quick tour</h2>

<p>
The same code runs three robots, picked by <code>Constants.kRobot</code>
(see <a href="{{ '/architecture/robots/' | relative_url }}">Multiple Robots</a>).
The <strong>comp</strong> robot is the new chassis and future main robot,
the <strong>secondary</strong> robot is the current robot and shares
comp's mechanisms and cameras, and the
<strong>practice</strong> robot is a drivetrain with no cameras.
</p>

<p>The comp and secondary robots' subsystems:</p>

<table>
<thead><tr><th>Subsystem</th><th>What it does</th></tr></thead>
<tbody>
<tr><td><a href="{{ '/subsystems/drive/' | relative_url }}">Drive</a></td><td>Four swerve modules, 28″×28″, max ~5.27 m/s.</td></tr>
<tr><td><a href="{{ '/subsystems/vision/' | relative_url }}">Vision</a></td><td>Any number of cameras, each a config plus an IO.</td></tr>
<tr><td><a href="{{ '/subsystems/shooter/' | relative_url }}">Shooter</a></td><td>Owns the flywheel, hood, hopper and kicker; the drive aims the robot.</td></tr>
<tr><td><a href="{{ '/subsystems/intake/' | relative_url }}">Intake</a></td><td>Pivoting arm that pulls game pieces from the floor.</td></tr>
<tr><td><a href="{{ '/subsystems/hopper/' | relative_url }}">Hopper</a></td><td>Carries pieces from intake to shooter.</td></tr>
<tr><td><a href="{{ '/subsystems/kicker/' | relative_url }}">Kicker</a></td><td>Pushes the piece the final inch into the flywheel.</td></tr>
</tbody>
</table>

</div>
