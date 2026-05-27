---
layout: default
title: Vision
eyebrow: Subsystem
description: Two Limelights with Megatag2 pose fusion, feeding the drive's pose estimator.
permalink: /subsystems/vision/
---

The vision subsystem runs two cameras, picks the best estimate every
frame, and hands it to `Drive#addVisionMeasurement`. It never moves a
motor — its only outputs are pose estimates.

| | |
| --- | --- |
| **Source** | `src/main/java/frc/robot/subsystems/vision/` |
| **Public class** | [`VisionSubsystem`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/subsystems/vision/VisionSubsystem.java) extends `StateMachine<State>` |
| **Constants** | [`VisionConstants`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/subsystems/vision/VisionConstants.java) |

## States

```java
enum State { UNDETERMINED, VISION_SCANNING, BROKEN }
```

`BROKEN` is a soft-fail state for when both cameras stop reporting
fresh data — drive falls back to odometry-only.

## Cameras

Two physical cameras:

| Camera | Mount | Use |
| --- | --- | --- |
| **Turret Limelight** | On the turret housing | Primary — gets aim feedback for the shooter. |
| **Chassis Limelight** | Forward-facing, rear of chassis | Secondary — wider FOV, helps with initial localization. |

### Turret Limelight transform (from `VisionConstants`)

| Parameter | Value |
| --- | --- |
| Pitch | 20.5° |
| Height above ground | 4.181 in |
| Forward offset from turret center | 4.594 in |
| Right offset from turret center | 4.270 in |
| Turret→robot-center transform | (-3.290, -4.750, 13.735) in |

The turret-to-robot-center transform is the most important constant in
this file. If your shots are consistently long or short, this is the
first place to look.

### Chassis Limelight transform

| Parameter | Value |
| --- | --- |
| Pitch | 45° |
| Yaw | 180° |
| Forward offset | 13 in |
| Right offset | 0.75 in |
| Height | 5.75 in |

## IO implementations

- **`VisionIOHardwareLimelight`** — real hardware, NetworkTables reads
  via [`LimelightHelpers`]({{ '/utilities/get-tuned/' | relative_url }}).
- **`VisionIOSimPhoton`** — simulated cameras via PhotonVision's sim
  pipeline. Renders fiducial detections from the 2026 field layout.

## The processing pipeline

Every loop:

1. `io.readInputs()` pulls Megatag1 + Megatag2 estimates for each camera.
2. `processCamera()` decides which estimate to use:
   - **Megatag2** is preferred. It's a single-tag-aware solve that's
     more stable for partial occlusions.
   - **Megatag1** is the fallback when Megatag2 is unavailable.
3. The chosen estimate's standard deviations are scaled by the
   camera's trust factor (`1.0` for chassis, `1.3` for turret).
4. `Drive#addVisionMeasurement(pose, timestamp, stdDevs)` is called.

> Note: an experimental weighted-variance fusion across both cameras
> is implemented but currently disabled in favor of the turret-only
> path. See the `processCamera()` flow in
> `VisionSubsystem.java` for the fusion code.

## Standard deviations

| Source | Linear σ | Angular σ |
| --- | --- | --- |
| Baseline | 0.02 m | 0.10 rad |
| Megatag2 factor | × 0.5 | × ∞ (ignored) |
| Chassis camera factor | × 1.0 | × 1.0 |
| Turret camera factor | × 1.3 | × 1.3 |

Megatag2 trusts gyro for rotation, so its angular σ is effectively
infinite — the pose estimator only fuses translation.

## Field constants

The 2026 Rebuilt & Andymark AprilTag layout. Field is ~16.54 m × ~8.21 m.
Valid fiducial IDs are 1–32, with a subset of tag IDs actually
populated for the game.

## Pose estimate types

Three small data classes:

- **`FiducialObservation`** — one detected tag, with 3D pose and
  ambiguity.
- **`MegatagPoseEstimate`** — robot pose from a Megatag solve, with
  metadata (num tags, avg distance, ambiguity).
- **`VisionFieldPoseEstimate`** — what gets handed to `Drive`. Wraps
  pose, timestamp, and stdDevs.

## Public API

```java
// Read from the subsystem
List<VisionFieldPoseEstimate> getRecentEstimates();
boolean isAnyCameraConnected();

// Read from VisionConstants
Transform3d kTurretCameraToTurret;
Transform3d kTurretToRobotCenter;
```

## Common pitfalls

- **Tags suddenly invalid.** Check the valid-IDs list in
  `VisionConstants`. Newly-added field tags need to be added here.
- **Pose jumps.** Likely a low-tag-count Megatag1 estimate slipping
  through. Either tighten the ambiguity threshold or rely on Megatag2.
- **Turret aiming off by a few degrees.** Almost always the
  turret-to-robot-center transform. Re-measure with calipers, don't
  eyeball.
- **Latency feels high.** Verify timestamps include capture latency
  (`tl` + `cl` from Limelight) — `LimelightHelpers` does this for you,
  but if you bypass it, you'll see the camera lag.
