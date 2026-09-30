---
layout: default
title: Vision
eyebrow: Subsystem
description: Any number of cameras, each a config plus an IO, feeding the drive's pose estimator.
permalink: /subsystems/vision/
---

The vision subsystem is a list of cameras. Every loop it asks each
camera for a filtered pose estimate and hands the good ones to
`RobotState#addVisionEstimate`, which forwards them to the drive's pose
estimator on the real robot.

## The pieces

| Class | Role |
| --- | --- |
| `CameraConfig` | Name, NetworkTables name, vendor type, robot→camera `Transform3d`, std-dev factor. |
| `CameraIO` | Interface with `@AutoLog` inputs and `setRobotOrientation`. |
| `CameraIOLimelight` | Reads MegaTag 1 and MegaTag 2 from a Limelight. |
| `CameraIOPhoton` | PhotonVision coprocessor multi-tag solve (lowest-ambiguity fallback) plus the heading-seeded trig solve. |
| `CameraIOPhotonSim` | `CameraIOPhoton` with a simulated camera attached. |
| `Camera` | Owns one config, one IO, one inputs object. Filters and weights the estimate. |
| `Vision` | Owns the cameras and forwards estimates. |

## Adding a camera

```java
public static final CameraConfig kRearCamera = new CameraConfig("Rear Camera", "photon-rear", CameraConfig.Type.PHOTON)
        .robotToCamera(new Transform3d(...))
        .stdDevFactor(1.5);
```

Add it to `VisionConstants.kCameras`. Adding a vendor is one new
`CameraIO` class and one line in `Camera.of`.

## Filtering

An estimate is rejected when the camera sees no targets, the pose is
zero, the timestamp repeats, or the chassis yaw rate over the previous
100 ms exceeded `kMaxYawRateRadPerSec`. Translation comes from MegaTag 2,
heading from MegaTag 1. Standard deviations scale with average tag
distance squared over tag count, times the camera's `stdDevFactor`.

## Cameras on this robot

| Config | Network name | Mount |
| --- | --- | --- |
| `kShooterCamera` | `limelight-turret` | On the fixed shooter, pitched up 20.5° |
| `kChassisCamera` | `limelight` | Rear of chassis, pitched up 45°, yawed 180° |
