---
layout: default
title: Vision
eyebrow: Subsystem
description: Any number of cameras of any vendor, reporting generic pose and object observations.
permalink: /subsystems/vision/
---

The vision subsystem is a list of cameras. Every loop each camera reads
its vendor-specific data into generic observations, `Camera` filters and
weights them, and `Vision` hands the accepted poses to
`RobotState#addVisionMeasurement`, which feeds the drive's pose estimator
on the real robot.

## States

| State | Behavior |
| --- | --- |
| `APRIL_TAGS` | Every camera runs its AprilTag pipeline. |
| `OBJECTS` | Cameras that have a detection pipeline switch to it; the rest keep estimating pose. |
| `UNDETERMINED` | While the robot is disabled. Behaves like `APRIL_TAGS`, so the robot keeps localizing before a match. |

The "Vision Disable" dashboard button stops pose estimates from reaching
the drive until the robot reboots. Cameras keep logging.

## Observations

Each `CameraIO` fills `CameraInputs`:

| Field | Meaning |
| --- | --- |
| `poseObservations` | `PoseObservation(timestamp, robotPose, ambiguity, tagCount, averageTagDistance, source)` |
| `objectObservations` | `ObjectObservation(timestamp, classId, yawDegrees, pitchDegrees, area, confidence)` |
| `tagIds` | Tags seen this loop. |

`PoseSource` sets how much each kind of estimate is trusted:

| Source | Translation | Heading | Ambiguity check |
| --- | --- | --- | --- |
| `MEGATAG_1` | ignored | trusted | yes |
| `MEGATAG_2` | trusted ×0.5 | ignored | no |
| `MULTI_TAG` | trusted | trusted | no |
| `SINGLE_TAG` | trusted | trusted | yes |
| `TRIG_SOLVE` | trusted ×0.5 | ignored | no |

This is the old "MegaTag 2 translation, MegaTag 1 rotation" rule written
as data, so a new vendor only has to say where its poses came from.

## Filtering

A pose is rejected when it has no tags, is the zero pose, is off the
field, repeats the last timestamp for its source, has a Z error over
`kMaxZErrorMeters`, is a single ambiguous tag from a source that checks
ambiguity, or was captured while the chassis was spinning faster than
`kMaxYawRateRadPerSec`. Accepted and rejected poses are both logged per
camera.

Standard deviations scale with average tag distance squared over tag
count, times the camera's `stdDevFactor`, times the source factors above.

## Object detection

`Camera` projects each object observation onto the floor using the
camera's robot→camera transform and the configured object height, then
converts it to a field position using the robot pose at capture time.
`Vision#getObjects()` returns everything seen in the last
`kObjectMemorySeconds`, and `Vision#getClosestObject()` returns the
nearest one.

## Adding a camera

```java
public static final CameraConfig kIntakeCamera = new CameraConfig("Intake Camera", "photon-intake", CameraConfig.Type.PHOTON)
        .robotToCamera(new Transform3d(...))
        .pipelines(0, 1)
        .objectHeight(Units.inchesToMeters(2.25))
        .stdDevFactor(1.5);
```

Add it to `VisionConstants.kCameras`. A camera on a moving mechanism
passes a supplier instead: `.robotToCamera(() -> turretToCamera(turret.getAngle()))`.

## Adding a vendor

Write one `CameraIO` that fills the three observation arrays, and add
one constant to `CameraConfig.Type`:

```java
QUEST(CameraIOQuest::new)
```

Nothing else changes.

## Cameras on this robot

| Config | Network name | Notes |
| --- | --- | --- |
| `kShooterCamera` | `limelight-turret` | Fixed to the shooter. Applies the same in-code offset to its reported pose as the turret-era code did. |
| `kChassisCamera` | `limelight` | Rear of chassis. Offsets live in the Limelight web UI. |
