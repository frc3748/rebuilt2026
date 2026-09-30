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
camera, as `Vision/<name>/AcceptedPoses` and `RejectedPoses`. In
simulation each camera also logs its pose and the tags it sees
(`Vision/<name>/CameraPose`, `Vision/<name>/Tags`).

Standard deviations scale with average tag distance squared over tag
count, times the camera's `stdDevFactor`, times the source factors above.

## Object detection

`Camera` projects each object observation onto the floor using the
camera's robot→camera transform and the configured object height, then
converts it to a field position using the robot pose at capture time.
`Vision#getObjects()` returns everything seen in the last
`kObjectMemorySeconds`, and `Vision#getClosestObject()` returns the
nearest one.

## Camera types

`CameraConfig.Type` picks the `CameraIO` used on the real robot:

| Type | IO | Notes |
| --- | --- | --- |
| `LIMELIGHT_3` | `CameraIOLimelight` | MegaTag 1 + 2. |
| `LIMELIGHT_3G` | `CameraIOLimelight` | MegaTag 1 + 2. |
| `LIMELIGHT_4` | `CameraIOLimelight` | Also switches on the camera's internal IMU (`SetIMUMode(1)`). Only LL4 does this. |
| `PHOTON` | `CameraIOPhoton` | PhotonVision. Intended for object detection this year. |

In simulation every camera uses `CameraIOPhotonSim`, whatever its type.

## Adding a camera

Cameras belong to a robot. Each `RobotDefinition` returns its own list
from `cameras()`, which is empty by default. `CompRobot` builds its
cameras in protected methods, `chassisCamera()` and `shooterCamera()`,
so a robot that extends it can replace one. A new camera is one more
method:

```java
protected CameraConfig intakeCamera() {
    return new CameraConfig("Intake Camera", "photon-intake", CameraConfig.Type.PHOTON)
            .robotToCamera(new Transform3d(...))
            .pipelines(0, 1)
            .objectHeight(Units.inchesToMeters(2.25))
            .stdDevFactor(1.5);
}
```

Then return it from the robot's `cameras()`:

```java
@Override
public CameraConfig[] cameras() {
    return new CameraConfig[] { chassisCamera(), shooterCamera(), intakeCamera() };
}
```

A camera on a moving mechanism passes a supplier instead:
`.robotToCamera(() -> turretToCamera(turret.getAngle()))`. On a robot
with no cameras, `Vision` simply has nothing to do.

## Adding a vendor

Write one `CameraIO` that fills the three observation arrays, and add
one constant to `CameraConfig.Type`:

```java
QUEST(CameraIOQuest::new)
```

Nothing else changes.

## Cameras on the comp and secondary robots

Defined in `CompRobot`; `SecondaryRobot` inherits them. The practice
robot has none.

| Method | Network name | Type | Notes |
| --- | --- | --- | --- |
| `shooterCamera()` | `limelight-turret` | `LIMELIGHT_4` | Fixed to the shooter; its transform starts from `shooter().shooterToRobotCenter`. Shifts its reported pose by an in-code `reportedPoseOffset`. |
| `chassisCamera()` | `limelight` | `LIMELIGHT_4` | Rear of chassis. Offsets live in the Limelight web UI. |
