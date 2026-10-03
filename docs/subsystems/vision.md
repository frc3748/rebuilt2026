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

## Heading from MegaTag 1

MegaTag 2 needs the robot's heading to be right already, so the heading
comes from MegaTag 1, and only from frames that can't be wrong. A
MegaTag 1 frame is rejected as "not strict" unless it sees at least
`kStrictHeadingMinTags` (2) tags, the tags average under
`kStrictHeadingMaxDistanceMeters` (4 m), ambiguity is under
`kStrictHeadingMaxAmbiguity`, and the chassis is turning slower than
`kStrictHeadingMaxYawRateRadPerSec`. `MULTI_TAG` frames (PhotonVision)
count too.

Each strict frame adds a heading sample. `Vision` keeps the last
`kHeadingWindowSeconds` of them, and once `kHeadingSamples` (5) samples
all agree within `kHeadingAgreementDegrees` (1°), their average is the
heading fix:

- **While disabled**, if the fix is more than
  `kHeadingCorrectionDegrees` off, `Vision` resets the pose's rotation to
  it. This fixes the gyro on the start line.
- **During the match**, strict frames still go into the pose estimator,
  so the heading keeps getting corrected by good frames only.
- `Vision/HeadingConfirmed` turns true when the fix and the robot's
  heading agree. The dashboard's **Heading confirmed** check waits for
  it. Once confirmed it stays confirmed while the gyro holds the heading.

If the cameras can't see two tags within 4 m from your start pose,
the check stays red. Turn the robot past some tags while setting it up
(it stays confirmed afterwards), or relax the limits in
`VisionConstants`.

Vision also raises alerts for the dashboard: a camera disconnected
(warning), no accepted pose for `kNoVisionSeconds` while enabled
(warning), and vision and odometry disagreeing by more than
`kDisagreeMeters` (error).

## Filtering

A pose is rejected when it has no tags, is the zero pose, is off the
field, repeats the last timestamp for its source, has a Z error over
`kMaxZErrorMeters`, is a single ambiguous tag from a source that checks
ambiguity, was captured while the chassis was spinning faster than
`kMaxYawRateRadPerSec`, or is a MegaTag 1 frame that isn't strict.
Accepted and rejected poses are both logged per
camera, as `Vision/<name>/AcceptedPoses` and `RejectedPoses`. In
simulation each camera also logs its pose and the tags it sees
(`Vision/<name>/CameraPose`, `Vision/<name>/Tags`).

Standard deviations scale with average tag distance squared over tag
count, times the camera's `stdDevFactor`, times the source factors above.
`TRIG_SOLVE` counts as one tag at the distance of the tag it solved from:
PhotonVision solves it from the best tag alone and the robot's heading, so
a heading error of 1° moves it about 1.7 cm per metre to that tag. It used
to count every tag in view at their average distance, and right after a
fast spin it pulled the pose 9 cm off with tags 12 m away.

## Object detection

A camera reports objects only while it runs its detection pipeline.
`.pipelines(tags, detection)` gives a camera both jobs, and it switches
when `Vision` is in `OBJECTS`. `.detector(pipeline)` makes a camera that
only detects objects, like the Limelight 3 fuel camera on COMP. Its
pose observations are ignored.

`Camera` projects each object observation onto the floor, using the
camera's robot→camera transform and the configured object height. It
then converts the result to a field position using the robot pose at
capture time. `Vision` keeps everything seen in the last
`kObjectMemorySeconds`. It merges detections closer than
`kObjectMergeMeters` into one object, so each ball appears once.

| Method | Returns |
| --- | --- |
| `Vision#getObjects()` | Every `DetectedObject` (timestamp, class, field position, confidence). |
| `Vision#getObjectPoses()` | A `Pose2d` for each object, at the object and facing it from the robot. |
| `Vision#getClosestObject()` | The object nearest the robot. There's also a version that takes a point. |
| `Vision#getClosestObjectPose()` | The pose at the closest object, facing it from the robot. It can go straight into `AutoAlignToPoseCommand`. |
| `Vision#seesObjects()` | Whether anything is remembered right now. |
| `DetectedObject#poseFrom(origin)` | The pose at the object, facing it from `origin`. |

These are logged as `Vision/Objects`, `Vision/ClosestObject` and
`Vision/<name>/Objects`.

On a Limelight, a neural detector pipeline reports every detection (a
Limelight 3 needs a Google Coral for this). A color pipeline reports its
best target as class 0. PhotonVision color targets and simulated targets
have no class, so they count as class 0 too.

Limelight detections are timed from when each frame arrives (the `hb`
heartbeat), minus the pipeline and capture latency. Each frame is read only
once. So even slow detectors, like CPU-only neural detection on a Limelight
3A, place objects using the robot pose from when the image was taken.
`kObjectMemorySeconds` (0.5 s) is longer than the time between frames on
those, so objects don't blink out.

In simulation, the balls from the fuel simulation are PhotonVision sim
targets, so detector cameras see the real simulated balls. For speed,
only the `kSimMaxObjects` balls nearest the robot, within
`kSimObjectRangeMeters`, are simulated as targets each loop.

## Camera types

`CameraConfig.Type` picks the `CameraIO` used on the real robot:

| Type | IO | Notes |
| --- | --- | --- |
| `LIMELIGHT_3` | `CameraIOLimelight` | MegaTag 1 + 2, or neural and color detection. |
| `LIMELIGHT_3G` | `CameraIOLimelight` | MegaTag 1 + 2. |
| `LIMELIGHT_4` | `CameraIOLimelight` | Also switches on the camera's internal IMU (`SetIMUMode(1)`). Only LL4 does this. |
| `PHOTON` | `CameraIOPhoton` | PhotonVision. Intended for object detection this year. |

In simulation every camera uses `CameraIOPhotonSim`, whatever its type.

## Adding a camera

Cameras belong to a robot. Each `RobotDefinition` returns its own list
from `cameras()`, which is empty by default. `CompRobot` builds its
cameras in protected methods, `chassisCamera()`, `shooterCamera()` and `fuelCamera()`,
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
| `shooterCamera()` | `limelight-turret` | `LIMELIGHT_4` | Fixed to the shooter; its transform starts from `shooter().shooterToRobotCenter`. Shifts its reported pose by an in-code `reportedPoseOffset` on the real robot and in replay (the simulator's camera already reports the true pose, so it's skipped there). |
| `chassisCamera()` | `limelight` | `LIMELIGHT_4` | Rear of chassis. Offsets live in the Limelight web UI. |
