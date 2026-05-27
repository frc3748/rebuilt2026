---
layout: default
title: Time Buffers
eyebrow: Utilities
description: ConcurrentTimeInterpolatableBuffer — thread-safe, interpolated history for poses, angles, velocities.
permalink: /utilities/time-buffers/
---

The single most important utility in the codebase for "what was the
robot doing N milliseconds ago?" questions.

| | |
| --- | --- |
| **Source** | [`ConcurrentTimeInterpolatableBuffer.java`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/ConcurrentTimeInterpolatableBuffer.java) |
| **Inspired by** | WPILib's `TimeInterpolatableBuffer` |
| **Differences** | Thread-safe; bigger default capacity. |

## Why it exists

Two timing problems plague any robot with vision and a high-rate
control loop:

1. **Vision latency.** A camera frame is taken at time `t`, processed
   for ~20 ms, and arrives at `t + 20ms`. To fuse it correctly, you
   need the robot pose at `t`, not the pose at `t + 20ms`.
2. **Aiming a moving robot.** The shot is fired at time `t + release`.
   To aim correctly, the turret needs the pose at `t + release`, not
   the pose now.

Both reduce to: *given a timestamp, what was/will be the value?* The
buffer answers exactly that.

## API

```java
class ConcurrentTimeInterpolatableBuffer<T> {
  void   addSample(double timestamp, T value);
  T      getSample(double timestamp);        // interpolates between bracketing samples
  T      getLatest();
  double getLatestTimestamp();
  void   clear();
}
```

The interpolation requires `T` to implement
`Interpolatable<T>` or be one of the WPILib geometry types (which
WPILib already provides interpolation for).

## Used by `RobotState`

Three buffers live in
[`RobotState`]({{ '/architecture/robot-state/' | relative_url }}):

```java
ConcurrentTimeInterpolatableBuffer<Pose2d>           fieldToRobotBuffer;
ConcurrentTimeInterpolatableBuffer<Rotation2d>       turretAngleBuffer;
ConcurrentTimeInterpolatableBuffer<ChassisSpeeds>    driveSpeedsBuffer;
```

Each holds ~1 second of history. Every loop, the latest measurement
from the relevant subsystem is added. Consumers query by timestamp.

## Use cases

### Vision fusion at capture time

```java
// In VisionSubsystem
double captureTime = limelight.getCaptureTimestamp();
Pose2d odoPoseAtCapture = robotState.getFieldToRobotAtTime(captureTime);
// Fuse vision pose with odoPoseAtCapture as the reference frame.
```

### Predict pose at shot release

```java
// In ShooterSetpoint
double releaseTime = RobotTime.now() + estimatedShotLatency;
Pose2d predictedPose = robotState.getPredictedFieldToRobot(releaseTime);
```

Prediction uses the latest pose plus the latest `ChassisSpeeds`
integrated forward.

### Correct turret yaw against the camera

```java
Rotation2d turretAtCapture = robotState.getTurretAngleBuffer()
                                       .getSample(captureTime);
Rotation2d turretNow       = turret.getAngle();
Rotation2d delta            = turretNow.minus(turretAtCapture);
// Adjust the vision-reported target angle by delta.
```

## Implementation notes

- Backed by a `TreeMap<Double, T>` under a `ReadWriteLock` for the
  thread-safe variant.
- Old samples drop off when the buffer exceeds its capacity (size or
  age, whichever hits first).
- Querying a timestamp **before** the oldest sample returns the oldest
  sample. Querying **after** the newest returns the newest. No
  exceptions on out-of-range — the caller decides whether to trust
  the result.

## Pitfalls

- **Stale clocks.** All buffer queries assume `RobotTime.now()`-style
  FPGA timestamps. If you mix in a different time source, the
  interpolation is meaningless.
- **Capacity vs. age.** If you ask for samples older than the
  buffer's age limit, you'll silently get clamped to the oldest entry.
  Plot the requested-vs-served timestamps to verify.
- **Don't add at irregular rates.** Interpolation assumes the samples
  are reasonably dense over the queried interval. A sparse buffer
  gives jaggy interpolation.
