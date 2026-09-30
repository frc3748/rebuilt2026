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
| **Differences** | Thread-safe (`ConcurrentSkipListMap`); `getSample` returns an `Optional`. |

## Why it exists

Two timing problems plague any robot with vision and a high-rate
control loop:

1. **Vision latency.** A camera frame is taken at time `t`, processed
   for ~20 ms, and arrives at `t + 20ms`. To fuse it correctly, you
   need the robot pose at `t`, not the pose at `t + 20ms`.
2. **Motion at capture time.** A frame taken while the robot was
   spinning is blurry. To reject it, you need the yaw rate around `t`,
   not the yaw rate now.

Both reduce to: *given a timestamp, what was the value?* The
buffer answers exactly that.

## API

```java
public final class ConcurrentTimeInterpolatableBuffer<T> {
  static <T> ConcurrentTimeInterpolatableBuffer<T> createBuffer(Interpolator<T> interpolator, double historySeconds);
  static <T extends Interpolatable<T>> ConcurrentTimeInterpolatableBuffer<T> createBuffer(double historySeconds);
  static ConcurrentTimeInterpolatableBuffer<Double> createDoubleBuffer(double historySeconds);

  void                    addSample(double timeSeconds, T sample);
  Optional<T>             getSample(double timeSeconds);   // interpolates between bracketing samples
  Map.Entry<Double, T>    getLatest();
  ConcurrentNavigableMap<Double, T> getInternalBuffer();
  void                    clear();
}
```

`Pose2d` and the other WPILib geometry types implement
`Interpolatable`, so they use the second factory.

## Used by `RobotState`

[`RobotState`]({{ '/architecture/robot-state/' | relative_url }}) keeps
`LOOKBACK_TIME = 1.0` s of:

```java
ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobot;
ConcurrentTimeInterpolatableBuffer<Double> driveYawAngularVelocity, driveRollAngularVelocity,
                                           drivePitchAngularVelocity, accelX, accelY;
```

`Drive` adds a sample to each every loop. Consumers query through
`RobotState`, not the buffers directly.

## Use cases

### Locating an object at capture time

```java
// In Camera.locate
Pose2d robot = state.getFieldToRobot(observation.timestamp())
        .orElse(state.getLatestFieldToRobot().getValue());
```

The detected object is projected from where the robot was when the
frame was taken, not where it is now.

### Rejecting frames taken while spinning

```java
// In Camera.isStable
state.getMaxAbsDriveYawAngularVelocityInRange(timestamp - kStabilityWindowSeconds, timestamp);
```

This scans the yaw-rate buffer's internal map over the window before
the frame.

## Implementation notes

- Backed by a `ConcurrentSkipListMap<Double, T>`, so it is safe to
  read and write from different threads.
- Each `addSample` drops samples older than the history length.
- Querying a timestamp **before** the oldest sample returns the oldest
  sample. Querying **after** the newest returns the newest. No
  exceptions on out-of-range — the caller decides whether to trust
  the result.

## Pitfalls

- **Stale clocks.** All buffer queries assume FPGA timestamps
  (`RobotTime.getTimestampSeconds()`, `Timer.getFPGATimestamp()`). If you mix in a different time source, the
  interpolation is meaningless.
- **History length.** If you ask for samples older than the
  buffer's history, you'll silently get clamped to the oldest entry.
  Plot the requested-vs-served timestamps to verify.
- **Don't add at irregular rates.** Interpolation assumes the samples
  are reasonably dense over the queried interval. A sparse buffer
  gives jaggy interpolation.
