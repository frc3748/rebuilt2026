---
layout: default
title: Geometry & Math
eyebrow: Utilities
description: GeomUtil, MathHelpers — pose math you'll write fifty times if you don't import these.
permalink: /utilities/geometry-math/
---

Two utility classes cover almost every geometric operation in the codebase:

## `GeomUtil`

[`GeomUtil`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/GeomUtil.java)
holds pose-transformation helpers.

```java
// Compose & invert
Transform2d  invert(Transform2d t);
Pose2d       compose(Pose2d a, Transform2d b);

// Conversions
Transform2d  poseAsTransform(Pose2d p);
Pose2d       transformAsPose(Transform2d t);

// 2D <-> 3D
Pose3d       toPose3d(Pose2d p);   // z = 0, pitch/roll = 0
Pose2d       toPose2d(Pose3d p);   // drop z, pitch, roll

// Distance / angle queries
double       distanceTo(Pose2d a, Pose2d b);
Rotation2d   angleTo(Pose2d from, Translation2d target);
```

The 2D ↔ 3D helpers are used heavily by vision and turret math because
shooting is inherently 3D but the drive lives in 2D.

## `MathHelpers`

[`MathHelpers`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/MathHelpers.java)
exposes pre-baked zero constants and angle utilities.

### Zero constants

Allocating `new Pose2d()` every loop is wasteful and gives the GC
something to do. These constants are immutable singletons:

```java
MathHelpers.kPose2dZero;
MathHelpers.kPose3dZero;
MathHelpers.kRotation2dZero;
MathHelpers.kRotation3dZero;
MathHelpers.kTranslation2dZero;
MathHelpers.kTranslation3dZero;
```

Use them anywhere you'd type `new Pose2d()` for an identity value.

### Angle helpers

```java
double wrap(double radians);        // wrap to [-pi, pi]
double wrapDegrees(double deg);     // wrap to [-180, 180]
double smallestAngleDelta(double a, double b);
```

These are the ones you reach for when an angle is "almost equal" to
another but shows up off by 2π.

## Patterns you'll see

- **`MathHelpers.kPose2dZero`** in default field-pose fallbacks.
- **`GeomUtil.distanceTo`** in any shooter solver.
- **`GeomUtil.toPose3d`** when the drive pose needs to seed a vision
  transform chain.

## Adding new helpers

Both files are catch-alls — if you find yourself writing the same
geometric expression in two subsystems, lift it into one of these.
Don't make a new utility class for one method.
