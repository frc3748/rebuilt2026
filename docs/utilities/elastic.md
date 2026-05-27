---
layout: default
title: Driver Feedback (Elastic)
eyebrow: Utilities
description: Sending toast notifications to the Elastic dashboard.
permalink: /utilities/elastic/
---

[`Elastic`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/Elastic.java)
is a thin wrapper around the [Elastic dashboard's](https://frc-elastic.gitbook.io/docs)
notification API. Use it for **anything you want the driver to glance
at during a match.**

## API

```java
public static void Elastic.sendNotification(Notification n);

public record Notification(
    NotificationLevel level,
    String title,
    String description,
    double displayTimeMillis);

public enum NotificationLevel { INFO, WARNING, ERROR }
```

The dashboard renders the notification as a colored toast that fades
after `displayTimeMillis`.

## Usage examples

```java
// Mechanism homed
Elastic.sendNotification(new Elastic.Notification(
    NotificationLevel.INFO,
    "Climb zeroed",
    "Ready to deploy",
    2000));

// Vision dropout
Elastic.sendNotification(new Elastic.Notification(
    NotificationLevel.WARNING,
    "Vision lost",
    "No tags for 2s — odometry only",
    3000));

// Auto failed to load
Elastic.sendNotification(new Elastic.Notification(
    NotificationLevel.ERROR,
    "Auto not loaded",
    "Selected path missing on roboRIO",
    5000));
```

## When to send a notification

Useful:

- A mechanism finished homing.
- Vision restored after a dropout.
- A wrong-alliance match start was detected.
- An auto routine selected but not found on disk.
- A subsystem failed to determine state.

**Not** useful:

- Anything that happens every loop.
- Anything purely informational that the driver couldn't act on.
- Repeated firings of the same message (rate-limit it yourself).

## Best practice

The dashboard is a finite attention resource. Three notifications in
the first 15 seconds of a match is one too many. If you find yourself
adding a fourth, it probably belongs in AdvantageKit (for developers)
or a permanent dashboard widget (for the driver), not in a toast.

## Where it goes

Elastic listens on NetworkTables. The wrapper publishes a JSON-encoded
notification under a known NT key; the dashboard subscribes and
renders. No setup required on the robot side beyond standard NT.
