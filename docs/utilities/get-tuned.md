---
layout: default
title: Tuning with GetTuned
eyebrow: Utilities
description: A thin wrapper over DogLog for dashboard-tunable constants.
permalink: /utilities/get-tuned/
---

[`GetTuned`](https://github.com/frc3748/rebuilt2026/blob/main/src/main/java/frc/robot/util/GetTuned.java)
is the codebase's entry point for **anything you'd want to tune at a
competition without redeploying.**

## The API

```java
double  GetTuned.getNumber(String key, double defaultValue);
boolean GetTuned.getBoolean(String key, boolean defaultValue);
String  GetTuned.getString(String key, String defaultValue);
```

The first call with a given key registers the default with DogLog and
publishes it to NetworkTables. Subsequent calls return the live value
— editable from the dashboard, persisted between reboots.

## Usage convention

Use it **inline at the read site**, not stored in a field:

```java
// Good — picks up dashboard changes
double kP = GetTuned.getNumber("Drive/AutoAlign/kP", 5.0);
controller.setP(kP);

// Bad — captures the boot value
private final double kP = GetTuned.getNumber("Drive/AutoAlign/kP", 5.0);
```

Storing it in a field defeats the point. The cost of calling
`getNumber` is a single hash lookup — negligible compared to the rest
of a `periodic()` body.

## Key naming

A `/`-separated path. The convention is:

```
<Subsystem>/<Mechanism>/<Group>/<Param>
```

Examples:

- `Drive/Module/Drive/kP`
- `Shooter/Flywheel/MaxRPS`
- `Vision/Turret/StdDevFactor`
- `AutoAlign/Translation/kP`

Names need to be stable — changing one loses the dashboard value
between deploys.

## What to tune via `GetTuned` (and what not to)

**Yes:**

- PID gains
- Tolerances
- Override setpoints
- Feature flags (boolean) you want to flip at competition

**No:**

- CAN IDs, gear ratios, geometry — these are physical and don't change
  without a mechanical change.
- Anything that affects safety. The robot should be safe with the
  default value; tunable knobs are for performance, not correctness.

## How values reach the dashboard

DogLog auto-publishes to NetworkTables and to a SmartDashboard widget.
You can edit values from Shuffleboard, Elastic, the DogLog GUI, or any
NT client. Changes persist in `dogconfig` files on the roboRIO between
reboots.

## See also

- [Logging & Telemetry]({{ '/architecture/logging/' | relative_url }})
  for the broader picture of how data flows.
- [DogLog docs](https://doglog.dev/) for the underlying library.
