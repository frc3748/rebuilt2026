---
layout: default
title: Vendor Dependencies
eyebrow: Reference
description: Every external library the codebase pulls in and why.
permalink: /reference/vendor-deps/
---

Vendor deps live in `vendordeps/` and are pulled in by GradleRIO at
build time. Here's the full set.

## WPILib core

| Lib | Why |
| --- | --- |
| WPILib 2026 (bundled with toolchain) | The base FRC SDK. |
| **WPILib NewCommands** | Command-based framework. Every state machine extends `SubsystemBase`, every transition is a `Command`. |

## Logging & telemetry

| Lib | Why |
| --- | --- |
| **AdvantageKit** (Littleton Robotics) | The replay-driven logger. Annotation processor generates `*AutoLogged` classes. See [Logging & Telemetry]({{ '/architecture/logging/' | relative_url }}). |
| **DogLog** | Dashboard-tunable values. See [`GetTuned`]({{ '/utilities/get-tuned/' | relative_url }}). |

## Motor controllers

| Lib | Why |
| --- | --- |
| **REVLib** | Spark Max / Spark Flex API. Used for every NEO on the robot. |
| **Phoenix 6** (`v26.1.1`) | Pigeon 2 gyro, any TalonFX motors. |
| **Phoenix 5** (`v5.36.0`) | Legacy CTRE devices, kept for backward compatibility. |

## Vision

| Lib | Why |
| --- | --- |
| **PhotonLib** | Simulator-side AprilTag pipeline. Real cameras use Limelight via NetworkTables (no vendor dep needed). |

## Trajectory & autonomy

| Lib | Why |
| --- | --- |
| **PathPlannerLib** (`2026.1.2`) | Trajectory generation, path-following, auto-builder. |
| **ChoreoLib2026** | Alternative trajectory format; ready for use even if no autos currently use it. |

## Sensors

| Lib | Why |
| --- | --- |
| **libgrapplefrc2026** | Grapple/LaserCAN-style sensors. |
| **redux** | Redux Robotics sensor support. |
| **ThriftyLib** (`2026.1.0`) | Thrifty Bot encoder support. |

## Simulation

| Lib | Why |
| --- | --- |
| **MapleSim** (`0.4.0-beta`) | Advanced physics simulator. The drive-train sim layer. |

## Misc

| Lib | Why |
| --- | --- |
| **LumynLabs** | Vendor utilities. |
| **URCL** | URCL motor-data logger (REV telemetry over CAN). |

## Updating vendor deps

For most libs, the WPILib VS Code "Manage Vendor Libraries → Check for
Updates" workflow is the right path. For libs without that integration:

1. Download the new JSON from the vendor's site.
2. Drop it into `vendordeps/`, replacing the older version.
3. `./gradlew --refresh-dependencies build`.

Watch the changelog for breaking API changes — Phoenix 6 in particular
sometimes renames methods between minor versions.

## Pitfalls

- **Phoenix 5 + 6 conflict.** Don't import from `com.ctre.phoenix.*`
  in new code; always use `com.ctre.phoenix6.*`. The legacy package is
  there for one or two devices that haven't migrated.
- **Vendor JSON out of sync.** If `./gradlew build` complains about a
  missing artifact, force-refresh: `./gradlew --refresh-dependencies build`.
- **REVLib API changes between seasons.** Spark Max → Spark Flex APIs
  shifted. The codebase pins to a specific version; update with care.
