---
layout: default
title: CAN ID Map
eyebrow: Reference
description: Every device on the CAN bus and its ID, for each robot.
permalink: /reference/can-ids/
---

Drive IDs live in each robot's `DriveConfig` subclass; mechanism IDs
live in the `*Constants` files. This page is the cross-reference.

> **Authoritative source** is the file in the "Defined in" column.
> If a number here disagrees with the source, the source wins.

## Competition robot

### Drive

Defined in `robots/competition/CompetitionDrive.java`.

| Module | Drive (Spark Flex) | Turn (Spark MAX) | CANcoder |
| --- | --- | --- | --- |
| Front-left | 8 | 3 | 4 |
| Front-right | 4 | 7 | 2 |
| Back-left | 2 | 9 | 1 |
| Back-right | 6 | 5 | 3 |

| Device | CAN ID |
| --- | --- |
| Pigeon 2 gyro | 50 |

The CANcoders reuse IDs 1–4 alongside REV devices. Devices from
different vendors can share an ID.

### Mechanisms

| Device | Controller | CAN ID | Defined in |
| --- | --- | --- | --- |
| Flywheel leader | Spark Flex | 55 | `FlywheelConstants` |
| Flywheel follower | Spark Flex | 56 | `FlywheelConstants` |
| Hood | Spark MAX | 54 | `HoodConstants` |
| Intake extension leader | Spark MAX | 46 | `IntakeConstants` |
| Intake extension follower | Spark MAX | 47 | `IntakeConstants` |
| Intake rollers | Spark Flex | 48 | `IntakeConstants` |
| Hopper | Spark Flex | 15 | `HopperConstants` |
| Kicker | Spark MAX | 42 | `KickerConstants` |

## Competition V2 robot

Defined in `robots/competitionv2/CompetitionV2Drive.java`. It keeps the
competition robot's electronics, so every CAN ID above is the same. Only
the module zero rotations change (they start at `0` until calibrated).

## Practice robot

Defined in `robots/practice/PracticeDrive.java`. The practice robot has
no mechanisms and no CANcoders; the turn motors use the Spark's
absolute encoder.

| Module | Drive (Spark MAX) | Turn (Spark MAX) | Drive inverted |
| --- | --- | --- | --- |
| Front-left | 4 | 3 | yes |
| Front-right | 5 | 7 | no |
| Back-left | 6 | 2 | yes |
| Back-right | 8 | 41 | no |

The NavX gyro is on USB (`NavXComType.kUSB1`), not CAN.

## Adding a new device

1. Pick a CAN ID that doesn't conflict with another device of the same
   vendor on that robot.
2. Add it to the robot's `DriveConfig` or the mechanism's `*Constants` class.
3. Document it on this page so the next person doesn't have to grep.

## Conflict checklist

Before deploying with a new device:

- [ ] ID is unique among that vendor's devices on the bus.
- [ ] Device shows up in REV Hardware Client / Phoenix Tuner X.
- [ ] Firmware is up to date.
- [ ] Brake/coast mode set as expected.
- [ ] Current limit set in the `MotorConfig` (`.currentLimit(...)`).
