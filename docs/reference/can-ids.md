---
layout: default
title: CAN ID Map
eyebrow: Reference
description: Every device on the CAN bus and its ID.
permalink: /reference/can-ids/
---

CAN IDs are scattered across the various `*Constants` files. This page
is the canonical cross-reference.

> **Authoritative source** is the constants file for each subsystem.
> If a number here disagrees with the source, the source wins.

## Drive

| Device | CAN ID | Defined in |
| --- | --- | --- |
| Drive motor 0 (FL) | 8 | `DriveConstants` |
| Drive motor 1 (FR) | 2 | `DriveConstants` |
| Drive motor 2 (BL) | 4 | `DriveConstants` |
| Drive motor 3 (BR) | 6 | `DriveConstants` |
| Turn motor 0 (FL) | 3 | `DriveConstants` |
| Turn motor 1 (FR) | 9 | `DriveConstants` |
| Turn motor 2 (BL) | 7 | `DriveConstants` |
| Turn motor 3 (BR) | 5 | `DriveConstants` |
| CANcoder 0 (FL) | 4 | `DriveConstants` |
| CANcoder 1 (FR) | 1 | `DriveConstants` |
| CANcoder 2 (BL) | 2 | `DriveConstants` |
| CANcoder 3 (BR) | 3 | `DriveConstants` |
| Pigeon 2 gyro | 50 | `DriveConstants` |

The CANcoders share IDs 1–4 with REVLib's NEO IDs — they're on the
CTRE bus, so no conflict.

## Shooter

| Device | CAN ID | Defined in |
| --- | --- | --- |
| Flywheel master | (see `FlywheelConstants`) | `FlywheelConstants` |
| Flywheel follower | (see `FlywheelConstants`) | `FlywheelConstants` |
| Hood motor | (see `HoodConstants`) | `HoodConstants` |
| Turret motor | (see `TurretConstants`) | `TurretConstants` |

## Intake / Hopper / Kicker

| Device | CAN ID | Defined in |
| --- | --- | --- |
| Intake pivot | (see `IntakeConstants`) | `IntakeConstants` |
| Intake roller | (see `IntakeConstants`) | `IntakeConstants` |
| Hopper conveyor | (see `HopperConstants`) | `HopperConstants` |
| Kicker | (see `KickerConstants`) | `KickerConstants` |

## Climb

| Device | CAN ID | Defined in |
| --- | --- | --- |
| Climb motor | (see `ClimbConstants`) | `ClimbConstants` |

## Adding a new device

1. Pick a CAN ID that doesn't conflict on the same bus (the rio's CAN
   and the CANivore are separate; CTRE and REV devices can share IDs).
2. Add it to the relevant `*Constants` class.
3. Document it on this page so the next person doesn't have to grep.

## Conflict checklist

Before deploying with a new device:

- [ ] ID is unique within the same CAN bus.
- [ ] Device shows up in REV Hardware Client / Phoenix Tuner X.
- [ ] Firmware is up to date.
- [ ] Brake/coast mode set as expected.
- [ ] Current limit set in `*IOSpark` configuration.
