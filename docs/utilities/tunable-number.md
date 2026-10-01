---
layout: default
title: Tuning
eyebrow: Utilities
description: Tuning mode, live values on the real robot, Save, and the Gradle task that writes tuned values back into the code.
permalink: /utilities/tunable-number/
---

Every number you'd tune on the real robot is a `TunableNumber`. This
covers:

- every motor's gains and current limit
- setpoints and speeds
- the shot table
- drive, heading-lock, aim and auto-align gains
- vision trust

Turn on **tuning mode** from the Tune tab in robotTools Drive, edit
values while the robot runs, press **Save**, and the next
`./gradlew deploy` writes the saved values into the right Java files
for you.

## The workflow

1. In robotTools Drive, open **Tune** and press **Tuning mode**. A
   "Tuning on" light stays in the top bar on every tab.
2. Pick a group (Flywheel, Hood, Drive, Shot Table, …) and change
   values:
   - Type a number and press Enter, or use the − / + buttons. Shift
     steps 10×, and the arrow keys work too.
   - Changes apply live and show in amber.
   - Groups with a motor show a plot of goal against measured, with
     current.
   - **Spin to custom setpoint** runs the shooter's TUNING state, using
     Flywheel/Custom Setpoint and Hood/Custom Setpoint.
   - **Intake out** swings the intake.
3. Press **Save**:
   - The changed values are kept on the robot in
     `/home/lvuser/tuning.json`, or `build/tuning-sim.json` in the
     simulator.
   - They survive a reboot and apply even with tuning mode off.
   - **Revert** puts unsaved edits back.
   - **Forget saved** drops everything saved and goes back to the code.
4. Deploy. `./gradlew deploy` runs `pullTuning` first: it copies the
   saved values off the robot over FTP and edits the Java files, then
   compiles and deploys. It prints every change and anything it left
   for you. Commit the edits.

On the next boot the robot sees that the saved values match the code
and drops them. Until then the pre-match check says how many tuned
values aren't in the code yet.

Tuning mode does nothing while the FMS is attached: matches always run
the code plus whatever was saved.

`./gradlew pullTuning` does step 4 without deploying. If no robot is
reachable, it uses the values saved in the simulator instead; `-Psim`
forces that. Deploy only takes values from the robot, and says so if
the simulator has some waiting. Use `-ProbotHost=10.37.48.2` to pick an
address, and `-PskipTuning` to deploy without pulling.

## Where Save writes each value

The robot records where every value came from: the file and line of the
call (`.pid(0.5, 0, 0)`, `addShot(...)`,
`new TunableNumber("Key", 0.8)`), or the field it was read from
(`driveKp`). `pullTuning` uses that to find the number.

| The value lives in | Tuned on | Written as |
| --- | --- | --- |
| The robot's own folder (`robots/comp/CompDrive.java`) | That robot | The number edited in place |
| A shared file (`FlywheelConstants`, `DriveConfig`, …) | COMP | The number edited in place, since shared values are COMP's |
| A shared file, or COMP's folder | SECONDARY or PRACTICE | `Tuning.override("Key", value)` in that robot's class |

That matches how the robots inherit: SECONDARY takes COMP's values and
only stores what differs. An override looks like this, and
`pullTuning` adds the method and import if they aren't there:

```java
@Override
public void tune() {
    super.tune();
    Tuning.override("Flywheel/kP", 0.6);
}
```

What it can edit:

- **A plain number:** edited directly.
- **A unit call** (`Units.degreesToRadians(1.5)`, `Math.toRadians`,
  `Units.inchesToMeters`): the number inside is converted.
- **A number times a named constant from the same file**
  (`0.006 * kWheelRadius`): the number is scaled.

Anything else, like `FieldConstants.FUNNEL_RADIUS.in(Inches)`, is left
alone and printed with the value to type in by hand. If someone changed
a number in the code after it was tuned, it's also left alone and
printed. Running it twice changes nothing the second time.

## `TunableNumber`

```java
new TunableNumber("Trench/Hood Down Radius", 0.8);              // literal: records this line
TunableNumber.field("Flywheel/Speed Tolerance", constants, "speedTolerance");  // reads the field, records the class chain
new TunableNumber(key, value, source);                          // explicit source (shot table, camera factor)

tunable.get();                         // the live value in tuning mode, otherwise saved or code
tunable.onChange(controller::setP);    // runs on every change, and once at startup if a saved value differs
tunable.integer();                     // rounds (current limits)
tunable.degrees();                     // stored in radians, shown in degrees on the Tune tab
tunable.restartToApply();              // read once at startup (path PID); the Tune tab marks it
```

Use `get()` where the value is read each loop. Use `onChange` where it
has to be pushed somewhere, like a controller or a motor config.
`Robot.robotPeriodic()` calls `TunableNumber.pollAll()` first thing,
which also publishes the catalog the Tune tab reads.

Motor gains need no code. Every gain you write in a `MotorConfig` is
tunable under `<motor name>/<gain>`: the calls are `.pid`,
`.feedforward`, `.gravity` / `.cosineGravity`, `.maxMotion` and
`.currentLimit`. The Spark, Talon and sim IO apply changes live. A gain
you never wrote, like a `kG` on a flywheel, isn't tunable; add it to the
`MotorConfig` first.

## Logs

- Each value is published at `/Tunable/<key>` and logged under
  `NetworkInputs/Tunable/…`.
- Each value's code default is logged once under `TunableDefaults/…`.
- The Tune tab reads `Tuning/Catalog`, `Tuning/Enabled`,
  `Tuning/Active` and `Tuning/Pending`.

robotTools still lists tunables that ended a run away from their code
default.
