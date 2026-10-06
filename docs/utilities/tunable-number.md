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

## Auto-tune

Every motor group on the Tune tab has an **Auto-tune** card. Press it
twice and the robot measures that motor and proposes its gains, the same
way you'd tune it by hand: find the feedforward, then raise kP until it
just stops overshooting.

It only runs with tuning mode on and the robot enabled in **Test** mode
(on a cart, or with room for the mechanism to move). Otherwise it toasts
why and does nothing.

- **Flywheels and rollers** (`SpinMotor`): ramps to 6 V at 2 V/s, coasts,
  steps 4 V, and stops. It fits volts = kS + kV·speed + kA·acceleration
  to every sample, then starts kP from the fit. To check kP it spins at
  half speed and adds a 2 V load. kP is raised ×1.5 until the speed dips
  less than 3%, and goes back to the last good kP if the speed overshoots
  more than 10% or oscillates.
- **Arms, hoods and the intake** (`PosMotor`): only inside the motor's
  travel range, from `tuneRange(min, max)` or else its soft limits. A
  motor without either has no button. It moves to the middle, ramps
  slowly up and down at 0.5 V/s, and pulses ±3 V for at most 0.15 s near
  each end, staying 12% away from the limits. The fit adds kG (constant)
  or kCos (`cosineGravity`). Then it steps between two points and raises
  kP ×1.5 until it settles within 2% of the move, up to 7 rounds. If it
  overshoots more than 10% or oscillates, it goes back to the last kP
  that didn't. A MAXMotion profile faster than 80%
  of what the motor can actually do is slowed to that.
- **Drive PID, Drive Sim, Turn PID, Turn Sim** run the
  [Measure autos]({{ '/commands/autos/' | relative_url }}#measuring-autos)
  (Drive feedforward or Steering), so drive and turn motors get tuned too.

While it runs, the other motors in the same mechanism hold still and the
motor's current limit drops to 30 A (or its own limit, if lower). It
stops and puts everything back if the motor moves more than 2% past the
range, or stalls for 0.5 s while being pushed. When it finishes, the
proposed values show on the Tune tab like any edit. **Save** keeps them;
**Revert** drops them. The card shows each step while it runs, then the
result.

- **Logs:** `AutoTune/Active`, `AutoTune/Motor`, `AutoTune/Step`, and
  `AutoTune/<motor>/…` for every proposed gain, round and overshoot.
  `Cockpit/AutoTune` holds the last result for each group.
- **What it can't tune:** a gain the `MotorConfig` never declares. A
  roller with no `.feedforward(...)` only gets kP; add
  `.feedforward(0, 0, 0)` and Auto-tune fills it in.
- **Simulator:** `MotorAutoTuneTest` runs it on COMP in maple-sim. The
  flywheel kV has to match the motor's physics within 5%, the hood kG and
  intake kCos have to come out within 0.06 V, and every roller has to get
  a kP. On the real robot, watch the first run with a hand on the disable
  button.

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

- Each value is published at `/Tunable/<key>`.
- `Tuning/Catalog` lists every value with its code default and saved value, logged when it changes. A value is logged under `NetworkInputs/Tunable/…` only once it differs from its code default, and NetworkTables is only read while tuning mode is on, so 160-odd tunables cost nothing per loop in a match.
- The Tune tab reads `Tuning/Catalog`, `Tuning/Enabled`,
  `Tuning/Active` and `Tuning/Pending`.

robotTools still lists tunables that ended a run away from their code
default.
