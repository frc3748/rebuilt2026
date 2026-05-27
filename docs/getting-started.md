---
layout: default
title: Getting Started
eyebrow: Tutorial
description: Get the code on your machine, open it in WPILib VS Code, run the simulator, and see the robot move.
permalink: /getting-started/
---

This page walks you from "I have nothing installed" to a robot you can
drive in the simulator and watch in AdvantageScope. We are not going
to use a command line for git — everything happens through
**GitHub Desktop** (or the GitHub website) and **WPILib VS Code**.

## Prerequisites

Install these once, in this order:

1. **[WPILib 2026](https://github.com/wpilibsuite/allwpilib/releases)** — the FRC installer. It bundles VS Code (a custom build called *WPILib VS Code*), the JDK 17 the toolchain expects, and the FRC simulator + Driver Station. Don't substitute your own JDK or your own VS Code.
2. **[GitHub Desktop](https://desktop.github.com/)** — graphical UI for cloning, pulling, and pushing. If you'd rather use the github.com website, that works too; both flows are described below.
3. **[AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope/releases)** — viewer for live and recorded robot data. Ships with WPILib but installing the standalone release keeps you on a current version.
4. **A GitHub account** — joined to the team's GitHub organisation so you have push access.

Once those are installed, log into GitHub Desktop with the same account you joined the org with.

## Get the code with GitHub

You have two paths. Pick whichever you prefer; they end at the same
place: a folder on your computer containing the robot code.

### Path A — GitHub Desktop

1. Open GitHub Desktop.
2. **File → Clone repository…**
3. Pick the **URL** tab. Paste the repo URL (`https://github.com/frc3748/rebuilt2026.git`) or, if the org is in the dropdown, choose `frc3748/rebuilt2026` from the list.
4. Pick a local path. *Don't* put the folder inside OneDrive, Dropbox, or any sync tool — they fight with VS Code's file watchers.
5. Click **Clone**.

When you want updates from the rest of the team, open GitHub Desktop and click **Fetch origin**, then **Pull origin** if there's something new.

When you want to share your work, GitHub Desktop walks you through:
**Summary** → **Commit to main** → **Push origin**. (For real changes
we use pull requests — see [Working with branches](#working-with-branches)
below.)

### Path B — github.com in the browser

1. Open the repo page on github.com.
2. Click the green **Code** button, then **Open with GitHub Desktop** (which triggers the same clone flow as above). Or **Download ZIP** if you want a one-off snapshot — but you can't push back from a ZIP, so this is rarely what you want.

### Working with branches

For anything beyond fixing a typo, work on a **branch**, not directly on `main`:

1. In GitHub Desktop: **Current Branch** dropdown → **New branch** → give it a name like `your-name/shooter-tuning`.
2. Make your changes in WPILib VS Code, commit them via GitHub Desktop, click **Push origin**.
3. GitHub Desktop will prompt you to **Create Pull Request** — click it. Your browser opens to github.com with the PR ready to fill in.
4. Add a description, request a review, click **Create pull request**.
5. After the review approves, a lead clicks **Squash and merge** on github.com to fold your changes into `main`.

You'll spend more time in GitHub Desktop than in the github.com site.
Use the site for pull requests, code reviews, and issues.

## Open the code in WPILib VS Code

1. Launch **2026 WPILib VS Code** (the custom build — *not* a regular VS Code you might have installed for other classes).
2. **File → Open Folder…** → pick the folder GitHub Desktop cloned to.
3. The bottom-right corner will say "Java" and "Gradle". Wait until the Gradle import finishes — it'll show a spinning loader at the bottom for a minute or two on first open. If you start editing or running before this finishes, IntelliSense will give you wrong answers about generated AdvantageKit classes.

The right way to do almost everything in WPILib VS Code is the
**Command Palette**:

> **Ctrl + Shift + P** (or **Cmd + Shift + P** on macOS) opens the
> command palette. Start typing the name of what you want.

Some commands are also available via **Ctrl + P** (the file-quick-open
palette has a fallthrough for WPILib commands — that's why you can
hit `Ctrl + P` and type `simulate`).

## Add vendor dependencies

Vendor dependencies live in `vendordeps/`. We already have them
committed, but if you ever pull a fresh checkout that's missing them,
or you need to add a new one:

1. Click the **WPILib icon** in the left sidebar (the red hexagon-ish icon below the Extensions icon).
2. Click **WPILib: Manage Vendor Libraries**.
3. Choose **Install new libraries (online)** to add a new one by URL, or **Manage Current Libraries** to update existing.
4. Paste the vendor's URL, click OK, and the JSON drops into `vendordeps/` automatically.

The vendors used by this codebase are listed on the
[Vendor Dependencies]({{ '/reference/vendor-deps/' | relative_url }})
page. If `./gradlew build` ever complains about a missing artifact, this
is the first place to check.

## Deploy from VS Code (Ctrl + P)

With a roboRIO connected (USB, tethered Ethernet, or the radio):

1. Press **Ctrl + Shift + P** (Command Palette).
2. Type `deploy`.
3. Pick **WPILib: Deploy Robot Code**.

That's it. The deploy uses the team number from `.wpilib/wpilib_preferences.json`,
talks to the rio, builds the JAR if it's stale, ships it over, and
restarts the robot program. You'll see progress in the **Terminal**
panel at the bottom.

The same thing also works from **Ctrl + P** (the quick-open palette
handles WPILib commands as a fallthrough) — type `deploy` there if
your hands are already on `Ctrl + P` from navigating files.

## Run the simulator (Ctrl + P)

For sim-only development you don't need a robot:

1. Press **Ctrl + Shift + P** (or **Ctrl + P**).
2. Type `simulate`.
3. Pick **WPILib: Simulate Robot Code**.
4. WPILib pops up a dialog asking which Halsim extensions to enable. Tick **Sim GUI** and **DriverStation** (the one in the list is usually labelled `halsim_ds_socket`). **Sim GUI** alone is not enough — without DriverStation you can't enable the robot.
5. Click OK. Two windows open: the Sim GUI (joystick mappings, field, modules) and the Driver Station.

> **If you're asked "do you want A or B"** during sim launch, that's
> the choice of which halsim extension to use. Pick **the
> DriverStation option** so you get the real DS UI. A plain Sim GUI
> by itself can't enable the robot.

## Use the Driver Station

The Driver Station window controls match state (disabled, autonomous,
teleop) and routes joystick input to the robot.

1. Plug in an Xbox controller (or any HID joystick).
2. In the Driver Station, look at the panel on the left side — it has four numbered joystick slots (0, 1, 2, 3). Drag your controller from the "USB Devices" list on the right into **slot 0** (the first one). The codebase's main controller binding reads from port 0.
3. Up at the top, you'll see four buttons: **TeleOperated**, **Autonomous**, **Practice**, **Test**. Click **TeleOperated** for normal driving, or **Autonomous** to run the selected auto routine.
4. To the right of those, the big **Enable** / **Disable** buttons control match state. Click **Enable** to let the robot start receiving commands.
5. If **Enable** is greyed out, the Driver Station can't see the robot code or the joystick. Check that:
   - The Sim GUI is still running (don't close it).
   - The joystick is in slot 0 (the slot has a green checkmark when populated).
   - "Comms" and "Robot Code" indicators on the left are both green.

To pick which autonomous routine runs, switch to the **Driver Station's
Setup tab** and use the "Game Data" dropdown — but the actual auto
selector for this code lives in **Elastic** or the SmartDashboard
chooser. See [Auto Commands]({{ '/commands/auto-commands/' | relative_url }}).

## Connect AdvantageScope

AdvantageScope is where you watch what the robot is actually doing —
poses, state machines, mechanism positions, the works.

### One-time setup

1. Open AdvantageScope.
2. **AdvantageScope (macOS) / Help (Windows/Linux) → Preferences**. (On Windows it's `Help → Show Preferences…`).
3. Set **roboRIO Address** to `10.37.48.2`. This is the team's static IP — `10.TE.AM.2` where the team number is 3748.
4. Close preferences.

### Connect to the simulator

When the simulator is running on your computer:

1. AdvantageScope → **File → Connect to Robot**.
2. From the dropdown, choose **Default Simulator**. (For real-robot connections, you'd choose **roboRIO** — same dropdown, different option.)

You should see the connection indicator at the bottom turn green and
data start streaming. Drag fields from the left sidebar into the tabs
on the right to plot them.

### Connect to a real robot

Same path, just pick **roboRIO** instead of **Default Simulator** in
the connect dropdown. The team IP you set in preferences is what it
uses.

## Install our custom robot assets

The default AdvantageScope 3D field doesn't know what our robot
looks like. Install the team's custom asset bundle so the field view
shows our actual chassis + mechanisms:

1. In AdvantageScope: **Help → Show Assets Folder**. A file-explorer window opens at the assets directory (something like `~/AppData/Roaming/AdvantageScope/userAssets/` on Windows or `~/Library/Application Support/AdvantageScope/userAssets/` on macOS).
2. Inside that folder, clone the team's asset repo. The fastest path is GitHub Desktop:
   - GitHub Desktop → **File → Clone repository → URL** tab → paste `https://github.com/frc3748/Robot_Rebuilt.git` (the team's asset repo) → **Local Path** set to the AdvantageScope assets folder above → **Clone**.
3. Back in AdvantageScope: **File → Use Assets Folder → reload** (or just restart AdvantageScope).
4. In any 3D Field tab, the **Robot** dropdown will now include `Robot_Rebuilt` — pick it.

The asset bundle defines:

- The chassis model (the static base of the robot).
- Component slots for moveable mechanisms — each slot has a name like `Intake`, `Climb`, `Shooter`, and expects a `Pose3d` value to render.

## What to log to the 3D field

For the components to actually move in AdvantageScope, the robot code
has to log a `Pose3d` (or array of them) under the right key. The
mapping for our asset bundle:

| AdvantageScope component | NetworkTables / AdvantageKit key | Notes |
| --- | --- | --- |
| Robot chassis | `NT/AdvantageKit/RealOutputs/Odometry/Robot` | Already logged by `Drive` — `Pose2d` is auto-promoted to `Pose3d` for the 3D field. |
| Intake | `NT/AdvantageKit/RealOutputs/Intake/ComponentPoses` | Array of `Pose3d` for the four-bar links + roller. |
| Hopper | `NT/AdvantageKit/RealOutputs/Hopper/Roller` | One `Pose3d` for the conveyor angle. |
| Kicker | `NT/AdvantageKit/RealOutputs/Kicker/ComponentPose` | One `Pose3d`. |
| Climb | `NT/AdvantageKit/RealOutputs/Climb/ComponentPoses` | Array of `Pose3d` for the two elevator stages. |
| Turret | `NT/AdvantageKit/RealOutputs/Shooter/Turret/ComponentPose` | One `Pose3d`. |
| Hood | `NT/AdvantageKit/RealOutputs/Shooter/Hood/ComponentPose` | One `Pose3d`. |
| Flywheel (cosmetic spin) | `NT/AdvantageKit/RealOutputs/Shooter/Flywheel/ComponentPose` | One `Pose3d`. |
| Game pieces (fuel) | `NT/AdvantageKit/RealOutputs/MapleSim/Fuel` | Array of `Pose3d` — already logged by `MapleSimPhysics`. |

### Wiring a new component pose

The pattern in the subsystem's `update()` looks like this:

```java
Logger.recordOutput(
    "Intake/ComponentPoses",
    new Pose3d[] {frontLink, rearLink, lowerLink, upperLink, roller});
```

Each `Pose3d` is relative to the **robot origin** (centre of the
chassis at the ground), not the field — AdvantageScope composes them
with the robot's field pose automatically.

### Setting up the 3D Field tab

1. In AdvantageScope, **➕ → 3D Field**.
2. Click the gear icon in the tab. Set:
   - **Field** → `2026 Rebuilt` (or whichever season's field is shipped).
   - **Robot** → `Robot_Rebuilt`.
3. Drag the keys from the table above into the tab's pose slots. The slot names match the components defined in our asset bundle.
4. Save the layout — **File → Export Layout** — and check it into the team's docs so everyone gets the same view.

## Your first change

The fastest "did everything install right?" check:

1. Open `src/main/java/frc/robot/Robot.java`.
2. Find `robotPeriodic()`.
3. Add a line:
   ```java
   Logger.recordOutput("HelloWorld", 42);
   ```
4. Press **Ctrl + Shift + P → WPILib: Simulate Robot Code** (or **Ctrl + P → simulate**).
5. In AdvantageScope, find `HelloWorld` under `NT/AdvantageKit/RealOutputs`.

If you see the value, you're good — your toolchain is wired correctly.

## Next steps

- [Robot Lifecycle]({{ '/architecture/robot-lifecycle/' | relative_url }}) — how `Main`, `Robot`, and `RobotState` fit together.
- [The IO Layer Pattern]({{ '/architecture/io-pattern/' | relative_url }}) — why every subsystem has three IO files.
- [State Machines]({{ '/architecture/state-machines/' | relative_url }}) — the framework every mechanism uses.
- [Drive]({{ '/subsystems/drive/' | relative_url }}) — read this first; every other subsystem mirrors its structure.
