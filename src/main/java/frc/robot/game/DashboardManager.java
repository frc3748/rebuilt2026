package frc.robot.game;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.Set;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import frc.robot.Superstructure;
import frc.robot.commands.ActionCommands;
import frc.robot.commands.AutoAlignToPoseCommand;
import frc.robot.commands.SelfTest;
import frc.robot.commands.autos.AutoRoutine;
import frc.robot.commands.autos.DiagnosticAuto;
import frc.robot.commands.autos.MeasureAuto;
import frc.robot.commands.autos.MeasureFeedforward;
import frc.robot.commands.autos.MeasureSteering;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.HeadingLock;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LogFolder;
import frc.robot.util.cockpit.Cockpit;
import frc.robot.util.cockpit.Cockpit.Check;
import frc.robot.util.cockpit.Cockpit.Level;
import frc.robot.util.cockpit.Cockpit.Marker;
import frc.robot.util.cockpit.Cockpit.Tab;
import frc.robot.util.motor.Motor;
import frc.robot.util.motor.MotorAutoTune;
import frc.robot.util.state.StateMachine;
import frc.robot.util.tuning.Tuning;

public class DashboardManager {
    private static final double kMinBatteryVolts = 12.3;
    private static final double kStartToleranceMeters = 0.05;
    private static final double kStartToleranceDegrees = 2.0;
    private static final double kPlaybackStepSeconds = 0.05;
    private static final int kCatalogPathPoints = 60;
    private static final double kSelfTestHours = 12.0;

    private final RobotState state;
    private final GameState game;
    private final LoggedDashboardChooser<AutoRoutine> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    private AutoRoutine previewed;
    private Optional<Alliance> previewedAlliance = Optional.empty();
    private Pose2d[] previewPath = new Pose2d[0];
    private Playback playback = Playback.EMPTY;
    private final List<AutoRoutine> catalogAutos = new ArrayList<>();
    private String catalog = "[]";
    private Optional<Alliance> catalogAlliance = Optional.empty();
    private boolean catalogBuilt;
    private Optional<Pose2d> startPose = Optional.empty();
    private boolean followingPath;
    private boolean enabledOnce;
    private boolean autoTuning;
    private final DisconnectNotifier disconnects;

    public DashboardManager(RobotState state, GameState game, String robotName, List<AutoRoutine> autos) {
        this.state = state;
        this.game = game;
        disconnects = new DisconnectNotifier(state::disconnectedDevices, Set.of(state.getDrive().gyroName()));

        autoChooser.addDefaultOption("None", AutoRoutine.none());
        for (String name : AutoBuilder.getAllAutoNames()) {
            AutoRoutine auto = AutoRoutine.pathPlanner(name);
            autoChooser.addOption(name, auto);
            catalogAutos.add(auto);
        }
        for (AutoRoutine auto : autos) {
            autoChooser.addOption(auto.name(), auto);
            catalogAutos.add(auto);
        }

        PathPlannerLogging.setLogActivePathCallback(poses -> {
            followingPath = !poses.isEmpty();
            Logger.recordOutput("Odometry/Trajectory", poses.toArray(Pose2d[]::new));
        });

        Logger.recordOutput("Cockpit/Robot", robotName);
        registerButtons();
        registerTuning();
        registerChecks();
        registerMarkers();
    }

    public AutoRoutine getSelectedAuto() {
        AutoRoutine selected = autoChooser.get();
        return selected != null ? selected : AutoRoutine.none();
    }

    public void update() {
        if (DriverStation.isEnabled() && !DriverStation.isTest()) {
            enabledOnce = true;
        }
        previewSelectedAuto();
        disconnects.update();
        if (DriverStation.isDisabled() && (!catalogBuilt || !DriverStation.getAlliance().equals(catalogAlliance))) {
            catalogAlliance = DriverStation.getAlliance();
            catalog = buildCatalog();
            catalogBuilt = true;
        }
        Logger.recordOutput("Cockpit/Autos", catalog);
        Logger.recordOutput("Cockpit/Diagnostics", DiagnosticAuto.results());
        Logger.recordOutput("Cockpit/Measurements", MeasureAuto.results());
        Logger.recordOutput("Cockpit/AutoTune", MotorAutoTune.results());

        Logger.recordOutput("Game/Phase", game.getPhase());
        Logger.recordOutput("Game/HubActive", game.isHubActive());
        Logger.recordOutput("Game/HubActiveNext", game.isHubActiveNext());
        Logger.recordOutput("Game/Timeline", game.getTimeline());
        Logger.recordOutput("Game/WonAuto", game.wonAuto());
        Logger.recordOutput("Game/SecondsToShift", game.getSecondsUntilShift());
        Logger.recordOutput("Game/DistanceToHub", TrenchZone.getDistanceToClosestShootingPose(state));

        Logger.recordOutput("Cockpit/Auto/Name", getSelectedAuto().name());
        Logger.recordOutput("Cockpit/Auto/Path", previewPath);
        Logger.recordOutput("Cockpit/Auto/Start", startPose.stream().toArray(Pose2d[]::new));
        Logger.recordOutput("Cockpit/Auto/Playback/Poses", playback.poses());
        Logger.recordOutput("Cockpit/Auto/Playback/Times", playback.times());
        Logger.recordOutput("Cockpit/Auto/Playback/Speeds", playback.speeds());
        Logger.recordOutput("Cockpit/Auto/Playback/PathStarts", playback.pathStarts());
        Logger.recordOutput("Cockpit/Auto/Playback/PathNames", playback.pathNames());
        if (DriverStation.isAutonomousEnabled()) {
            Logger.recordOutput("Cockpit/Auto/Step", autoStep());
            Pose2d robot = state.getLatestFieldToRobot().getValue();
            double drift = followingPath
                    ? state.getDrive().getPathTarget().map(target -> target.getTranslation().getDistance(robot.getTranslation())).orElse(0.0)
                    : 0.0;
            Logger.recordOutput("Cockpit/Auto/DriftMeters", drift);
        }
    }

    public void clearPreview() {
        previewed = null;
    }

    private void previewSelectedAuto() {
        AutoRoutine selected = getSelectedAuto();
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (selected == previewed && alliance.equals(previewedAlliance)) {
            return;
        }
        previewed = selected;
        previewedAlliance = alliance;

        List<Pose2d> poses = new ArrayList<>();
        startPose = Optional.empty();
        for (PathPlannerPath path : selected.previewPaths()) {
            if (startPose.isEmpty()) {
                startPose = path.getStartingHolonomicPose().map(AllianceFlip::forAlliance);
            }
            for (Pose2d pose : path.getPathPoses()) {
                poses.add(AllianceFlip.forAlliance(pose));
            }
        }
        previewPath = poses.toArray(Pose2d[]::new);
        playback = Playback.of(selected.previewPaths(), state.getDrive().getPathPlannerConfig());
    }

    private String buildCatalog() {
        StringBuilder json = new StringBuilder("[");
        for (AutoRoutine auto : catalogAutos) {
            List<PathPlannerPath> paths = auto.previewPaths();
            Optional<Pose2d> start = paths.isEmpty() ? Optional.empty()
                    : paths.get(0).getStartingHolonomicPose().map(AllianceFlip::forAlliance);
            if (start.isEmpty()) {
                continue;
            }
            List<Pose2d> poses = new ArrayList<>();
            for (PathPlannerPath path : paths) {
                path.getPathPoses().forEach(pose -> poses.add(AllianceFlip.forAlliance(pose)));
            }
            int step = Math.max(1, poses.size() / kCatalogPathPoints);
            if (json.length() > 1) {
                json.append(',');
            }
            json.append("{\"name\":").append(quote(auto.name()))
                    .append(",\"mode\":").append(quote(auto.mode()))
                    .append(",\"start\":").append(point(start.get())).append(",\"path\":[");
            for (int index = 0; index < poses.size(); index += step) {
                json.append(index > 0 ? "," : "").append(point(poses.get(index)));
            }
            json.append("]}");
        }
        return json.append(']').toString();
    }

    private static String point(Pose2d pose) {
        return String.format(Locale.ROOT, "[%.3f,%.3f,%.1f]", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

    private static String quote(String text) {
        return "\"" + text.replace("\\", "\\\\").replace("\"", "\\\"") + "\"";
    }

    private record Playback(Pose2d[] poses, double[] times, double[] speeds, double[] pathStarts, String[] pathNames) {
        static final Playback EMPTY = new Playback(new Pose2d[0], new double[0], new double[0], new double[0], new String[0]);

        static Playback of(List<PathPlannerPath> paths, RobotConfig config) {
            List<Pose2d> poses = new ArrayList<>();
            List<Double> times = new ArrayList<>();
            List<Double> speeds = new ArrayList<>();
            double[] starts = new double[paths.size()];
            String[] names = new String[paths.size()];
            double offset = 0.0;
            for (int index = 0; index < paths.size(); index++) {
                PathPlannerPath path = paths.get(index);
                Rotation2d heading = path.getStartingHolonomicPose().map(Pose2d::getRotation).orElse(Rotation2d.kZero);
                PathPlannerTrajectory trajectory = path.generateTrajectory(new ChassisSpeeds(), heading, config);
                double total = trajectory.getTotalTimeSeconds();
                starts[index] = offset;
                names[index] = path.name;
                for (double time = 0.0; time < total + kPlaybackStepSeconds; time += kPlaybackStepSeconds) {
                    PathPlannerTrajectoryState sample = trajectory.sample(Math.min(time, total));
                    poses.add(AllianceFlip.forAlliance(sample.pose));
                    times.add(offset + Math.min(time, total));
                    speeds.add(sample.linearVelocity);
                }
                offset += total;
            }
            return new Playback(poses.toArray(Pose2d[]::new), times.stream().mapToDouble(Double::doubleValue).toArray(),
                    speeds.stream().mapToDouble(Double::doubleValue).toArray(), starts, names);
        }
    }

    private String autoStep() {
        List<String> parts = new ArrayList<>();
        if (followingPath) {
            parts.add("Following path");
        }
        Superstructure robot = state.getSuperstructure();
        if (robot.getIntake().map(intake -> intake.getState() == Intake.State.INTAKE).orElse(false)) {
            parts.add("Intaking");
        }
        if (robot.getShooter().map(Shooter::isFiring).orElse(false)) {
            parts.add("Shooting");
        }
        return parts.isEmpty() ? "Waiting" : String.join(" · ", parts);
    }

    private void registerButtons() {
        Superstructure robot = state.getSuperstructure();
        Cockpit.toggleButton("hubShot", "Hub-front shot", Tab.TELEOP, ActionCommands.toggleFixedShot(state),
                () -> robot.getShooter().map(Shooter::isHoldingShot).orElse(false));
        Cockpit.toggleButton("vision", "Vision", Tab.TEST,
                disabledSafe(() -> state.getVision().setEstimating(!state.getVision().isEstimating())),
                state.getVision()::isEstimating);
        Cockpit.button("stow", "Clear overrides & stow", Tab.TELEOP, Commands.runOnce(() -> {
            robot.clearOverrides();
            robot.getIntake().ifPresent(intake -> intake.requestTransition(Intake.State.STOW));
        }));
        Cockpit.button("selfTest", "Run self-test", Tab.TEST,
                Commands.either(Commands.defer(() -> SelfTest.build(state), Set.of(state.getDrive())).asProxy(),
                        disabledSafe(() -> Cockpit.toast(Level.WARNING, "Can't run the self-test",
                                autoTuning ? "Wait for auto-tune to finish" : "Enable Test mode with the robot on a cart")),
                        () -> DriverStation.isTest() && DriverStation.isEnabled() && !autoTuning));
        Cockpit.confirmButton("zeroHood", "Zero hood", Tab.TEST, disabledSafe(robot.shooterAction(Shooter::zero)));
        Cockpit.confirmButton("zeroIntake", "Zero intake", Tab.TEST, disabledSafe(robot.intakeAction(Intake::zero)));
        Cockpit.confirmButton("zeroHeading", "Zero heading", Tab.TEST, disabledSafe(state.getDrive()::zeroHeading));
        Cockpit.button("resetMultiplier", "Reset flywheel multiplier", Tab.TEST, disabledSafe(robot.shooterAction(Shooter::resetMultiplier)));
        if (Constants.kMode == Mode.SIM) {
            Cockpit.button("simStart", "Put robot on start pose", Tab.PREMATCH,
                    disabledSafe(() -> startPose.ifPresent(state.getDrive()::setPose)));
        }
    }

    private void registerMarkers() {
        Cockpit.marker("driveTo", "Drive to", Marker.TARGET, AutoAlignToPoseCommand::activeTarget);
        Cockpit.marker("hub", "Hub", Marker.AIM, () -> state.shouldShootHub() ? Optional.of(state.getDriveAnglePos()) : Optional.empty());
        Cockpit.marker("pass", "Pass", Marker.AIM, () -> state.shouldShootHub() ? Optional.empty() : Optional.of(state.getDriveAnglePos()));
        Cockpit.marker("fuel", "Fuel", Marker.POINT, () -> state.getVision().getClosestObjectPose());
        Cockpit.marker("aimPoint", "Aiming here", Marker.CROSSHAIR, () -> shooterWorking()
                ? state.getDrive().getRecentAimTarget().map(target -> new Pose2d(target, Rotation2d.kZero))
                : Optional.empty());
        Cockpit.zone("trench", "Hood down", () -> TrenchZone.hoodLowerRequired(state)
                ? Optional.of(new Pose2d(TrenchZone.closestTrench(state), Rotation2d.kZero))
                : Optional.empty(), TrenchZone::hoodLowerRadius);
        Cockpit.assist("align", "Auto-align", () -> AutoAlignToPoseCommand.activeTarget().isPresent());
        Cockpit.assist("aim", "Aim lock", () -> state.getDrive().getState() == Drive.State.TRAVERSING_AT_ANGLE);
        Cockpit.assist("path", "Path", () -> followingPath);
        Cockpit.assist("heading", "Heading lock", () -> HeadingLock.isEngaged() && DriverStation.isTeleopEnabled());
    }

    private boolean shooterWorking() {
        return state.getSuperstructure().getShooter()
                .map(shooter -> shooter.getState() != Shooter.State.IDLE && shooter.getState() != Shooter.State.UNDETERMINED)
                .orElse(false);
    }

    private void registerTuning() {
        Superstructure robot = state.getSuperstructure();
        Cockpit.toggleButton("tuning", "Tuning mode", Tab.TUNE, disabledSafe(() -> Tuning.setEnabled(!Tuning.isEnabled())),
                Tuning::isEnabled);
        Cockpit.button("tuneSave", "Save", Tab.TUNE, disabledSafe(() -> {
            if (!Tuning.isActive()) {
                Cockpit.toast(Level.WARNING, "Turn on tuning mode first", DriverStation.isFMSAttached() ? "Tuning is off while the FMS is attached" : "");
                return;
            }
            int saved = Tuning.save();
            Cockpit.toast(saved > 0 ? Level.INFO : Level.WARNING, saved > 0 ? "Saved " + saved + " on the robot" : "Nothing changed",
                    saved > 0 ? "Deploy and they're written into the code" : "");
        }));
        Cockpit.button("tuneRevert", "Revert", Tab.TUNE, disabledSafe(() -> {
            int reverted = Tuning.revert();
            Cockpit.toast(Level.INFO, reverted > 0 ? "Reverted " + reverted : "Nothing to revert", "");
        }));
        Cockpit.confirmButton("tuneForget", "Forget saved", Tab.TUNE, disabledSafe(() -> {
            int forgotten = Tuning.forget();
            Cockpit.toast(Level.INFO, forgotten > 0 ? "Forgot " + forgotten + " saved" : "Nothing was saved", "Back to the values in the code");
        }));
        registerAutoTune();
        Cockpit.toggleButton("tuneShooter", "Spin to custom setpoint", Tab.TUNE, Commands.runOnce(robot.shooterAction(shooter ->
                shooter.requestTransition(shooter.getState() == Shooter.State.TUNING ? Shooter.State.IDLE : Shooter.State.TUNING))),
                () -> robot.getShooter().map(shooter -> shooter.getState() == Shooter.State.TUNING).orElse(false));
        Cockpit.toggleButton("tuneIntake", "Intake out", Tab.TUNE, Commands.runOnce(robot.intakeAction(intake ->
                intake.requestTransition(intake.getState() == Intake.State.INTAKE ? Intake.State.STOW : Intake.State.INTAKE))),
                () -> robot.getIntake().map(intake -> intake.getState() == Intake.State.INTAKE).orElse(false));
    }

    private void registerAutoTune() {
        List<StateMachine<?>> machines = new ArrayList<>();
        state.getSuperstructure().subsystems().forEach(machine -> collectMechanisms(machine, machines));
        for (StateMachine<?> machine : machines) {
            for (Motor motor : machine.getMotors()) {
                Cockpit.quietConfirmButton("autotune:" + MotorAutoTune.group(motor), "Auto-tune", Tab.TUNE,
                        afterSelfTest(tracked(MotorAutoTune.build(machine, motor))));
            }
        }
        for (String group : List.of("Drive PID", "Drive Sim")) {
            Cockpit.quietConfirmButton("autotune:" + group, "Auto-tune", Tab.TUNE,
                    afterSelfTest(inTestMode(tracked(measuring(group,
                            Commands.defer(() -> new MeasureFeedforward(state).build(), Set.of(state.getDrive())))))));
        }
        for (String group : List.of("Turn PID", "Turn Sim")) {
            Cockpit.quietConfirmButton("autotune:" + group, "Auto-tune", Tab.TUNE,
                    afterSelfTest(inTestMode(tracked(measuring(group,
                            Commands.defer(() -> new MeasureSteering(state).build(), Set.of(state.getDrive())))))));
        }
    }

    private Command tracked(Command command) {
        return command.beforeStarting(() -> autoTuning = true).finallyDo(() -> autoTuning = false);
    }

    private static Command measuring(String group, Command command) {
        return command
                .beforeStarting(() -> {
                    Logger.recordOutput("AutoTune/Active", true);
                    Logger.recordOutput("AutoTune/Motor", group);
                    Logger.recordOutput("AutoTune/Step", "Measuring");
                })
                .finallyDo(() -> {
                    Logger.recordOutput("AutoTune/Active", false);
                    Logger.recordOutput("AutoTune/Step", "");
                });
    }

    private static Command inTestMode(Command command) {
        return Commands.either(command,
                Commands.runOnce(() -> Cockpit.toast(Level.WARNING, "Can't auto-tune the drive",
                        Tuning.isActive() ? "Enable Test mode first" : "Turn on tuning mode first")),
                () -> DriverStation.isTest() && DriverStation.isEnabled() && Tuning.isActive());
    }

    private static Command afterSelfTest(Command command) {
        return Commands.either(
                Commands.runOnce(() -> Cockpit.toast(Level.WARNING, "Can't auto-tune yet", "Wait for the self-test to finish")),
                command.asProxy(), SelfTest::isRunning);
    }

    private static void collectMechanisms(StateMachine<?> machine, List<StateMachine<?>> machines) {
        if (!machine.getMotors().isEmpty()) {
            machines.add(machine);
        }
        machine.getChildSubsystems().forEach(child -> collectMechanisms(child, machines));
    }

    private static Command disabledSafe(Runnable action) {
        return Commands.runOnce(action).ignoringDisable(true);
    }

    private void registerChecks() {
        Cockpit.check("logging", "Logging", () -> LogFolder.isUsb() ? Check.pass("Logging to the USB stick")
                : Check.warn("No USB stick in the roboRIO. Logs go to its own storage, which keeps only the newest 150 MB."));
        Cockpit.check("tuning", "Tuning", () -> Tuning.isEnabled() ? Check.warn("Tuning mode is on")
                : Tuning.pending() > 0 ? Check.warn(Tuning.pending() + " tuned on the robot but not in the code yet. Deploy to write them in.")
                        : Check.pass("Every value is in the code"));
        Cockpit.gauge("distance", "To hub", "m", () -> TrenchZone.getDistanceToClosestShootingPose(state),
                this::shooterWorking);
        Cockpit.check("battery", "Battery voltage", this::checkBattery);
        Cockpit.check("batteryPicked", "Battery picked", () -> state.getBattery().isPicked()
                ? Check.pass(state.getBattery().getId())
                : Check.warn("Pick the battery that's in the robot"));
        Cockpit.check("auto", "Auto picked", this::checkAuto);
        Cockpit.check("startPose", "On the start pose", this::checkStartPose);
        Cockpit.check("heading", "Heading confirmed", this::checkHeading);
        Cockpit.check("devices", "Everything connected", () -> {
            List<String> missing = state.disconnectedDevices();
            return missing.isEmpty() ? Check.pass("All connected") : Check.fail(String.join(", ", missing) + " disconnected");
        });
        Cockpit.check("controllers", "Controllers", this::checkControllers);
        Cockpit.check("selfTest", "Self-test", this::checkSelfTest);
    }

    private Check checkBattery() {
        if (Constants.kMode == Mode.SIM) {
            return Check.pass("Simulated");
        }
        double volts = RobotController.getBatteryVoltage();
        String text = String.format("%.2f V", volts);
        if (enabledOnce || volts >= kMinBatteryVolts) {
            return Check.pass(text);
        }
        return Check.fail(text + ", swap it for one above " + kMinBatteryVolts + " V");
    }

    private Check checkAuto() {
        String name = getSelectedAuto().name();
        if ("None".equals(name)) {
            return Check.warn("No auto picked");
        }
        if (!name.toLowerCase().contains("game")) {
            return Check.warn(name + " isn't a match auto");
        }
        return Check.pass(name);
    }

    private Check checkStartPose() {
        if (enabledOnce) {
            return Check.pass("Match started");
        }
        if (startPose.isEmpty()) {
            return Check.pass("This auto has no start pose");
        }
        if (!state.getVision().isHeadingConfirmed()) {
            return Check.fail("Waiting for vision to place the robot");
        }
        Pose2d robot = state.getLatestFieldToRobot().getValue();
        Pose2d start = startPose.get();
        Translation2d offset = start.getTranslation().minus(robot.getTranslation());
        double turn = start.getRotation().minus(robot.getRotation()).getDegrees();
        Logger.recordOutput("Cockpit/Auto/StartErrorMeters", offset.getNorm());
        Logger.recordOutput("Cockpit/Auto/StartErrorDegrees", turn);
        String summary = String.format("%.0f cm · %.1f°", offset.getNorm() * 100, Math.abs(turn));
        if (offset.getNorm() <= kStartToleranceMeters && Math.abs(turn) <= kStartToleranceDegrees) {
            return Check.pass(summary);
        }
        return Check.fail(startAdvice(offset, turn, DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red));
    }

    public static String startAdvice(Translation2d offset, double turnDegrees, boolean red) {
        double forward = red ? -offset.getX() : offset.getX();
        double left = red ? -offset.getY() : offset.getY();
        List<String> moves = new ArrayList<>();
        if (Math.abs(left) > kStartToleranceMeters / 2) {
            moves.add(String.format("%.0f cm %s", Math.abs(left) * 100, left > 0 ? "left" : "right"));
        }
        if (Math.abs(forward) > kStartToleranceMeters / 2) {
            moves.add(String.format("%.0f cm %s", Math.abs(forward) * 100, forward > 0 ? "forward" : "back"));
        }
        String text = moves.isEmpty() ? "" : "Move " + String.join(" and ", moves);
        if (Math.abs(turnDegrees) > kStartToleranceDegrees / 2) {
            String turn = String.format("turn %.0f° %s", Math.abs(turnDegrees), turnDegrees > 0 ? "counterclockwise" : "clockwise");
            text = text.isEmpty() ? Character.toUpperCase(turn.charAt(0)) + turn.substring(1) : text + ", " + turn;
        }
        return text;
    }

    private Check checkHeading() {
        if (enabledOnce || state.getVision().isHeadingConfirmed()) {
            return Check.pass(enabledOnce && !state.getVision().isHeadingConfirmed() ? "Match started" : "Confirmed by MegaTag 1");
        }
        return Check.fail("Waiting for MegaTag 1 to see 2 or more tags up close");
    }

    private Check checkSelfTest() {
        if (enabledOnce) {
            return Check.pass("Not needed after the first enable");
        }
        if (SelfTest.hasPassed()) {
            return Check.pass("Passed");
        }
        if (!SelfTest.failures().isEmpty()) {
            return Check.fail("Failed: " + String.join(", ", SelfTest.failures()));
        }
        boolean field = DriverStation.isFMSAttached();
        Optional<SelfTest.Saved> saved = SelfTest.saved();
        if (saved.isPresent() && saved.get().passed() && saved.get().sameCode() && saved.get().hoursAgo() < kSelfTestHours) {
            return Check.pass("Passed in the pits " + hoursText(saved.get().hoursAgo()) + " ago");
        }
        if (saved.isPresent() && !saved.get().passed() && saved.get().sameCode()) {
            String text = "Failed in the pits: " + String.join(", ", saved.get().failures());
            return field ? Check.warn(text) : Check.fail(text);
        }
        if (field) {
            return Check.warn("Not run on this code. It can't run on the field, so run it in the pits next");
        }
        return Check.fail(saved.isPresent() && !saved.get().sameCode()
                ? "New code since the last self-test. In Test mode, press Run self-test with the robot on a cart"
                : "In Test mode, press Run self-test with the robot on a cart");
    }

    private Check checkControllers() {
        List<String> missing = new ArrayList<>();
        if (!DriverStation.isJoystickConnected(0)) {
            missing.add("Driver");
        }
        if (!DriverStation.isJoystickConnected(1)) {
            missing.add("Operator");
        }
        return missing.isEmpty() ? Check.pass("Both plugged in") : Check.warn(String.join(" and ", missing) + " not plugged in");
    }

    private static String hoursText(double hours) {
        return hours < 1 ? Math.max(1, Math.round(hours * 60)) + " min" : String.format(Locale.ROOT, "%.1f h", hours);
    }
}
