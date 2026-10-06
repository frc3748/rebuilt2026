package frc.robot.commands.autos;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.commands.FollowPath;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConfig;
import frc.robot.util.cockpit.Cockpit;
import frc.robot.util.cockpit.Cockpit.Level;

public abstract class DiagnosticAuto extends AutoRoutine {
    public static final double kPassMeters = 0.05;
    public static final double kPassDegrees = 3.0;
    protected static final double kFoot = Units.feetToMeters(1.0);

    private static final double kTestSpeed = 1.5;
    private static final double kTestAcceleration = 2.0;
    private static final double kAutoSpeed = 4.0;
    private static final double kAutoAcceleration = 3.5;
    private static final double kSettleSeconds = 0.5;
    private static final double kTurnToleranceDegrees = 1.0;
    private static final double kTurnTimeoutSeconds = 4.0;
    private static final double kSamePlaceMeters = 0.02;
    private static final Map<String, String> results = new LinkedHashMap<>();
    private static String[] resultLines = new String[0];

    public record Result(boolean passed, double errorMeters, double errorDegrees, double trackingMeters) {}

    protected record Leg(double forward, double left, double degrees, boolean turnInPlace, boolean autoSpeed) {}

    protected final RobotState state;
    private final String id;
    private Pose2d start = Pose2d.kZero;
    private double worstTracking;
    private Result lastResult;

    protected DiagnosticAuto(RobotState state, String id, String name) {
        super(name);
        this.state = state;
        this.id = id;
        mode("Diagnostic");
    }

    protected abstract List<Leg> legs();

    protected static Leg drive(double forward, double left, double degrees) {
        return new Leg(forward, left, degrees, false, false);
    }

    protected static Leg atAutoSpeed(double forward, double left, double degrees) {
        return new Leg(forward, left, degrees, false, true);
    }

    protected static Leg turn(double degrees) {
        return new Leg(0.0, 0.0, degrees, true, false);
    }

    public static String[] results() {
        return resultLines;
    }

    public Result lastResult() {
        return lastResult;
    }

    @Override
    public final Command build() {
        Drive drive = state.getDrive();
        List<Leg> legs = legs();
        List<Command> steps = new ArrayList<>();
        steps.add(Commands.runOnce(() -> {
            start = drive.getPose();
            worstTracking = 0.0;
        }));
        for (Leg leg : legs) {
            steps.add(Commands.defer(() -> leg.turnInPlace() ? turnTo(target(leg).getRotation()) : follow(leg), Set.of(drive)));
        }
        steps.add(Commands.runOnce(drive::stop, drive));
        steps.add(Commands.waitSeconds(kSettleSeconds));
        steps.add(Commands.runOnce(() -> report(target(legs.get(legs.size() - 1)))));
        return Commands.sequence(steps.toArray(Command[]::new)).withName(name());
    }

    private Pose2d target(Leg leg) {
        return start.transformBy(new Transform2d(leg.forward(), leg.left(), Rotation2d.fromDegrees(leg.degrees())));
    }

    private Command follow(Leg leg) {
        Drive drive = state.getDrive();
        DriveConfig config = drive.getConfig();
        Pose2d from = drive.getPose();
        Pose2d to = target(leg);
        Translation2d delta = to.getTranslation().minus(from.getTranslation());
        if (delta.getNorm() < kSamePlaceMeters) {
            return turnTo(to.getRotation());
        }
        Rotation2d travel = delta.getAngle();
        double speed = Math.min(leg.autoSpeed() ? kAutoSpeed : kTestSpeed, config.autoSpeedLimit());
        double acceleration = Math.min(leg.autoSpeed() ? kAutoAcceleration : kTestAcceleration, config.autoMaxAcceleration);
        PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(new Pose2d(from.getTranslation(), travel), new Pose2d(to.getTranslation(), travel)),
                new PathConstraints(speed, acceleration, config.autoTurnLimit(), config.maxAngularAcceleration()),
                new IdealStartingState(0.0, from.getRotation()),
                new GoalEndState(0.0, to.getRotation()));
        path.preventFlipping = true;
        FollowPath follower = drive.followPath(path);
        return follower.deadlineFor(Commands.run(() -> worstTracking = Math.max(worstTracking, follower.crossTrackMeters())));
    }

    private Command turnTo(Rotation2d goal) {
        Drive drive = state.getDrive();
        DriveConfig config = drive.getConfig();
        ProfiledPIDController controller = new ProfiledPIDController(config.aimP, 0.0, config.aimD,
                new TrapezoidProfile.Constraints(config.maxAngularSpeed() * 0.5, config.maxAngularAcceleration() * 0.5));
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(Math.toRadians(kTurnToleranceDegrees));
        return Commands.run(() -> {
            double feedback = controller.calculate(drive.getRotation().getRadians(), goal.getRadians());
            drive.runVelocity(new ChassisSpeeds(0.0, 0.0, feedback + controller.getSetpoint().velocity));
        }, drive)
                .beforeStarting(() -> controller.reset(drive.getRotation().getRadians(), drive.getChassisSpeeds().omegaRadiansPerSecond))
                .until(controller::atGoal)
                .withTimeout(kTurnTimeoutSeconds)
                .finallyDo(drive::stop);
    }

    private void report(Pose2d expected) {
        Pose2d actual = state.getDrive().getPose();
        double meters = expected.getTranslation().getDistance(actual.getTranslation());
        double degrees = Math.abs(expected.getRotation().minus(actual.getRotation()).getDegrees());
        boolean passed = meters <= kPassMeters && degrees <= kPassDegrees;
        lastResult = new Result(passed, meters, degrees, worstTracking);

        String key = "Diagnostics/" + id + "/";
        Logger.recordOutput(key + "Passed", passed);
        Logger.recordOutput(key + "ErrorCm", meters * 100.0);
        Logger.recordOutput(key + "ErrorDegrees", degrees);
        Logger.recordOutput(key + "TrackingCm", worstTracking * 100.0);
        Logger.recordOutput(key + "Expected", expected);
        Logger.recordOutput(key + "Actual", actual);

        String cm = String.format(Locale.ROOT, "%.1f", meters * 100.0);
        String deg = String.format(Locale.ROOT, "%.1f", degrees);
        String tracking = String.format(Locale.ROOT, "%.1f", worstTracking * 100.0);
        results.put(id, String.join("\t", name(), passed ? "pass" : "fail", cm, deg, tracking));
        resultLines = results.values().toArray(String[]::new);
        Cockpit.toast(passed ? Level.INFO : Level.WARNING, name() + (passed ? " passed" : " missed"),
                "Ended " + cm + " cm and " + deg + "° from where it should, tracked the path within " + tracking + " cm");
    }
}
