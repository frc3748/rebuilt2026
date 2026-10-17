package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConfig;
import frc.robot.util.TunableNumber;

public class FollowPath extends Command {
    private static final double kLoopSeconds = 0.02;
    private static final double kIdealSpeedTolerance = 0.25;
    private static final double kIdealRotationDegrees = 30.0;
    private static final double kMovingEndSpeed = 0.1;
    private static final double kEndMeters = 0.02;
    private static final double kEndDegrees = 2.0;
    private static final double kStalledMeters = 0.05;
    private static final double kStalledSpeed = 0.05;
    private static final double kPushingSpeed = 0.2;
    private static final int kReferencePoses = 80;
    private static final TunableNumber maxLead = new TunableNumber("Path/Max Lead", 0.5);
    private static final TunableNumber feedforwardLead = new TunableNumber("Path/Feedforward Lead", 0.25);
    private static final TunableNumber maxCorrection = new TunableNumber("Path/Max Correction", 1.5);
    private static final TunableNumber settleSeconds = new TunableNumber("Path/Settle Seconds", 0.75);

    private final Drive drive;
    private final PathPlannerPath original;
    private final RobotConfig robotConfig;
    private final BooleanSupplier shouldFlip;
    private final PIDController alongTrack;
    private final PIDController crossTrack;
    private final PIDController rotation;
    private final EventScheduler events = new EventScheduler();
    private PathPlannerPath path;
    private PathPlannerTrajectory trajectory;
    private PathReference reference;
    private PathReference.Sample target;
    private PathReference.Projection projection;
    private double time;
    private double overtime;
    private double commandedSpeed;

    public FollowPath(Drive drive, PathPlannerPath path, PIDConstants translation, PIDConstants turning, RobotConfig robotConfig,
            BooleanSupplier shouldFlip) {
        this.drive = drive;
        this.original = path;
        this.robotConfig = robotConfig;
        this.shouldFlip = shouldFlip;
        alongTrack = new PIDController(translation.kP, translation.kI, translation.kD);
        crossTrack = new PIDController(translation.kP, translation.kI, translation.kD);
        rotation = new PIDController(turning.kP, turning.kI, turning.kD);
        rotation.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive);
        addRequirements(EventScheduler.getSchedulerRequirements(path).toArray(Subsystem[]::new));
        setName(path.name == null ? "FollowPath" : path.name);
    }

    @Override
    public void initialize() {
        path = shouldFlip.getAsBoolean() && !original.preventFlipping ? original.flipPath() : original;
        Pose2d pose = drive.getPose();
        ChassisSpeeds speeds = drive.getChassisSpeeds();
        trajectory = idealTrajectory(pose, speeds)
                .orElseGet(() -> path.generateTrajectory(speeds, pose.getRotation(), robotConfig));
        DriveConfig config = drive.getConfig();
        reference = PathReference.build(trajectory, config.autoTurnLimit());
        alongTrack.reset();
        crossTrack.reset();
        rotation.reset();
        time = 0.0;
        overtime = 0.0;
        commandedSpeed = 0.0;
        projection = reference.project(pose.getTranslation(), 0);
        target = reference.atTime(0.0);
        PathPlannerAuto.setCurrentTrajectory(trajectory);
        PathPlannerAuto.currentPathName = original.name;
        PathPlannerLogging.logActivePath(path);
        Logger.recordOutput("Auto/Path/Reference", reference.poses(kReferencePoses));
        events.initialize(trajectory);
    }

    private Optional<PathPlannerTrajectory> idealTrajectory(Pose2d pose, ChassisSpeeds speeds) {
        var ideal = path.getIdealStartingState();
        if (ideal == null) {
            return Optional.empty();
        }
        double speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        boolean idealSpeed = Math.abs(speed - ideal.velocityMPS()) <= kIdealSpeedTolerance;
        boolean idealRotation = Math.abs(pose.getRotation().minus(ideal.rotation()).getDegrees()) <= kIdealRotationDegrees;
        return idealSpeed && idealRotation ? path.getIdealTrajectory(robotConfig) : Optional.empty();
    }

    @Override
    public void execute() {
        Pose2d pose = drive.getPose();
        projection = reference.project(pose.getTranslation(), projection.index());
        if (time >= reference.totalTime()) {
            overtime += kLoopSeconds;
        }
        time = Math.min(reference.totalTime(), time + kLoopSeconds);
        target = reference.atTime(time);
        boolean held = target.distance() - projection.distance() > maxLead.get();
        if (held) {
            time = reference.timeAtDistance(projection.distance() + maxLead.get());
            target = reference.atTime(time);
        }
        double along = target.distance() - projection.distance();
        double limit = maxCorrection.get();
        double speed = (held ? 0.0 : target.speed()) + MathUtil.clamp(alongTrack.calculate(-along, 0.0), -limit, limit);
        double correction = MathUtil.clamp(crossTrack.calculate(projection.crossTrack(), 0.0), -limit, limit);
        Translation2d tangent = projection.tangent();
        double vx = tangent.getX() * speed - tangent.getY() * correction;
        double vy = tangent.getY() * speed + tangent.getX() * correction;
        commandedSpeed = Math.hypot(vx, vy);
        double omega = (held ? 0.0 : target.omega())
                + rotation.calculate(pose.getRotation().getRadians(), target.pose().getRotation().getRadians());
        omega = MathUtil.clamp(omega, -drive.getConfig().maxAngularSpeed(), drive.getConfig().maxAngularSpeed());
        DriveFeedforwards feedforwards = !held && Math.abs(along) < feedforwardLead.get() && Math.abs(projection.crossTrack()) < feedforwardLead.get()
                ? trajectory.sample(time).feedforwards
                : DriveFeedforwards.zeros(robotConfig.numModules);

        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, pose.getRotation()), feedforwards);

        PathPlannerLogging.logCurrentPose(pose);
        PathPlannerLogging.logTargetPose(target.pose());
        Logger.recordOutput("Auto/Path/CrossTrackMeters", projection.crossTrack());
        Logger.recordOutput("Auto/Path/AlongTrackMeters", along);
        Logger.recordOutput("Auto/Path/Held", held);
        Logger.recordOutput("Auto/Path/Time", time);
        events.execute(time);
    }

    public Pose2d targetPose() {
        return target == null ? drive.getPose() : target.pose();
    }

    public double crossTrackMeters() {
        return projection == null ? 0.0 : Math.abs(projection.crossTrack());
    }

    @Override
    public boolean isFinished() {
        if (time < reference.totalTime()) {
            return false;
        }
        if (path.getGoalEndState().velocityMPS() >= kMovingEndSpeed) {
            return true;
        }
        Pose2d pose = drive.getPose();
        Pose2d end = reference.end();
        boolean turned = Math.abs(pose.getRotation().minus(end.getRotation()).getDegrees()) <= kEndDegrees;
        boolean there = pose.getTranslation().getDistance(end.getTranslation()) <= kEndMeters;
        ChassisSpeeds speeds = drive.getChassisSpeeds();
        boolean stalled = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < kStalledSpeed
                && commandedSpeed > kPushingSpeed
                && reference.totalDistance() - projection.distance() <= kStalledMeters;
        return (turned && (there || stalled)) || overtime >= settleSeconds.get();
    }

    @Override
    public void end(boolean interrupted) {
        PathPlannerAuto.currentPathName = "";
        PathPlannerAuto.setCurrentTrajectory(null);
        if (!interrupted && path.getGoalEndState().velocityMPS() < kMovingEndSpeed) {
            drive.stop();
        }
        Logger.recordOutput("Auto/Path/Held", false);
        PathPlannerLogging.logActivePath(null);
        events.end();
    }
}
