package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.game.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConfig;
import frc.robot.util.TunableNumber;

public class FollowPath extends Command {
    private static final double kLoopSeconds = 0.02;
    private static final double kIdealSpeedTolerance = 0.25;
    private static final double kIdealRotationDegrees = 30.0;
    private static final double kMovingEndSpeed = 0.1;
    private static final double kEndMeters = 0.05;
    private static final double kEndDegrees = 3.0;
    private static final TunableNumber followLead = new TunableNumber("Path/Follow Lead", 0.3);
    private static final TunableNumber maxLead = new TunableNumber("Path/Max Lead", 0.75);
    private static final TunableNumber settleSeconds = new TunableNumber("Path/Settle Seconds", 0.75);
    private static final TunableNumber wallMargin = new TunableNumber("Path/Wall Margin", 0.05);

    private final Drive drive;
    private final PathPlannerPath original;
    private final PPHolonomicDriveController controller;
    private final RobotConfig robotConfig;
    private final BooleanSupplier shouldFlip;
    private final EventScheduler events = new EventScheduler();
    private PathPlannerPath path;
    private PathPlannerTrajectory trajectory;
    private PathPlannerTrajectoryState target;
    private double time;
    private double overtime;

    public FollowPath(Drive drive, PathPlannerPath path, PPHolonomicDriveController controller, RobotConfig robotConfig,
            BooleanSupplier shouldFlip) {
        this.drive = drive;
        this.original = path;
        this.controller = controller;
        this.robotConfig = robotConfig;
        this.shouldFlip = shouldFlip;
        addRequirements(drive);
        addRequirements(EventScheduler.getSchedulerRequirements(path).toArray(Subsystem[]::new));
        setName(path.name == null ? "FollowPath" : path.name);
    }

    @Override
    public void initialize() {
        path = shouldFlip.getAsBoolean() && !original.preventFlipping ? original.flipPath() : original;
        Pose2d pose = drive.getPose();
        ChassisSpeeds speeds = drive.getChassisSpeeds();
        controller.reset(pose, speeds);
        trajectory = idealTrajectory(pose, speeds)
                .orElseGet(() -> path.generateTrajectory(speeds, pose.getRotation(), robotConfig));
        time = 0.0;
        overtime = 0.0;
        target = trajectory.sample(0.0);
        PathPlannerAuto.setCurrentTrajectory(trajectory);
        PathPlannerAuto.currentPathName = original.name;
        PathPlannerLogging.logActivePath(path);
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
        double lead = pose.getTranslation().getDistance(trajectory.sample(time).pose.getTranslation());
        double rate = MathUtil.clamp((maxLead.get() - lead) / (maxLead.get() - followLead.get()), 0.0, 1.0);
        double total = trajectory.getTotalTimeSeconds();
        if (time >= total) {
            overtime += kLoopSeconds;
        }
        time = Math.min(total, time + kLoopSeconds * rate);
        target = shape(trajectory.sample(time), rate);

        drive.runVelocity(controller.calculateRobotRelativeSpeeds(pose, target), target.feedforwards);

        PathPlannerLogging.logCurrentPose(pose);
        PathPlannerLogging.logTargetPose(target.pose);
        Logger.recordOutput("Auto/Path/Rate", rate);
        Logger.recordOutput("Auto/Path/LeadMeters", lead);
        Logger.recordOutput("Auto/Path/Time", time);
        events.execute(time);
    }

    private PathPlannerTrajectoryState shape(PathPlannerTrajectoryState state, double rate) {
        DriveConfig config = drive.getConfig();
        Rotation2d heading = state.pose.getRotation();
        double cos = Math.abs(heading.getCos());
        double sin = Math.abs(heading.getSin());
        double halfX = config.bumperLength() / 2.0 * cos + config.bumperWidth() / 2.0 * sin + wallMargin.get();
        double halfY = config.bumperLength() / 2.0 * sin + config.bumperWidth() / 2.0 * cos + wallMargin.get();
        double x = MathUtil.clamp(state.pose.getX(), halfX, FieldConstants.LAYOUT_LENGTH_METERS - halfX);
        double y = MathUtil.clamp(state.pose.getY(), halfY, FieldConstants.LAYOUT_WIDTH_METERS - halfY);
        double vx = state.fieldSpeeds.vxMetersPerSecond * rate;
        double vy = state.fieldSpeeds.vyMetersPerSecond * rate;
        if (x != state.pose.getX()) {
            vx = x < state.pose.getX() ? Math.min(vx, 0.0) : Math.max(vx, 0.0);
        }
        if (y != state.pose.getY()) {
            vy = y < state.pose.getY() ? Math.min(vy, 0.0) : Math.max(vy, 0.0);
        }
        PathPlannerTrajectoryState shaped = state.copyWithTime(state.timeSeconds);
        shaped.pose = new Pose2d(x, y, heading);
        shaped.fieldSpeeds = new ChassisSpeeds(vx, vy, state.fieldSpeeds.omegaRadiansPerSecond * rate);
        shaped.linearVelocity = Math.hypot(vx, vy);
        shaped.feedforwards = rate < 1.0 || x != state.pose.getX() || y != state.pose.getY()
                ? DriveFeedforwards.zeros(robotConfig.numModules)
                : state.feedforwards;
        return shaped;
    }

    public Pose2d targetPose() {
        return target == null ? drive.getPose() : target.pose;
    }

    @Override
    public boolean isFinished() {
        if (time < trajectory.getTotalTimeSeconds()) {
            return false;
        }
        if (path.getGoalEndState().velocityMPS() >= kMovingEndSpeed) {
            return true;
        }
        Pose2d pose = drive.getPose();
        boolean there = pose.getTranslation().getDistance(target.pose.getTranslation()) <= kEndMeters
                && Math.abs(pose.getRotation().minus(target.pose.getRotation()).getDegrees()) <= kEndDegrees;
        return there || overtime >= settleSeconds.get();
    }

    @Override
    public void end(boolean interrupted) {
        PathPlannerAuto.currentPathName = "";
        PathPlannerAuto.setCurrentTrajectory(null);
        if (!interrupted && path.getGoalEndState().velocityMPS() < kMovingEndSpeed) {
            drive.stop();
        }
        Logger.recordOutput("Auto/Path/Rate", 1.0);
        PathPlannerLogging.logActivePath(null);
        events.end();
    }
}
