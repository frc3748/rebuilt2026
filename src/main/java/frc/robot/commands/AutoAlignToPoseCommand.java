package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConfig;
import frc.robot.util.TunableNumber;

public class AutoAlignToPoseCommand extends Command {
    public enum AlignType {
        DEFAULT,
        ROTATION,
        TRANSLATION
    }

    private static final double kFeedforwardMinRadius = 0.0;
    private static final double kFeedforwardMaxRadius = 0.1;
    private static Tuning tuning;

    private final Drive drive;
    private final RobotState state;
    private final Pose2d target;
    private final AlignType alignType;
    private final ProfiledPIDController driveController;
    private final ProfiledPIDController thetaController;

    public AutoAlignToPoseCommand(Drive drive, RobotState state, Pose2d target, double constraintFactor) {
        this(drive, state, target, constraintFactor, AlignType.DEFAULT);
    }

    public AutoAlignToPoseCommand(Drive drive, RobotState state, Pose2d target, double constraintFactor,
            AlignType alignType) {
        this.drive = drive;
        this.state = state;
        this.target = target;
        this.alignType = alignType;

        DriveConfig config = drive.getConfig();
        if (tuning == null) {
            tuning = new Tuning(config);
        }
        driveController = new ProfiledPIDController(
                tuning.driveP.get(),
                0.0,
                0.0,
                new TrapezoidProfile.Constraints(
                        config.maxSpeedMetersPerSec * constraintFactor,
                        config.maxLinearAcceleration * constraintFactor),
                0.02);
        thetaController = new ProfiledPIDController(
                tuning.turnP.get(),
                0.0,
                0.0,
                new TrapezoidProfile.Constraints(config.maxAngularSpeed(), config.maxAngularAcceleration()),
                0.02);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        Pose2d current = state.getLatestFieldToRobot().getValue();
        ChassisSpeeds fieldSpeeds = state.getLatestMeasuredFieldRelativeChassisSpeeds();
        Rotation2d toTarget = target.getTranslation().minus(current.getTranslation()).getAngle();
        double closingVelocity = -new Translation2d(-fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond)
                .rotateBy(toTarget.unaryMinus())
                .getX();

        driveController.setP(tuning.driveP.get());
        thetaController.setP(tuning.turnP.get());
        driveController.setTolerance(tuning.metersTolerance.get());
        thetaController.setTolerance(tuning.radiansTolerance.get());
        driveController.reset(current.getTranslation().getDistance(target.getTranslation()), Math.min(0.0, closingVelocity));
        thetaController.reset(current.getRotation().getRadians(),
                state.getLatestRobotRelativeChassisSpeed().omegaRadiansPerSecond);

        drive.setFieldPoses(current, target);
        Logger.recordOutput("DriveToPose/Target", target);
        Logger.recordOutput("DriveToPose/Active", true);
    }

    @Override
    public void execute() {
        Pose2d current = state.getLatestFieldToRobot().getValue();

        double distance = current.getTranslation().getDistance(target.getTranslation());
        double ffScaler = MathUtil.clamp(
                (distance - kFeedforwardMinRadius) / (kFeedforwardMaxRadius - kFeedforwardMinRadius), 0.0, 1.0);
        if (alignType == AlignType.ROTATION) {
            distance = 0;
            ffScaler = 1;
        }

        double driveVelocityScalar = driveController.getSetpoint().velocity * ffScaler
                + driveController.calculate(distance, 0.0);
        if (distance < driveController.getPositionTolerance()) {
            driveVelocityScalar = 0.0;
        }

        double thetaVelocity = thetaController.getSetpoint().velocity * ffScaler
                + thetaController.calculate(current.getRotation().getRadians(), target.getRotation().getRadians());
        double thetaError = Math.abs(current.getRotation().minus(target.getRotation()).getRadians());
        if (thetaError < thetaController.getPositionTolerance()) {
            thetaVelocity = 0.0;
        }

        Rotation2d awayFromTarget = current.getTranslation().minus(target.getTranslation()).getAngle();
        Translation2d driveVelocity = new Translation2d(driveVelocityScalar, awayFromTarget);

        if (alignType == AlignType.ROTATION) {
            driveVelocity = new Translation2d();
        }
        if (alignType == AlignType.TRANSLATION) {
            thetaVelocity = 0;
        }

        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, current.getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("DriveToPose/Active", false);
        drive.runVelocity(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return switch (alignType) {
            case ROTATION -> thetaController.atGoal();
            case TRANSLATION -> driveController.atGoal();
            case DEFAULT -> driveController.atGoal() && thetaController.atGoal();
        };
    }

    private static class Tuning {
        final TunableNumber driveP;
        final TunableNumber turnP;
        final TunableNumber metersTolerance;
        final TunableNumber radiansTolerance;

        Tuning(DriveConfig config) {
            driveP = new TunableNumber("Auto Align/Drive kP", config.driveToPointP);
            turnP = new TunableNumber("Auto Align/Turn kP", config.driveToPointHeadingP);
            metersTolerance = new TunableNumber("Auto Align/Meters Tolerance", config.metersTolerance);
            radiansTolerance = new TunableNumber("Auto Align/Radians Tolerance", config.radiansTolerance);
        }
    }
}
