package frc.robot.commands;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConfig;
import frc.robot.subsystems.drive.HeadingLock;

public class DriveCommands {
    private static final double kDeadband = 0.1;
    private static final double kFeedforwardStartDelaySeconds = 2.0;
    private static final double kFeedforwardRampVoltsPerSecond = 0.1;
    private static final double kWheelRadiusMaxVelocity = 0.25;
    private static final double kWheelRadiusRampRate = 0.05;

    private DriveCommands() {}

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        double magnitude = MathUtil.applyDeadband(Math.hypot(x, y), kDeadband);
        Rotation2d direction = new Rotation2d(Math.atan2(y, x));
        return new Pose2d(Translation2d.kZero, direction)
                .transformBy(new Transform2d(magnitude * magnitude, 0.0, Rotation2d.kZero))
                .getTranslation();
    }

    private static Rotation2d fieldHeading(Drive drive) {
        boolean flipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        return flipped ? drive.getRotation().plus(Rotation2d.kPi) : drive.getRotation();
    }

    public static Command smartDrive(
            Drive drive,
            DoubleSupplier x,
            DoubleSupplier y,
            DoubleSupplier manualOmega,
            Supplier<Rotation2d> autoRotationGoal,
            Supplier<Drive.State> stateSupplier) {
        DriveConfig config = drive.getConfig();
        ProfiledPIDController angleController = new ProfiledPIDController(
                config.aimP, 0, config.aimD,
                new TrapezoidProfile.Constraints(config.maxAngularSpeed(), config.maxAngularAcceleration()));
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        DogLog.tunable("Auto Turn", config.aimP, angleController::setP);
        HeadingLock headingLock = new HeadingLock(
                config, drive::getGyroRotation, () -> drive.getChassisSpeeds().omegaRadiansPerSecond);

        return Commands.run(() -> {
            double omega;
            double raw = MathUtil.applyDeadband(manualOmega.getAsDouble(), kDeadband);
            if (stateSupplier.get() == Drive.State.TRAVERSING_AT_ANGLE) {
                headingLock.release();
                omega = angleController.calculate(drive.getRotation().getRadians(), autoRotationGoal.get().getRadians());
            } else if (raw != 0.0) {
                headingLock.release();
                omega = Math.copySign(raw * raw, raw) * drive.getMaxAngularSpeedRadPerSec();
            } else {
                omega = headingLock.hold();
            }

            Translation2d linear = getLinearVelocityFromJoysticks(x.getAsDouble(), y.getAsDouble());
            ChassisSpeeds speeds = new ChassisSpeeds(
                    linear.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                    linear.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                    omega);
            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, fieldHeading(drive)));
        }, drive).beforeStarting(() -> {
            angleController.reset(drive.getRotation().getRadians());
            headingLock.release();
        });
    }

    public static Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                Commands.runOnce(() -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),
                Commands.run(() -> drive.runCharacterization(0.0), drive).withTimeout(kFeedforwardStartDelaySeconds),
                Commands.runOnce(timer::restart),
                Commands.run(() -> {
                    double voltage = timer.get() * kFeedforwardRampVoltsPerSecond;
                    drive.runCharacterization(voltage);
                    velocitySamples.add(drive.getFFCharacterizationVelocity());
                    voltageSamples.add(voltage);
                }, drive).finallyDo(() -> {
                    int n = velocitySamples.size();
                    double sumX = 0.0;
                    double sumY = 0.0;
                    double sumXY = 0.0;
                    double sumX2 = 0.0;
                    for (int i = 0; i < n; i++) {
                        sumX += velocitySamples.get(i);
                        sumY += voltageSamples.get(i);
                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                    }
                    double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                    double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                    NumberFormat formatter = new DecimalFormat("#0.00000");
                    System.out.println("********** Drive FF Characterization Results **********");
                    System.out.println("\tkS: " + formatter.format(kS));
                    System.out.println("\tkV: " + formatter.format(kV));
                    SmartDashboard.putString("Character/kS", formatter.format(kS));
                    SmartDashboard.putString("Character/kV", formatter.format(kV));
                }));
    }

    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(kWheelRadiusRampRate);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                Commands.sequence(
                        Commands.runOnce(() -> limiter.reset(0.0)),
                        Commands.run(() -> {
                            double speed = limiter.calculate(kWheelRadiusMaxVelocity);
                            drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                        }, drive)),
                Commands.sequence(
                        Commands.waitSeconds(1.0),
                        Commands.runOnce(() -> {
                            state.positions = drive.getWheelRadiusCharacterizationPositions();
                            state.lastAngle = drive.getRotation();
                            state.gyroDelta = 0.0;
                        }),
                        Commands.run(() -> {
                            Rotation2d rotation = drive.getRotation();
                            state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                            state.lastAngle = rotation;
                        }).finallyDo(() -> {
                            double[] positions = drive.getWheelRadiusCharacterizationPositions();
                            double wheelDelta = 0.0;
                            for (int i = 0; i < 4; i++) {
                                wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                            }
                            double wheelRadius = (state.gyroDelta * drive.getConfig().driveBaseRadius()) / wheelDelta;

                            NumberFormat formatter = new DecimalFormat("#0.000");
                            System.out.println("********** Wheel Radius Characterization Results **********");
                            System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                            System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                            System.out.println("\tWheel Radius: " + formatter.format(wheelRadius) + " meters, "
                                    + formatter.format(Units.metersToInches(wheelRadius)) + " inches");
                        })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = Rotation2d.kZero;
        double gyroDelta = 0.0;
    }
}
