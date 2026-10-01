package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.util.motor.MotorConfig.Controller;

public class DriveConfig {
    public enum GyroType {
        PIGEON2,
        NAVX
    }

    public enum TurnSensor {
        CANCODER,
        SPARK_ABSOLUTE_ENCODER
    }

    public record ModuleConstants(int driveCanId, int turnCanId, int canCoderId, Rotation2d zeroRotation,
            boolean driveInverted) {}

    public GyroType gyro = GyroType.PIGEON2;
    public int pigeonCanId = 0;
    public NavXComType navXPort = NavXComType.kMXP_SPI;
    public int navXUpdateRateHz = 100;

    public Controller driveController = Controller.SPARK_FLEX;
    public TurnSensor turnSensor = TurnSensor.CANCODER;
    public ModuleConstants frontLeft;
    public ModuleConstants frontRight;
    public ModuleConstants backLeft;
    public ModuleConstants backRight;

    public double trackWidth;
    public double wheelBase;
    public double bumperHeight;
    public double odometryFrequency = 100.0;

    public double wheelRadiusMeters;
    public double driveReduction;
    public DCMotor driveGearbox = DCMotor.getNEO(1);
    public int driveCurrentLimit;
    public double driveKp;
    public double driveKi;
    public double driveKd;
    public double driveKs;
    public double driveKv;
    public double driveSparkKv;
    public double driveIntegrationCap = 0.001;

    public double turnReduction;
    public DCMotor turnGearbox = DCMotor.getNeo550(1);
    public int turnCurrentLimit;
    public boolean turnInverted = true;
    public boolean turnEncoderInverted;
    public double turnKp;
    public double turnKi;
    public double turnKd;
    public double turnKv;

    public double driveSimP = 0.1;
    public double driveSimD = 0.0;
    public double driveSimKs = 0.9;
    public double driveSimKv = 3.0;
    public double turnSimP = 3.0;
    public double turnSimD = 0.0;

    public double maxSpeedMetersPerSec;
    public double slowSpeedMetersPerSec = 0.5;
    public boolean aimInSlowMode = true;
    public double maxLinearAcceleration = 5.0;

    public double robotMassKg;
    public double robotMOI;
    public double wheelCOF;
    public PIDConstants pathTranslationPid;
    public PIDConstants pathRotationPid;
    public PathConstraints pathConstraints = new PathConstraints(
            4.8, 5.0, Units.degreesToRadians(360), Units.degreesToRadians(360));

    public double aimP = 8.0;
    public double aimD = 0.0;
    public double headingLockP = 5.0;
    public double headingLockD = 0.0;
    public double headingLockToleranceRadians = Units.degreesToRadians(1);
    public double headingLockCaptureRadPerSec = Units.degreesToRadians(10);

    public double driveToPointP = 4.0;
    public double driveToPointHeadingP = 3.0;
    public double metersTolerance = 0.04;
    public double radiansTolerance = Units.degreesToRadians(2.0);

    public ModuleConstants module(int index) {
        return switch (index) {
            case 0 -> frontLeft;
            case 1 -> frontRight;
            case 2 -> backLeft;
            default -> backRight;
        };
    }

    public Translation2d[] moduleTranslations() {
        return new Translation2d[] {
                new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };
    }

    public double driveBaseRadius() {
        return Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    }

    public double maxAngularSpeed() {
        return maxSpeedMetersPerSec / driveBaseRadius();
    }

    public double maxAngularAcceleration() {
        return maxLinearAcceleration / driveBaseRadius();
    }

    public double driveEncoderPositionFactor() {
        return 2 * Math.PI / driveReduction;
    }

    public double turnEncoderPositionFactor() {
        return 2 * Math.PI / turnReduction;
    }

    public RobotConfig pathPlannerConfig() {
        return new RobotConfig(
                robotMassKg,
                robotMOI,
                new ModuleConfig(
                        wheelRadiusMeters,
                        maxSpeedMetersPerSec,
                        wheelCOF,
                        driveGearbox.withReduction(driveReduction),
                        driveCurrentLimit,
                        1),
                moduleTranslations());
    }
}
