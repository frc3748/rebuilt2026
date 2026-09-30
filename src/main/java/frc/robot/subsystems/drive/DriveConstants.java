package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 5.265648;
  public static final double slowSpeedMetersPerSec = 0.5;

  public static final double odometryFrequency = 100.0;
  public static final double trackWidth = Units.inchesToMeters(28);
  public static final double wheelBase = Units.inchesToMeters(28);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

  public static final double kBumperHeight = Units.inchesToMeters(7);

  public static final Translation2d[] moduleTranslations = new Translation2d[] {
      new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
      new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
      new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
      new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
  };

  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(Units.rotationsToRadians(0.27490234375+0.5));
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(Units.rotationsToRadians(-0.0263671875+0.5));
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(Units.rotationsToRadians(0.084716796875));
  public static final Rotation2d backRightZeroRotation = new Rotation2d(Units.rotationsToRadians(-0.4609375));

  public static final int pigeonCanId = 50;

  public static final int frontLeftDriveCanId = 8;
  public static final int backLeftDriveCanId = 2;
  public static final int frontRightDriveCanId = 4;
  public static final int backRightDriveCanId = 6;

  public static final int frontLeftTurnCanId = 3;
  public static final int backLeftTurnCanId = 9;
  public static final int frontRightTurnCanId = 7;
  public static final int backRightTurnCanId = 5;

  public static final int frontLeftCanCoderId = 4;
  public static final int backLeftCanCoderId = 1;
  public static final int frontRightCanCoderId = 2;
  public static final int backRightCanCoderId = 3;

  public static final int driveMotorCurrentLimit = 45;
  public static final double wheelRadiusMeters = 0.0508;
  public static final double driveMotorReduction = 6.48;
  public static DCMotor driveGearbox = DCMotor.getNEO(1);

  public static final double driveEncoderPositionFactor =  2 * Math.PI / driveMotorReduction;
  public static final double driveEncoderVelocityFactor =  (2 * Math.PI / driveMotorReduction) / 60;

  public static final double driveKp = 0.01;
  public static final double driveKi = 0.0;
  public static final double driveKd = 0.0;

  public static final double driveSimP = 0.1;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.9;
  public static final double driveSimKv = 3;

  public static final double driveKs = 0.1;
  public static final double driveKv = 1.8;

  public static final double driveIntegrationCap = .001;
  public static double turnIntegrationCap = .5;

  public static final boolean turnInverted = true;
  public static final int turnMotorCurrentLimit = 45;
  public static final double turnMotorReduction = 12.1;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = (2 * Math.PI / turnMotorReduction);
  public static final double turnEncoderVelocityFactor = (2 * Math.PI / turnMotorReduction) / 60.0;

  public static final double turnKp = 0.7;
  public static final double turnKi = 0.0;
  public static final double turnKd = 0.0;
  public static final double turnKv = 0.0;

  public static final double turnSimP = 3.0;
  public static final double turnSimD = 0.0;

  public static final double turnPIDMinInput = 0;
  public static final double turnPIDMaxInput = 2 * Math.PI;

  public static final double robotMassKg = 50;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;

  public static final RobotConfig ppConfig = new RobotConfig(
      robotMassKg,
      robotMOI,
      new ModuleConfig(
          wheelRadiusMeters,
          maxSpeedMetersPerSec,
          wheelCOF,
          driveGearbox.withReduction(driveMotorReduction),
          driveMotorCurrentLimit,
          1),
      moduleTranslations);

  public static final PathConstraints pathConstraint = new PathConstraints(
        4.8,
        5.0,
        Units.degreesToRadians(360),
        Units.degreesToRadians(360)
    );

  public static double kDriveToPointP = 4.0;
  public static double kDriveToPointI = 0.0;
  public static double kDriveToPointD = 0;

  public static double kMaxLinearAcceleration = 5.0;

  public static double kDriveToPointHeadingP = 3;
  public static double kDriveToPointHeadingI = 0.0;
  public static double kDriveToPointHeadingD = 0.00;

  public static final double kDriveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

  public static final double kMaxAngularSpeed = maxSpeedMetersPerSec / kDriveBaseRadius;
  public static final double kMaxAngularAcceleration = kMaxLinearAcceleration / kDriveBaseRadius;

  public static final double metersTolerance = Units.inchesToMeters(1);
  public static final double radiansTolerance = Units.degreesToRadians(1);

  public static final double metersAccelTolerance = 0.075;
  public static final double radAccelTolerance = Math.PI / 16;

  public static double kABDriveP = 3.8;
  public static double kABDriveI = 0.0;
  public static double kABDriveD = 0.0;

  public static double kABTurnP = 7.0;
  public static double kABTurnI = 0.0;
  public static double kABTurnD = 0.0;
}
