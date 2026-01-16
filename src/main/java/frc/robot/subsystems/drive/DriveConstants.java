package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {

  public static final double maxSpeedMetersPerSec = 4.8*4; // check why this is broken
  public static final double slowSpeedMetersPerSec = 2;

  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(28); // TODO CHANGE
  public static final double wheelBase = Units.inchesToMeters(28); // TODO CHANGE
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

  public static final Translation2d[] moduleTranslations = new Translation2d[] {
      new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
      new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
      new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
      new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
  };

  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0); // can 1
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0); // can 2
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(0); // can 3
  public static final Rotation2d backRightZeroRotation = new Rotation2d(0); // can 4

  public static final int pigeonCanId = 50;

  public static final int frontLeftDriveCanId = 2;
  public static final int backLeftDriveCanId = 6;
  public static final int frontRightDriveCanId = 4;
  public static final int backRightDriveCanId = 8;

  public static final int frontLeftTurnCanId = 9;
  public static final int backLeftTurnCanId = 7;
  public static final int frontRightTurnCanId = 3;
  public static final int backRightTurnCanId = 5;

  public static final int frontLeftCanCoderId = 1;
  public static final int backLeftCanCoderId = 3;
  public static final int frontRightCanCoderId = 2;
  public static final int backRightCanCoderId = 4;

  public static final int driveMotorCurrentLimit = 55; 
  public static final double wheelRadiusMeters = 0.0508;
  public static final double driveMotorReduction = 6.48;// Swerve X2i x3 with 10 pinion teeth
  public static DCMotor driveGearbox = DCMotor.getNEO(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor = 2 * Math.PI / 6.48; // Wheel Radians
  public static final double driveEncoderVelocityFactor = (2 * Math.PI / 6.48) / 60; // Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.008;
  public static final double driveKi = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKf = 0.29;

  public static final double driveKs = 0.0;
  public static final double driveKv = 0.29;

  public static final double driveIntegrationCap = .001;
  public static double turnIntegrationCap = .5;

  // Turn motor configuration
  public static final boolean turnInverted = true; // try to check this out
  public static final int turnMotorCurrentLimit = 55;
  public static final double turnMotorReduction = 12.1; // 9424.0 / 203.0;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = (2 * Math.PI / turnMotorReduction); // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI / turnMotorReduction) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 0.7;
  public static final double turnKi = 0.0;
  public static final double turnKd = 0.0;
  public static final double turnKf = 0.0;

  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
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

  // auto align

  public static double kDriveToPointP = 4.0; // everything else is 3
  public static double kDriveToPointI = 0.0;
  public static double kDriveToPointD = 0.005;

  public static double kMaxLinearSpeed = 5;
  public static double kMaxLinearAcceleration = 3.5;

  public static double kDriveToPointHeadingP = 3;
  public static double kDriveToPointHeadingI = 0.0;
  public static double kDriveToPointHeadingD = 0.00;

  public static final double kTrackWidthX = Units.inchesToMeters(36);
  public static final double kTrackWidthY = Units.inchesToMeters(36);
  public static final double kDriveBaseRadius = Math.hypot(kTrackWidthX / 2.0, kTrackWidthY / 2.0);

  public static final double kMaxAngularSpeed = kMaxLinearSpeed / kDriveBaseRadius;
  public static final double kMaxAngularAcceleration = kMaxLinearAcceleration / kDriveBaseRadius;

  public static final double metersTolerance = Units.inchesToMeters(1);
  public static final double radiansTolerance = Units.degreesToRadians(1);

  public static final double metersAccelTolerance = 0.075;
  public static final double rotAccelTolerance = Math.PI / 16;
}
