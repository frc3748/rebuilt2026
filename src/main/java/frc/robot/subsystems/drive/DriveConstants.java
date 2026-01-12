package frc.robot.subsystems.drive;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

// import com.pathplanner.lib.config.ModuleConfig;
// import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;


public class DriveConstants {

    public static final double maxSpeedMetersPerSec  = 4.8;
    public static final double odometryFrequency = 100.0; // Hz
    public static final double trackWidth = Units.inchesToMeters(28); //TODO CHANGE
    public static final double wheelBase = Units.inchesToMeters(28); // TODO CHANGE
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

    public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };


    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.158936); // TODO update 
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(-0.464111);
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.466553);
    public static final Rotation2d backRightZeroRotation = new Rotation2d(-0.429443);

    public static final int pigeonCanId = 50;

    public static final int frontLeftDriveCanId = 8;
    public static final int backLeftDriveCanId = 6;
    public static final int frontRightDriveCanId = 2;
    public static final int backRightDriveCanId = 4;

    public static final int frontLeftTurnCanId = 9;
    public static final int backLeftTurnCanId = 7;
    public static final int frontRightTurnCanId = 3;
    public static final int backRightTurnCanId = 5;

    public static final int frontLeftCanCoderId = 4;
    public static final int backLeftCanCoderId = 3;
    public static final int frontRightCanCoderId = 1;
    public static final int backRightCanCoderId = 2;

    public static final int driveMotorCurrentLimit = 55;
    public static final double wheelRadiusMeters = 0.0508;
    public static final double driveMotorReduction = 6.48;// Swerve X2i x3 with 10 pinion teeth
      //(45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14 pinion teeth
    public static DCMotor driveGearbox = DCMotor.getNEO(1);


    public static final CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
      // Drive encoder configuration
    public static final double driveEncoderPositionFactor = (.0508) * 2 * Math.PI / 6.48; // Wheel Radians
    public static final double driveEncoderVelocityFactor = ((.0508) * 2 * Math.PI / 6.48) / 60; // Wheel Rad/Sec

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
    public static final boolean turnInverted = true;
    public static final int turnMotorCurrentLimit = 55;
    public static final double turnMotorReduction = 12.1; //9424.0 / 203.0;
    public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = true;
    public static final double turnEncoderPositionFactor = (1 / 12.1 * 2 * Math.PI) ; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (1 / 12.1 * 2 * Math.PI) / 60.0 ; // RPM -> Rad/Sec

    // Turn PID configuration
    public static final double turnKp = 0.007;
    public static final double turnKi = 0.0;
    public static final double turnKd = 0.0;
    public static final double turnKf = 0.0;

    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    // PathPlanner configuration
    public static final double robotMassKg = 74.088;
    public static final double robotMOI = 6.883;
    public static final double wheelCOF = 1.2;

    public static final RobotConfig ppConfig =
          new RobotConfig(
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
}
