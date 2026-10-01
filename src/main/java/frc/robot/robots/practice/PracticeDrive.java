package frc.robot.robots.practice;

import com.pathplanner.lib.config.PIDConstants;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConfig;
import frc.robot.util.motor.MotorConfig.Controller;

public class PracticeDrive extends DriveConfig {
    private static final double kWheelRadius = 0.0508;
    private static final double kHalfTrack = 0.3998;

    public PracticeDrive() {
        gyro = GyroType.NAVX;
        navXPort = NavXComType.kUSB1;
        navXUpdateRateHz = 50;

        driveController = Controller.SPARK_MAX;
        turnSensor = TurnSensor.SPARK_ABSOLUTE_ENCODER;
        frontLeft = new ModuleConstants(4, 3, -1, Rotation2d.fromRotations(0.0), true);
        frontRight = new ModuleConstants(5, 7, -1, Rotation2d.fromRotations(0.0), false);
        backLeft = new ModuleConstants(6, 2, -1, Rotation2d.fromRotations(0.0), true);
        backRight = new ModuleConstants(8, 41, -1, Rotation2d.fromRotations(0.0), false);

        trackWidth = 2 * kHalfTrack;
        wheelBase = 2 * kHalfTrack;

        wheelRadiusMeters = kWheelRadius;
        driveReduction = 7.31;
        driveGearbox = DCMotor.getNEO(1);
        driveCurrentLimit = 35;
        driveKp = 0.006 * kWheelRadius;
        driveKv = 0.29 * 12.0;

        turnReduction = 12.8;
        turnGearbox = DCMotor.getNEO(1);
        turnCurrentLimit = 10;
        turnInverted = true;
        turnEncoderInverted = false;
        turnKp = 0.005 * Units.radiansToDegrees(1.0);

        maxSpeedMetersPerSec = 3.5;
        aimInSlowMode = false;

        robotMassKg = 74.088;
        robotMOI = 6.883;
        wheelCOF = 1.2;
        pathTranslationPid = new PIDConstants(4.5, 0, 0.01);
        pathRotationPid = new PIDConstants(3.9, 0, 0.03);
    }
}
