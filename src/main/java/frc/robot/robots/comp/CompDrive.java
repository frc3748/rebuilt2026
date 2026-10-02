package frc.robot.robots.comp;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConfig;
import frc.robot.util.motor.MotorConfig.Controller;

public class CompDrive extends DriveConfig {
    public CompDrive() {
        gyro = GyroType.PIGEON2;
        pigeonCanId = 50;

        driveController = Controller.SPARK_FLEX;
        turnSensor = TurnSensor.CANCODER;
        frontLeft = new ModuleConstants(8, 3, 4, Rotation2d.kZero, false);
        frontRight = new ModuleConstants(4, 7, 2, Rotation2d.kZero, false);
        backLeft = new ModuleConstants(2, 9, 1, Rotation2d.kZero, false);
        backRight = new ModuleConstants(6, 5, 3, Rotation2d.kZero, false);

        trackWidth = Units.inchesToMeters(28);
        wheelBase = Units.inchesToMeters(28);
        bumperHeight = Units.inchesToMeters(7);

        wheelRadiusMeters = 0.0508;
        driveReduction = 6.48;
        driveGearbox = DCMotor.getNeoVortex(1);
        driveCurrentLimit = 45;
        driveKp = 0.01;
        driveKs = 0.1;
        driveKv = 2.28;
        driveSparkKv = 0.0;

        turnReduction = 12.1;
        turnGearbox = DCMotor.getNEO(1);
        turnCurrentLimit = 45;
        turnInverted = true;
        turnEncoderInverted = true;
        turnKp = 0.7;

        maxSpeedMetersPerSec = 5.265648;
        slowSpeedMetersPerSec = 0.5;

        robotMassKg = 50;
        robotMOI = 6.883;
        wheelCOF = 1.2;
        pathTranslationPid = new PIDConstants(3.8, 0.0, 0.0);
        pathRotationPid = new PIDConstants(7.0, 0.0, 0.0);
    }
}
