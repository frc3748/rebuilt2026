package frc.robot.subsystems.shooter.dumper;

public class DumperConstants {
    public static final int kDumperCanId = 15;

    //PID is for the tuning the motor part i thinkk
    public static final double kDumperP = 0.5;
    public static final double kDumperI = 0;
    public static final double kDumperD = 0;

    //SVAG, is for static friction, velocity, acceleration and gravity, something like that
    public static final double kDumperS = 0, kDumperV = 0, kDumperA = 0, kDumperG = 0;
    public static final double kDumperMaxAccel = 800.0;
    public static final double kDumperCruiseVel = 600.0;
    public static final double kDumperDeviationErr = 1;

    public static final double kDumperSimP = 0.1;
    public static final double kDumperSimD = 0;

    // radians per motor rotation, (turret value honestly)
    public static final double kDumperPositionConversionFactor = 2.0 * Math.PI / 4.0 / (200.0 / 20.0);

    //radians per second
    public static final double kDumperVelocityConversionFactor = kDumperPositionConversionFactor / 60;

    public static final boolean kDumperInverted = false;
    public static final int kDumperCurrentLimit = 40;
    public static final double latencyCompensationMS = 20.0;

    public static final double kForwardSoftLimit = Math.PI/2;
    public static final double kBackwardSoftLimit = 0.0;  


}
