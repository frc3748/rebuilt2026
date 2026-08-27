package frc.robot.subsystems.shooter.dumper;

public class DumperConstants {
    public static final int kDumperCanId = 15;

    public static final double kDumperP = 0.5;
    public static final double kDumperI = 0;
    public static final double kDumperD = 0;
    public static final double kDumperS = 0;
    public static final double kDumperV = 0;
    public static final double kDumperA = 0;
    public static final double kDumperG = 0;
    public static final double kDumperMaxAccel = 800.0;
    public static final double kDumperCruiseVel = 600.0;
    public static final double kDumperDeviationErr = 1;

    public static final double kDumperPositionConversionFactor = 2.0 * Math.PI / 4.0 / (200.0 / 20.0);
    public static final double kDumperVelocityConversionFactor = kDumperPositionConversionFactor / 60;

    public static final boolean kDumperInverted = false;
    public static final int kDumperCurrentLimit = 40;

    public static final double latencyCompensationMS = 20.0;

    public static final double kForwardSoftLimit = Math.PI / 2;
    public static final double kBackwardSoftLimit = 0.0;
}
