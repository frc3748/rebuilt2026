package frc.robot.subsystems.climb;

public class ClimbConstants {

    public static int kClimbCanID = 0;

    // Climb PID
    public static double kClimbP = 0;
    public static double kClimbI = 0;
    public static double kClimbD = 0;
    public static double kClimbMaxAccel = 0;
    public static double kClimbCruiseVel = 0;
    public static double kClimbDeviationErr = 0;


    // factors
    public static double kClimbPositionConversionFactor = 0;
    public static double kClimbVelocityConversionFactor = 0;


    // Configuration
    public static boolean kClimbinverted = false;
    public static int kClimbCurrentLimit = 10;


    // setpoints
    public static double kClimbStowPos = 0;
    public static double kClimbUpPos = 0;
    public static double kClimbDownPos = 0;
    
}
