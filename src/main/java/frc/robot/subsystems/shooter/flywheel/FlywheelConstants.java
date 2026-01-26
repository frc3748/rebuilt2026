package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.util.Units;

public class FlywheelConstants {

    public static final double kPassMaxApexHeight = Units.inchesToMeters(160);

    public static final int kFlywheelCanID = 0;


    // Flywheel PID
    public static final double kFlywheelP = 0;
    public static final double kFlywheelI = 0;
    public static final double kFlywheelD = 0;
    public static final double kFlywheelMaxAccel = 0;
    public static final double kFlywheelCruiseVel = 0;
    public static final double kFlywheelDeviationErr = 0;


    // factors
    public static final double kFlywheelPositionConversionFactor = 0;
    public static final double kFlywheelVelocityConversionFactor = 0;


    // Configuration
    public static final boolean kFlywheelinverted = false;
    public static final int kFlywheelCurrentLimit = 10;


    // setpoints
    public static final double kFlywheelShootSpeed = 0;
    
    
}
