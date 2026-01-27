package frc.robot.subsystems.climb;

public class ClimbConstants {
  public static final double climbKp = 0.1;
  public static final double climbKi = 0.0;
  public static final double climbKd = 0.0;
  public static final double climbKv = 0.0;

  public static final double climbPIDMinInput = 0; // Radians
  public static final double climbPIDMaxInput = 2 * Math.PI; // Radians



  //change later
  public static final boolean climbInverted = false;
  public static final double climbEncoderPositionFactor = (2 * Math.PI / climbMotorReduction); // Rotations -> Radians
  public static final double climbEncoderVelocityFactor = (2 * Math.PI / climbMotorReduction) / 60.0; // RPM -> Rad/Sec
}