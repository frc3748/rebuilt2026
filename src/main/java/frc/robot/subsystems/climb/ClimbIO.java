package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    
  }

  public static class ClimbIOOutputs {
   
  }

  default void updateInputs(ClimbIOInputs inputs) {}

  default void applyOutputs(ClimbIOOutputs outputs) {}

  public default void setClimbVoltage(double volts) {}

  public default void setClimbPosition(double position) {}

  public default void stopClimb() {}

}
