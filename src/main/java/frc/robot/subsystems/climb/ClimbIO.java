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
}
