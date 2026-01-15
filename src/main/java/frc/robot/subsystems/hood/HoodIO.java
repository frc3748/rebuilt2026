package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    
  }

  public static class HoodIOOutputs {
   
  }

  default void updateInputs(HoodIOInputs inputs) {}

  default void applyOutputs(HoodIOOutputs outputs) {}
}
