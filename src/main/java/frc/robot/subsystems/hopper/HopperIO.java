package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    
  }

  public static class HopperIOOutputs {
   
  }

  default void updateInputs(HopperIOInputs inputs) {}

  default void applyOutputs(HopperIOOutputs outputs) {}
}
