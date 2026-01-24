package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    
  }

  public static class HoodIOOutputs {
   
  }

  default void updateInputs(HoodIOInputs inputs) {}

  default void applyOutputs(HoodIOOutputs outputs) {}

  public default void setHoodVoltage(double volts) {}

  public default void setHoodPosition(double position) {}

  public default void stopHood() {}
}
