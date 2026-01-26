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

  public default void setHopperVoltage(double volts) {}

  public default void setHopperSpeed(double position) {}

  public default void stopHopper() {}

}
