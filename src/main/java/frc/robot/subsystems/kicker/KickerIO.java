package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    
  }

  public static class KickerIOOutputs {
   
  }

  default void updateInputs(KickerIOInputs inputs) {}

  default void applyOutputs(KickerIOOutputs outputs) {}

  public default void setKickerVoltage(double volts) {}

  public default void setKickerSpeed(double position) {}

  public default void stopKicker() {}

}
