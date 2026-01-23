package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    
  }

  public static class IntakeIOOutputs {
   
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void applyOutputs(IntakeIOOutputs outputs) {}

  public default void setRollerVoltage(double volts) {}

  public default void setRollerSpeed(double speed) {}

  public default void setExtentionPosition(double position) {}

  public default void stopRollers() {}

  public default void stopExtention() {}


}
