package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    
  }


  default void updateInputs(IntakeIOInputs inputs) {}

  public default void setRollerVoltage(double volts) {}

  public default void setRollerSpeed(double speed) {}

  public default void setExtensionPosition(double position) {}

  public default void stopRollers() {}

  public default void stopExtension() {}


}
