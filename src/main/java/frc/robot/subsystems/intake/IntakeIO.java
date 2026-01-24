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
}
