package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double velocity = 0.0;
    public double rotorVelocity = 0.0;
    public double targetVelocity = 0.0;
    public double voltage = 0.0;

    public boolean proxTripped = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}
}