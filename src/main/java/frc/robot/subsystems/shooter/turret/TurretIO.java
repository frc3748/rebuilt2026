package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    
  }

  public static class TurretIOOutputs {
   
  }

  default void updateInputs(TurretIOInputs inputs) {}

  default void applyOutputs(TurretIOOutputs outputs) {}

  public default void setTurretVoltage(double volts) {}

  public default void setTurretPosition(double position) {}

  public default void stopTurret() {}
}
