package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    
  }

  public static class FlywheelIOOutputs {
   
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void applyOutputs(FlywheelIOOutputs outputs) {}

  public default void setFlywheelVoltage(double volts) {}

  public default void setFlywheelSpeed(double position) {}

  public default void stopFlywheel() {}

}
