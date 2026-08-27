package frc.robot.subsystems.shooter.dumper;

import org.littletonrobotics.junction.AutoLog;

public interface DumperIO {
  @AutoLog
  public static class DumperIOInputs {
    public double dumperPos = 0.0;
    public double dumperVelRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double desiredPos = 0.0;
    public boolean isDumping = false;
  }

  default void updateInputs(DumperIOInputs inputs) {}

  public default void setDumperVoltage(double volts) {}

  public default void setDumperPosition(double position, double ff) {}

  public default void setDumperPosition(double position) {}

  public default double getDumperPosition() { return 0.0; }

  public default double getDumperVelocity() { return 0.0; }

  public default void stopDumper() {}
}
