package frc.robot.util.motor;

import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {
    @AutoLog
    class MotorIOInputs {
        public double position = 0.0;
        public double velocity = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelsius = 0.0;
        public double[] followerAppliedVolts = new double[0];
        public double[] followerCurrentAmps = new double[0];
    }

    default void updateInputs(MotorIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void setOutput(double percent) {}

    default void setVelocity(double velocity, double feedforwardVolts) {}

    default void setPosition(double position, double feedforwardVolts, int slot) {}

    default void stop() {}

    default void setEncoderPosition(double position) {}

    default void setCurrentLimit(int amps) {}
}
