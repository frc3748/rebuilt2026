package frc.robot.util.motor;

import edu.wpi.first.math.MathUtil;

public class MotorIOSim implements MotorIO {
    private enum Mode {
        OPEN_LOOP,
        VELOCITY,
        POSITION
    }

    private static final double kLoopSeconds = 0.02;

    private final MotorConfig config;
    private Mode mode = Mode.OPEN_LOOP;
    private double setpoint;
    private double position;
    private double velocity;
    private double appliedVolts;

    public MotorIOSim(MotorConfig config) {
        this.config = config;
        if (!Double.isNaN(config.startingPosition)) {
            position = config.startingPosition;
        }
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.connected = true;
        switch (mode) {
            case VELOCITY -> {
                velocity += (setpoint - velocity) * Math.min(1.0, kLoopSeconds / config.simVelocityLagSeconds);
                appliedVolts = MathUtil.clamp(
                        config.gains.kS * Math.signum(setpoint) + config.gains.kV * setpoint, -12.0, 12.0);
            }
            case POSITION -> {
                double maxStep = config.maxMotion && config.gains.cruiseVel > 0
                        ? config.gains.cruiseVel * kLoopSeconds
                        : Double.MAX_VALUE;
                double step = MathUtil.clamp(setpoint - position, -maxStep, maxStep);
                velocity = step / kLoopSeconds;
                appliedVolts = 0.0;
            }
            case OPEN_LOOP -> velocity += (openLoopVelocity() - velocity)
                    * Math.min(1.0, kLoopSeconds / config.simVelocityLagSeconds);
        }

        position += velocity * kLoopSeconds;
        if (!Double.isNaN(config.reverseSoftLimit)) {
            position = Math.max(position, config.reverseSoftLimit);
        }
        if (!Double.isNaN(config.forwardSoftLimit)) {
            position = Math.min(position, config.forwardSoftLimit);
        }

        inputs.position = position;
        inputs.velocity = velocity;
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = 0.0;
    }

    private double openLoopVelocity() {
        if (config.gains.kV <= 0.0) {
            return appliedVolts;
        }
        double driving = Math.max(0.0, Math.abs(appliedVolts) - config.gains.kS);
        return Math.signum(appliedVolts) * driving / config.gains.kV;
    }

    @Override
    public void setVoltage(double volts) {
        mode = Mode.OPEN_LOOP;
        appliedVolts = volts;
    }

    @Override
    public void setOutput(double percent) {
        mode = Mode.OPEN_LOOP;
        appliedVolts = percent * 12.0;
    }

    @Override
    public void setVelocity(double velocity, double feedforwardVolts) {
        mode = Mode.VELOCITY;
        setpoint = velocity;
    }

    @Override
    public void setPosition(double position, double feedforwardVolts, int slot) {
        mode = Mode.POSITION;
        setpoint = position;
    }

    @Override
    public void stop() {
        mode = Mode.OPEN_LOOP;
        appliedVolts = 0.0;
    }

    @Override
    public void setEncoderPosition(double position) {
        this.position = position;
    }
}
