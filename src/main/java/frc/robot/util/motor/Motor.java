package frc.robot.util.motor;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;

public class Motor {
    private final String name;
    private final MotorIO io;
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();
    private double setpoint;

    public Motor(MotorConfig config) {
        this(config.name(), createIO(config));
    }

    public Motor(String name, MotorIO io) {
        this.name = name;
        this.io = io;
    }

    private static MotorIO createIO(MotorConfig config) {
        return switch (Constants.kMode) {
            case REAL -> new MotorIOSpark(config);
            case SIM -> new MotorIOSim(config);
            case REPLAY -> new MotorIO() {};
        };
    }

    public void update() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
        Logger.recordOutput(name + "/Setpoint", setpoint);
    }

    public void setVoltage(double volts) {
        setpoint = 0.0;
        io.setVoltage(volts);
    }

    public void setOutput(double percent) {
        setpoint = 0.0;
        io.setOutput(percent);
    }

    public void setVelocity(double velocity) {
        setVelocity(velocity, 0.0);
    }

    public void setVelocity(double velocity, double feedforwardVolts) {
        setpoint = velocity;
        io.setVelocity(velocity, feedforwardVolts);
    }

    public void setPosition(double position) {
        setPosition(position, 0.0);
    }

    public void setPosition(double position, double feedforwardVolts) {
        setPosition(position, feedforwardVolts, 0);
    }

    public void setPosition(double position, double feedforwardVolts, int slot) {
        setpoint = position;
        io.setPosition(position, feedforwardVolts, slot);
    }

    public void stop() {
        setpoint = 0.0;
        io.stop();
    }

    public void setEncoderPosition(double position) {
        io.setEncoderPosition(position);
    }

    public void setCurrentLimit(int amps) {
        io.setCurrentLimit(amps);
    }

    public double getPosition() {
        return inputs.position;
    }

    public double getVelocity() {
        return inputs.velocity;
    }

    public double getCurrentAmps() {
        return inputs.currentAmps;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public boolean atPosition(double tolerance) {
        return Math.abs(inputs.position - setpoint) < tolerance;
    }

    public boolean atVelocity(double tolerance) {
        return Math.abs(inputs.velocity - setpoint) < tolerance;
    }
}
