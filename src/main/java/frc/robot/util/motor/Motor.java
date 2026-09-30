package frc.robot.util.motor;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.util.state.Hardware;

public class Motor implements Hardware {
    protected enum Mode {
        STOP,
        VOLTAGE,
        OUTPUT,
        VELOCITY,
        POSITION
    }

    private final String name;
    private final MotorIO io;
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    private Mode mode = Mode.STOP;
    private double goal;
    private double feedforward;
    private int slot;

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

    @Override
    public void read() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    @Override
    public void write() {
        switch (mode) {
            case STOP -> io.stop();
            case VOLTAGE -> io.setVoltage(goal);
            case OUTPUT -> io.setOutput(goal);
            case VELOCITY -> io.setVelocity(goal, feedforward);
            case POSITION -> io.setPosition(goal, feedforward, slot);
        }
        Logger.recordOutput(name + "/Mode", mode.name());
        Logger.recordOutput(name + "/Goal", goal);
    }

    protected final void request(Mode mode, double goal, double feedforward, int slot) {
        this.mode = mode;
        this.goal = goal;
        this.feedforward = feedforward;
        this.slot = slot;
    }

    public void stop() {
        request(Mode.STOP, 0.0, 0.0, 0);
    }

    public void setVoltage(double volts) {
        request(Mode.VOLTAGE, volts, 0.0, 0);
    }

    public void setOutput(double percent) {
        request(Mode.OUTPUT, percent, 0.0, 0);
    }

    public void setCurrentLimit(int amps) {
        io.setCurrentLimit(amps);
    }

    protected final void setEncoderPosition(double position) {
        io.setEncoderPosition(position);
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

    public double getGoal() {
        return goal;
    }
}
