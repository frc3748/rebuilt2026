package frc.robot.util.motor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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

    private static final double kHotCelsius = 80.0;
    private static final double kOverheatedCelsius = 95.0;

    private final String name;
    private final MotorIO io;
    private final MotorConfig config;
    private final double selfTestPosition;
    private final Alert disconnectedAlert;
    private final Alert hotAlert;
    private final Alert overheatedAlert;
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    private Mode mode = Mode.STOP;
    private double goal;
    private double feedforward;
    private int slot;

    public Motor(MotorConfig config) {
        this("Motors/" + config.name(), createIO(config), config);
    }

    public Motor(String name, MotorIO io) {
        this(name, io, null);
    }

    private Motor(String name, MotorIO io, MotorConfig config) {
        this.name = name;
        this.io = io;
        this.config = config;
        double selfTestPosition = config == null ? Double.NaN : config.selfTestPosition();
        this.selfTestPosition = selfTestPosition;
        String label = name.substring(name.lastIndexOf('/') + 1);
        disconnectedAlert = new Alert("Devices", label + " motor disconnected", AlertType.kError);
        hotAlert = new Alert(label + " motor is hot", AlertType.kWarning);
        overheatedAlert = new Alert(label + " motor is overheating", AlertType.kError);
    }

    private static MotorIO createIO(MotorConfig config) {
        return switch (Constants.kMode) {
            case REAL -> config.controller == MotorConfig.Controller.TALON_FX
                    ? new MotorIOTalonFX(config)
                    : new MotorIOSpark(config);
            case SIM -> new MotorIOSim(config);
            case REPLAY -> new MotorIO() {};
        };
    }

    @Override
    public void read() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
        disconnectedAlert.set(!inputs.connected && Constants.kMode == Constants.Mode.REAL);
        hotAlert.set(inputs.tempCelsius >= kHotCelsius && inputs.tempCelsius < kOverheatedCelsius);
        overheatedAlert.set(inputs.tempCelsius >= kOverheatedCelsius);
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

    public double getAppliedVolts() {
        return inputs.appliedVolts;
    }

    MotorConfig config() {
        return config;
    }

    public double getCurrentAmps() {
        return inputs.currentAmps;
    }

    public double getGoal() {
        return goal;
    }

    public String getName() {
        return name;
    }

    public boolean isConnected() {
        return inputs.connected;
    }

    public double getSelfTestPosition() {
        return selfTestPosition;
    }

    public double getTravel() {
        double[] range = config == null ? null : config.tuningRange();
        return range == null ? Double.NaN : range[1] - range[0];
    }

    public void holdPosition(double position) {
        request(Mode.POSITION, position, 0.0, 0);
    }
}
