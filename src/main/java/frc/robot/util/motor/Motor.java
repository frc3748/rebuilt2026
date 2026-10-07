package frc.robot.util.motor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
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
    private static final double kStallAmpsFraction = 0.8;
    private static final double kStallSpeedFraction = 0.02;
    private static final double kStallErrorFraction = 0.01;
    private static final double kStallSeconds = 1.0;

    private final String name;
    private final MotorIO io;
    private final MotorConfig config;
    private final double selfTestPosition;
    private final Alert disconnectedAlert;
    private final Alert hotAlert;
    private final Alert overheatedAlert;
    private final Alert stallAlert;
    private final Debouncer stallDebounce = new Debouncer(kStallSeconds, Debouncer.DebounceType.kRising);
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    private Mode mode = Mode.STOP;
    private double goal;
    private double feedforward;
    private int slot;
    private int currentLimit;
    private double stalledGoal = Double.NaN;

    public Motor(MotorConfig config) {
        this("Motors/" + config.name(), createIO(config), config);
    }

    public Motor(String name, MotorIO io) {
        this(name, io, null);
    }

    Motor(String name, MotorIO io, MotorConfig config) {
        this.name = name;
        this.io = io;
        this.config = config;
        double selfTestPosition = config == null ? Double.NaN : config.selfTestPosition();
        this.selfTestPosition = selfTestPosition;
        String label = name.substring(name.lastIndexOf('/') + 1);
        disconnectedAlert = new Alert("Devices", label + " motor disconnected", AlertType.kError);
        hotAlert = new Alert(label + " motor is hot", AlertType.kWarning);
        overheatedAlert = new Alert(label + " motor is overheating", AlertType.kError);
        stallAlert = new Alert(label + " was pushing into a hard stop, so it stopped. Check its zero", AlertType.kWarning);
        currentLimit = config == null ? 0 : config.currentLimit;
    }

    private static MotorIO createIO(MotorConfig config) {
        return switch (Constants.kMode) {
            case REAL -> config.controller == MotorConfig.Controller.TALON_FX ? new MotorIOTalonFX(config) : createSpark(config);
            case SIM -> new MotorIOSim(config);
            case REPLAY -> new MotorIO() {};
        };
    }

    private static MotorIO createSpark(MotorConfig config) {
        try {
            return new MotorIOSpark(config);
        } catch (IllegalStateException e) {
            new Alert("Devices", config.name() + " motor didn't start: " + e.getMessage(), AlertType.kError).set(true);
            return new MotorIO() {};
        }
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
        boolean stalled = stalledIntoStop();
        switch (mode) {
            case STOP -> io.stop();
            case VOLTAGE -> io.setVoltage(goal);
            case OUTPUT -> io.setOutput(goal);
            case VELOCITY -> io.setVelocity(goal, feedforward);
            case POSITION -> {
                if (stalled) {
                    io.stop();
                } else {
                    io.setPosition(goal, feedforward, slot);
                }
            }
        }
        stallAlert.set(stalled);
        Logger.recordOutput(name + "/Mode", mode.name());
        Logger.recordOutput(name + "/Goal", goal);
        Logger.recordOutput(name + "/Stalled", stalled);
    }

    private boolean stalledIntoStop() {
        double travel = getTravel();
        if (mode != Mode.POSITION || !Double.isFinite(travel) || currentLimit <= 0) {
            stalledGoal = Double.NaN;
            stallDebounce.calculate(false);
            return false;
        }
        if (!Double.isNaN(stalledGoal) && Math.abs(goal - stalledGoal) > kStallErrorFraction * travel) {
            stalledGoal = Double.NaN;
        }
        boolean pushing = Double.isNaN(stalledGoal)
                && Math.abs(goal - inputs.position) > kStallErrorFraction * travel
                && Math.abs(inputs.velocity) < kStallSpeedFraction * travel
                && Math.abs(inputs.currentAmps) >= kStallAmpsFraction * currentLimit;
        if (stallDebounce.calculate(pushing)) {
            stalledGoal = goal;
        }
        return !Double.isNaN(stalledGoal);
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
        currentLimit = MotorConfig.safeCurrentLimit(amps);
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
