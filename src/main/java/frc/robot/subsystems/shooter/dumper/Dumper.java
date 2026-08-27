package frc.robot.subsystems.shooter.dumper;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.util.state.StateMachine;

public class Dumper extends StateMachine<Dumper.State> implements DumperIO {
    private final RobotState state;
    private final DumperIO dumperIO;
    private final DumperIOInputsAutoLogged inputs = new DumperIOInputsAutoLogged();
    private final Timer timer;

    private double tunedSetpoint = 0.0;
    private Consumer<Object> override;

    public Dumper(DumperIO dumperIO, RobotState state) {
        super("Dumper", State.UNDETERMINED, State.class);
        this.dumperIO = dumperIO;
        this.state = state;

        this.timer = new Timer();
        timer.start();

        registerStateTransitions();
        registerStateCommands();
        enable();
    }

    @Override
    public void update() {
        dumperIO.updateInputs(inputs);

        if (override != null) {
            override.accept(null);
        } else if (getState() == State.TUNING) {
            setPos(tunedSetpoint);
        } else if (getState() == State.HUB_DUMPING || getState() == State.PASS_DUMPING) {
            setPos(DumperConstants.kForwardSoftLimit);
        } else {
            setPos(DumperConstants.kBackwardSoftLimit);
        }

        Logger.processInputs("Dumper", inputs);
        Logger.recordOutput("Dumper/Overriden", override != null);
    }

    public void setPos(double position, double ff) {
        dumperIO.setDumperPosition(position, ff);
    }

    public void setPos(double position) {
        dumperIO.setDumperPosition(position);
    }

    public void stop() {
        dumperIO.stopDumper();
    }

    private void registerStateTransitions() {
        addOmniTransitions(State.IDLE, State.HUB_DUMPING, State.PASS_DUMPING, State.UNDETERMINED, State.TUNING);
    }

    private void registerStateCommands() {
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }

    public void setOverride(Consumer<Object> override) {
        this.override = override;
    }

    public enum State {
        UNDETERMINED,

        IDLE,
        HUB_DUMPING,
        PASS_DUMPING,
        TUNING
    }
}
