package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.state.StateMachine;

public class Kicker extends StateMachine<Kicker.State> implements KickerIO{

    private final KickerIO kickerIO;
    private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

    public Kicker(KickerIO kickerIO) {
        super("Kicker", State.UNDETERMINED, State.class);
        this.kickerIO = kickerIO;
        registerStateTransitions();
        registerStateCommands();
        enable();
    }

    @Override
    public void update() {
        kickerIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public void shoot() {
        kickerIO.setKickerSpeed(0);

    }

    public void stop() {
        kickerIO.stopKicker();
    }

    private void registerStateTransitions() {
        addOmniTransitions(State.IDLE, State.SHOOT);
    }

    private void registerStateCommands() {
        registerStateCommand(State.IDLE, Commands.run(() -> stop()));
        registerStateCommand(State.SHOOT, Commands.run(() -> shoot()));
    }

     @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }
    
    public enum State {
        UNDETERMINED,

        IDLE,
        SHOOT

        // flags

    }
    
}
