package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.state.StateMachine;

public class Climb extends StateMachine<Climb.State> implements ClimbIO{

    private final ClimbIO climbIO;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    public Climb(ClimbIO climbIO) {
        super("Climb", State.UNDETERMINED, State.class);
        this.climbIO = climbIO;
        registerStateTransitions();
        registerStateCommands();
        enable();
    }

    @Override
    public void update() {
        climbIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public void stow() {
        climbIO.setClimbPosition(0);
    }
    
    public void up() {
        climbIO.setClimbPosition(0);

    }

    public void down(){
        climbIO.setClimbPosition(0);

    }

    public void stop() {
        climbIO.stopClimb();
    }

    private void registerStateTransitions() {
        addOmniTransitions(State.STOW, State.IDLE, State.UP, State.DOWN);
    }

    private void registerStateCommands() {
        registerStateCommand(State.STOW, Commands.run(() -> stow(), this));
        registerStateCommand(State.IDLE, Commands.run(() -> stop(), this));
        registerStateCommand(State.UP, Commands.run(() -> up(), this));
        registerStateCommand(State.DOWN, Commands.run(() -> down(), this));
    }

     @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }
    
    public enum State {
        UNDETERMINED,

        STOW,
        IDLE,
        UP,
        DOWN,

        // flags

    }
    
}
