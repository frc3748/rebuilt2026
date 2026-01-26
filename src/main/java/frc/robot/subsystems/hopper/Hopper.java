package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.state.StateMachine;

public class Hopper extends StateMachine<Hopper.State> implements HopperIO{

    private final HopperIO hopperIO;
    private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

    public Hopper(HopperIO hopperIO) {
        super("Hopper", State.UNDETERMINED, State.class);
        this.hopperIO = hopperIO;
        registerStateTransitions();
        registerStateCommands();
        enable();
    }

    @Override
    public void update() {
        hopperIO.updateInputs(inputs);
        Logger.processInputs("Hopper", inputs);
    }

    public void shoot() {
        hopperIO.setHopperVoltage(0);
    }

    public void outake() {
        hopperIO.setHopperVoltage(0);
    }

    public void stop() {
        hopperIO.stopHopper();
    }

    private void registerStateTransitions() {
        addOmniTransitions(State.IDLE, State.OUTAKE, State.SHOOT);
    }

    private void registerStateCommands() {
        registerStateCommand(State.IDLE, Commands.run(() -> stop()));
        registerStateCommand(State.OUTAKE, Commands.run(() -> outake()));
        registerStateCommand(State.SHOOT, Commands.run(() -> shoot()));
    }

     @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }
    
    public enum State {
        UNDETERMINED,

        IDLE,
        OUTAKE,
        SHOOT

        // flags

    }
    
}
