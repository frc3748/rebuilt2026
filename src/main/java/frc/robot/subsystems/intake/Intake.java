package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.state.StateMachine;

public class Intake extends StateMachine<Intake.State> implements IntakeIO{

    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO intakeIO) {
        super("Intake", State.UNDETERMINED, State.class);
        this.intakeIO = intakeIO;
        registerStateTransitions();
        registerStateCommands();
        enable();
    }

    @Override
    public void update() {
        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public void stow() {
        intakeIO.setExtentionPosition(0);
        intakeIO.setRollerSpeed(0);
    }

    public void intakeidle() {
        intakeIO.setExtentionPosition(0);
        intakeIO.stopRollers();
    }
    
    public void intake() {
        intakeIO.setExtentionPosition(0);
        intakeIO.setRollerSpeed(0);
    }

    public void outake(){
        intakeIO.setExtentionPosition(0);
        intakeIO.setRollerSpeed(0);
    }

    public void stop() {
        intakeIO.stopExtention();
        intakeIO.stopRollers();
    }

    private void registerStateTransitions() {
        addOmniTransitions(State.STOW, State.IDLE, State.INTAKE, State.OUTAKE);
    }

    private void registerStateCommands() {
        registerStateCommand(State.STOP, Commands.run(() -> stop(), this));
        registerStateCommand(State.STOW, Commands.run(() -> stow(), this));
        registerStateCommand(State.IDLE, Commands.run(() -> intakeidle(), this));
        registerStateCommand(State.INTAKE, Commands.run(() -> intake(), this));
        registerStateCommand(State.OUTAKE, Commands.run(() -> outake(), this));
    }

     @Override
    protected void determineSelf() {
        setState(State.STOP);
    }
    
    public enum State {
        UNDETERMINED,

        STOP,
        STOW,
        IDLE,
        INTAKE,
        OUTAKE,

        // flags

    }
}

