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
        intakeIO.setExtensionPosition(IntakeConstants.kExtensionStowSetpoint);
        intakeIO.stopRollers();
    }

    public void intakeidle() {
        intakeIO.setExtensionPosition(IntakeConstants.kExtensionIntakeSetpoint);
        intakeIO.stopRollers();
    }
    
    public void intake() {
        intakeIO.setExtensionPosition(IntakeConstants.kExtensionIntakeSetpoint);
        intakeIO.setRollerVoltage(IntakeConstants.kRollerIntakeVoltage);
    }

    public void outake(){
        intakeIO.setExtensionPosition(IntakeConstants.kExtensionOuttakeSetpoint);
        intakeIO.setRollerVoltage(IntakeConstants.kRollerOuttakeVoltage);
    }

    public void stop() {
        intakeIO.stopExtension();
        intakeIO.stopRollers();
    }

    private void registerStateTransitions() {
        addOmniTransitions(State.STOW, State.IDLE, State.INTAKE, State.OUTAKE);
    }

    private void registerStateCommands() {
        registerStateCommand(State.STOP, Commands.run(() -> stop()));
        registerStateCommand(State.STOW, Commands.run(() -> stow()));
        registerStateCommand(State.IDLE, Commands.run(() -> intakeidle()));
        registerStateCommand(State.INTAKE, Commands.run(() -> intake()));
        registerStateCommand(State.OUTAKE, Commands.run(() -> outake()));
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
        OUTAKE

        // flags

    }
}

