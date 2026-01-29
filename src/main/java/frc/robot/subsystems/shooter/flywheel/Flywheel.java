package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.state.StateMachine;

public class Flywheel extends StateMachine<Flywheel.State> implements FlywheelIO{

    private final FlywheelIO flywheelIO;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    public Flywheel(FlywheelIO flywheelIO) {
        super("Flywheel", State.UNDETERMINED, State.class);
        this.flywheelIO = flywheelIO;
        registerStateTransitions();
        registerStateCommands();
        enable();
    }


    @Override
    public void update() {
        flywheelIO.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
        
    }

    public void shoot() {
        flywheelIO.setFlywheelSpeed(0);

    }

    public void stop() {
        flywheelIO.stopFlywheel();
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
