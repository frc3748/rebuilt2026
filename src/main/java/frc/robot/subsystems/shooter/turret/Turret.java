package frc.robot.subsystems.shooter.turret;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.util.state.StateMachine;

public class Turret extends StateMachine<Turret.State> implements TurretIO{
    private final TurretIO turretIO;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    public Turret(TurretIO turretIO) {
        super("Turret", State.UNDETERMINED, State.class);
        this.turretIO = turretIO;
        registerStateTransitions();
        registerStateCommands();
        enable();
    }


    @Override
    public void update() {
        turretIO.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        
    }

    public void setPos(double position) {
        turretIO.setTurretPosition(position);

    }

    public void stop() {
        turretIO.stopTurret();
    }

    private void registerStateTransitions() {
        addOmniTransitions(State.IDLE, State.TARGET_TRACKING);
    }

    private void registerStateCommands() {
        // registerStateCommand(State.IDLE, Commands.run(() -> stop()));
        // registerStateCommand(State.TARGET_TRACKING, Commands.run(() -> setPos()));
    }

     @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }
    
    public enum State {
        UNDETERMINED,

        IDLE,
        TARGET_TRACKING

        // flags

    }

    
    
}
