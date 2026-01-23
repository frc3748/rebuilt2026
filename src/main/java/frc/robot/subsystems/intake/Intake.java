package frc.robot.subsystems.intake;

import frc.robot.util.state.StateMachine;

public class Intake extends StateMachine<Intake.State> implements IntakeIO{
    
    public Intake() {
        super("Intake", State.UNDETERMINED, State.class);
    }

    @Override
    public void determineSelf() {
        setState(State.IDLE);
    }
    
    public enum State {
        UNDETERMINED,
        IDLE
    }
}
