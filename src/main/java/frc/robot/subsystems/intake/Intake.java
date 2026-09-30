package frc.robot.subsystems.intake;

import frc.robot.util.state.StateMachine;

public abstract class Intake extends StateMachine<Intake.State> {
    public enum State {
        UNDETERMINED,
        STOW,
        IDLE,
        INTAKE,
        OUTAKE,
        SHAKE
    }

    protected Intake() {
        super("Intake", State.UNDETERMINED, State.class);
    }

    public abstract void rollIn();

    public abstract void rollOut();
}
