package frc.robot.subsystems.kicker;

import static frc.robot.subsystems.kicker.KickerConstants.*;

import frc.robot.RobotState;
import frc.robot.util.motor.SpinMotor;
import frc.robot.util.state.StateMachine;

public class Kicker extends StateMachine<Kicker.State> {
    public enum State {
        UNDETERMINED,
        IDLE,
        SHOOT,
        OUTAKE
    }

    private final RobotState robotState;
    private final SpinMotor kicker = new SpinMotor(kKicker);

    public Kicker(RobotState robotState) {
        super("Kicker", State.UNDETERMINED, State.class);
        this.robotState = robotState;
        addHardware(kicker);
        allowAllTransitions();
        enable();
    }

    @Override
    protected void applyState(State state) {
        switch (state) {
            case SHOOT -> kicker.set(robotState.getShooter().isReady() ? kShootSpeed.get() : 0);
            case OUTAKE -> kicker.set(kOuttakeSpeed.get());
            case IDLE, UNDETERMINED -> kicker.set(0);
        }
    }

    public void feed() {
        kicker.set(kShootSpeed.get());
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }
}
