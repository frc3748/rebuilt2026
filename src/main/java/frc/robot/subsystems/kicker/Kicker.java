package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.Logger;

import frc.robot.RobotState;
import frc.robot.util.motor.Motor;
import frc.robot.util.state.StateMachine;

public class Kicker extends StateMachine<Kicker.State> {
    private final RobotState state;
    private final Motor kicker = new Motor(KickerConstants.kKicker);
    private Runnable override;

    public Kicker(RobotState state) {
        super("Kicker", State.UNDETERMINED, State.class);
        this.state = state;
        addOmniTransitions(State.UNDETERMINED, State.IDLE, State.SHOOT, State.OUTAKE);
        enable();
    }

    @Override
    protected void update() {
        kicker.update();

        if (override != null) {
            override.run();
        } else {
            switch (getState()) {
                case SHOOT -> {
                    if (state.getShooter().isReady()) {
                        shoot();
                    } else {
                        stop();
                    }
                }
                case OUTAKE -> outtake();
                default -> stop();
            }
        }

        Logger.recordOutput("Kicker/Overriden", override != null);
    }

    public void shoot() {
        kicker.setVelocity(KickerConstants.kShootSpeed.get());
    }

    public void outtake() {
        kicker.setVelocity(KickerConstants.kOuttakeSpeed.get());
    }

    public void stop() {
        kicker.setVelocity(0);
    }

    public void setOverride(Runnable override) {
        this.override = override;
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }

    public enum State {
        UNDETERMINED,
        IDLE,
        SHOOT,
        OUTAKE
    }
}
