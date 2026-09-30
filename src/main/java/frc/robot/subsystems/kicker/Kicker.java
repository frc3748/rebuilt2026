package frc.robot.subsystems.kicker;

import java.util.function.BooleanSupplier;

import frc.robot.util.TunableNumber;
import frc.robot.util.motor.SpinMotor;
import frc.robot.util.state.StateMachine;

public class Kicker extends StateMachine<Kicker.State> {
    public enum State {
        UNDETERMINED,
        IDLE,
        SHOOT,
        OUTAKE
    }

    private final BooleanSupplier shooterReady;
    private final SpinMotor kicker;
    private final TunableNumber shootSpeed;
    private final TunableNumber outtakeSpeed;

    public Kicker(KickerConstants constants, BooleanSupplier shooterReady) {
        super("Kicker", State.UNDETERMINED, State.class);
        this.shooterReady = shooterReady;
        kicker = new SpinMotor(constants.motor);
        shootSpeed = new TunableNumber("Kicker/Shot Speed", constants.shootSpeed);
        outtakeSpeed = new TunableNumber("Kicker/Outtake Speed", constants.outtakeSpeed);
        addHardware(kicker);
        allowAllTransitions();
        enable();
    }

    @Override
    protected void applyState(State state) {
        switch (state) {
            case SHOOT -> kicker.set(shooterReady.getAsBoolean() ? shootSpeed.get() : 0);
            case OUTAKE -> kicker.set(outtakeSpeed.get());
            case IDLE, UNDETERMINED -> kicker.set(0);
        }
    }

    public void feed() {
        kicker.set(shootSpeed.get());
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }
}
