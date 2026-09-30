package frc.robot.subsystems.hopper;

import static frc.robot.subsystems.hopper.HopperConstants.*;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.motor.SpinMotor;
import frc.robot.util.state.StateMachine;

public class Hopper extends StateMachine<Hopper.State> {
    public enum State {
        UNDETERMINED,
        IDLE,
        OUTAKE,
        SHOOT
    }

    private final BooleanSupplier shooterReady;
    private final SpinMotor hopper = new SpinMotor(kHopper);
    private double spinRadians;

    public Hopper(BooleanSupplier shooterReady) {
        super("Hopper", State.UNDETERMINED, State.class);
        this.shooterReady = shooterReady;
        addHardware(hopper);
        allowAllTransitions();
        enable();
    }

    @Override
    protected void applyState(State state) {
        switch (state) {
            case SHOOT -> hopper.set(shooterReady.getAsBoolean() ? kShootSpeed.get() : 0);
            case OUTAKE -> hopper.set(kOuttakeSpeed.get());
            case IDLE, UNDETERMINED -> hopper.set(0);
        }
    }

    @Override
    protected void update() {
        spinRadians += 0.02 * hopper.getVelocity() / kRollerRadiusMeters;
        Logger.recordOutput("Hopper/Pose", new Pose3d()
                .plus(kOrigin)
                .plus(new Transform3d(new Translation3d(), new Rotation3d(0, 0, spinRadians))));
    }

    public void feed() {
        hopper.set(kShootSpeed.get());
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }
}
