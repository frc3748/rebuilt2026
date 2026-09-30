package frc.robot.subsystems.hopper;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.TunableNumber;
import frc.robot.util.motor.SpinMotor;
import frc.robot.util.state.StateMachine;

public class Hopper extends StateMachine<Hopper.State> {
    public enum State {
        UNDETERMINED,
        IDLE,
        OUTAKE,
        SHOOT
    }

    private final HopperConstants constants;
    private final BooleanSupplier shooterReady;
    private final SpinMotor hopper;
    private final TunableNumber shootSpeed;
    private final TunableNumber outtakeSpeed;
    private double spinRadians;

    public Hopper(HopperConstants constants, BooleanSupplier shooterReady) {
        super("Hopper", State.UNDETERMINED, State.class);
        this.constants = constants;
        this.shooterReady = shooterReady;
        hopper = new SpinMotor(constants.motor);
        shootSpeed = new TunableNumber("Hopper/Shoot Speed", constants.shootSpeed);
        outtakeSpeed = new TunableNumber("Hopper/Outtake Speed", constants.outtakeSpeed);
        addHardware(hopper);
        allowAllTransitions();
        enable();
    }

    @Override
    protected void applyState(State state) {
        switch (state) {
            case SHOOT -> hopper.set(shooterReady.getAsBoolean() ? shootSpeed.get() : 0);
            case OUTAKE -> hopper.set(outtakeSpeed.get());
            case IDLE, UNDETERMINED -> hopper.set(0);
        }
    }

    @Override
    protected void update() {
        spinRadians += 0.02 * hopper.getVelocity() / constants.rollerRadiusMeters;
        Logger.recordOutput("Hopper/Pose", new Pose3d()
                .plus(constants.origin)
                .plus(new Transform3d(new Translation3d(), new Rotation3d(0, 0, spinRadians))));
    }

    public void feed() {
        hopper.set(shootSpeed.get());
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }
}
