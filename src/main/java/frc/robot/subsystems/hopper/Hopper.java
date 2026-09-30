package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotState;
import frc.robot.util.motor.Motor;
import frc.robot.util.state.StateMachine;

public class Hopper extends StateMachine<Hopper.State> {
    private final RobotState state;
    private final Motor hopper = new Motor(HopperConstants.kHopper);
    private Runnable override;
    private double spinRadians;

    public Hopper(RobotState state) {
        super("Hopper", State.UNDETERMINED, State.class);
        this.state = state;
        addOmniTransitions(State.UNDETERMINED, State.IDLE, State.OUTAKE, State.SHOOT);
        enable();
    }

    @Override
    protected void update() {
        hopper.update();

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

        spinRadians += 0.02 * hopper.getVelocity() / HopperConstants.kRollerRadiusMeters;
        Logger.recordOutput("Hopper/Overriden", override != null);
        Logger.recordOutput("Hopper/Pose", new Pose3d()
                .plus(HopperConstants.kOrigin)
                .plus(new Transform3d(new Translation3d(), new Rotation3d(0, 0, spinRadians))));
    }

    public void shoot() {
        hopper.setVelocity(HopperConstants.kShootSpeed.get());
    }

    public void outtake() {
        hopper.setVelocity(HopperConstants.kOuttakeSpeed.get());
    }

    public void stop() {
        hopper.setVelocity(0);
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
        OUTAKE,
        SHOOT
    }
}
