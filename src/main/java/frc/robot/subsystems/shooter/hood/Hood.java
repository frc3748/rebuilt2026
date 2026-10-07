package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.game.ShooterSetpoint;
import frc.robot.util.TunableNumber;
import frc.robot.util.Visuals;
import frc.robot.util.motor.PosMotor;
import frc.robot.util.state.StateMachine;

public class Hood extends StateMachine<Hood.State> {
    public enum State {
        UNDETERMINED,
        IDLE,
        PASS_TRACKING,
        HUB_TRACKING,
        TUNING
    }

    private final RobotState robotState;
    private final HoodConstants constants;
    private final PosMotor hood;
    private final TunableNumber customSetpoint;
    private final TunableNumber readyTolerance;

    public Hood(RobotState robotState, HoodConstants constants) {
        super("Hood", State.UNDETERMINED, State.class);
        this.robotState = robotState;
        this.constants = constants;
        hood = new PosMotor(constants.motor
                .conversion(constants.radiansPerRotation, constants.radiansPerRotation / 60.0)
                .softLimits(constants.minLimit, constants.maxLimit)
                .startingPosition(constants.minLimit));
        customSetpoint = TunableNumber.field("Hood/Custom Setpoint", constants, "customSetpoint").degrees();
        readyTolerance = TunableNumber.field("Hood/Ready Tolerance", constants, "readyTolerance").degrees();
        addHardware(hood);
        allowAllTransitions();

        enable();
    }

    @Override
    protected void applyState(State state) {
        switch (state) {
            case HUB_TRACKING -> aim(robotState.getCurrentHubSetpoint());
            case PASS_TRACKING -> aim(robotState.getCurrentPassSetpoint());
            case TUNING -> setPos(customSetpoint.get(), 0);
            case IDLE -> setPos(constants.minLimit, 0);
            case UNDETERMINED -> hood.stop();
        }
    }

    @Override
    protected void update() {
        Visuals.record("Hood/Pose", new Pose3d()
                .plus(robotState.getShooterConstants().shooterToRobotCenter)
                .plus(constants.shooterToHood)
                .plus(new Transform3d(
                        new Translation3d(),
                        new Rotation3d(0, Units.degreesToRadians(-120) + hood.getPosition(), 0))));
    }

    public void zero() {
        hood.resetPosition(constants.minLimit);
    }

    public double getAngle() {
        return hood.getPosition();
    }

    public double getGoal() {
        return hood.getGoal();
    }

    public boolean isAtGoal() {
        return hood.isAtGoal(readyTolerance.get());
    }

    public void aim(ShooterSetpoint setpoint) {
        setPos(setpoint.getHoodRadians(), setpoint.getHoodFF());
    }

    public void setPos(double position, double feedforward) {
        hood.set(MathUtil.clamp(position, constants.minLimit, constants.maxLimit), feedforward);
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }
}
