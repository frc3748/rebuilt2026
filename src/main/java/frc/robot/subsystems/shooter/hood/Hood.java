package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.ShooterSetpoint;
import frc.robot.util.TrenchZone;
import frc.robot.util.motor.Motor;
import frc.robot.util.state.StateMachine;

public class Hood extends StateMachine<Hood.State> {
    private final RobotState state;
    private final Motor hood = new Motor(HoodConstants.kHood);
    private Runnable override;
    private boolean autoOverride;

    public Hood(RobotState state) {
        super("Hood", State.UNDETERMINED, State.class);
        this.state = state;

        addOmniTransitions(State.IDLE, State.HUB_TRACKING, State.PASS_TRACKING, State.UNDETERMINED, State.TUNING);

        SmartDashboard.putData("Hood Zero", Commands.runOnce(() -> hood.setEncoderPosition(HoodConstants.kMinLimit))
                .ignoringDisable(true)
                .withName("Hood Zero"));
        enable();
    }

    @Override
    protected void update() {
        hood.update();

        if (TrenchZone.hoodLowerRequired(state)
                && hood.getPosition() > HoodConstants.kMaxSetpointUnderTrench
                && !autoOverride) {
            setPos(HoodConstants.kMaxSetpointUnderTrench, 0);
        }

        if (override != null) {
            override.run();
        } else {
            switch (getState()) {
                case HUB_TRACKING -> aim(state.getCurrentHubSetpoint());
                case PASS_TRACKING -> aim(state.getCurrentPassSetpoint());
                case TUNING -> setPos(HoodConstants.kCustomSetpoint.get(), 0);
                default -> stop();
            }
        }

        Logger.recordOutput("Hood/Overriden", override != null);
        Logger.recordOutput("Hood/Pose", new Pose3d()
                .plus(VisionConstants.kShooterToRobotCenter)
                .plus(HoodConstants.kShooterToHood)
                .plus(new Transform3d(
                        new Translation3d(),
                        new Rotation3d(0, Units.degreesToRadians(-120) + hood.getPosition(), 0))));
    }

    public void aim(ShooterSetpoint setpoint) {
        setPos(setpoint.getHoodRadians(), setpoint.getHoodFF());
    }

    public void setPos(double position, double feedforward) {
        if (TrenchZone.hoodLowerRequired(state) && position > HoodConstants.kMaxSetpointUnderTrench) {
            position = HoodConstants.kMaxSetpointUnderTrench;
        }
        hood.setPosition(position, feedforward);
    }

    public void stop() {
        hood.stop();
    }

    public void setOverride(Runnable override) {
        this.override = override;
    }

    public void setAutoOverride(boolean autoOverride) {
        this.autoOverride = autoOverride;
    }

    @Override
    protected void determineSelf() {
        setState(State.UNDETERMINED);
    }

    public enum State {
        UNDETERMINED,
        IDLE,
        PASS_TRACKING,
        HUB_TRACKING,
        TUNING
    }
}
