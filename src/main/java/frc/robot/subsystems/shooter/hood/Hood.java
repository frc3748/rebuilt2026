package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

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
    private final PosMotor hood = new PosMotor(kHood);

    public Hood(RobotState robotState) {
        super("Hood", State.UNDETERMINED, State.class);
        this.robotState = robotState;
        addHardware(hood);
        allowAllTransitions();

        SmartDashboard.putData("Hood Zero", Commands.runOnce(() -> hood.resetPosition(kMinLimit))
                .ignoringDisable(true)
                .withName("Hood Zero"));
        enable();
    }

    @Override
    protected void applyState(State state) {
        switch (state) {
            case HUB_TRACKING -> aim(robotState.getCurrentHubSetpoint());
            case PASS_TRACKING -> aim(robotState.getCurrentPassSetpoint());
            case TUNING -> setPos(kCustomSetpoint.get(), 0);
            case IDLE, UNDETERMINED -> hood.stop();
        }
    }

    @Override
    protected void update() {
        Logger.recordOutput("Hood/Pose", new Pose3d()
                .plus(VisionConstants.kShooterToRobotCenter)
                .plus(kShooterToHood)
                .plus(new Transform3d(
                        new Translation3d(),
                        new Rotation3d(0, Units.degreesToRadians(-120) + hood.getPosition(), 0))));
    }

    public void aim(ShooterSetpoint setpoint) {
        setPos(setpoint.getHoodRadians(), setpoint.getHoodFF());
    }

    public void setPos(double position, double feedforward) {
        boolean underTrench = TrenchZone.hoodLowerRequired(robotState);
        hood.set(underTrench ? Math.min(position, kMaxSetpointUnderTrench) : position, feedforward);
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }
}
