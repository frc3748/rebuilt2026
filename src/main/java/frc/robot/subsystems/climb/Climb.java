package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.motor.PosMotor;
import frc.robot.util.state.StateMachine;

public class Climb extends StateMachine<Climb.State> {
    public enum State {
        UNDETERMINED,
        STOW,
        IDLE,
        UP,
        DOWN,
        ZEROING
    }

    private final PosMotor climb = new PosMotor(kClimb);
    private final BeamBreaker leftSensor = new BeamBreaker("Climb/Left Sensor", kLeftSensorId);
    private final BeamBreaker rightSensor = new BeamBreaker("Climb/Right Sensor", kRightSensorId);

    public Climb() {
        super("Climb", State.UNDETERMINED, State.class);
        addHardware(climb, leftSensor, rightSensor);
        allowAllTransitions();

        registerStateCommand(State.ZEROING, () -> climb.setCurrentLimit((int) kZeroCurrentLimit.get()));
        for (State next : State.values()) {
            if (next != State.ZEROING && next != State.UNDETERMINED) {
                addTransition(State.ZEROING, next, this::restoreCurrentLimit);
            }
        }

        SmartDashboard.putData("Climb Zero", zero().withName("Climb Zero"));
        enable();
    }

    @Override
    protected void applyState(State state) {
        switch (state) {
            case STOW -> climb.set(kStowSetpoint.get());
            case UP -> climb.set(kUpSetpoint.get());
            case DOWN -> climb.set(kDownSetpoint.get(), 0, 1);
            case ZEROING -> climb.setOutput(kZeroMotorOutput.get());
            case IDLE, UNDETERMINED -> climb.stop();
        }
    }

    @Override
    protected void update() {
        if (getState() == State.ZEROING && !isTransitioning() && isStalled()) {
            climb.resetPosition(0);
            requestTransition(State.STOW);
            Elastic.sendNotification(new Notification().withTitle("Climb Zero").withDescription("Climb has been zeroed!"));
        }
        Logger.recordOutput("Climb/Pose", new Pose3d(new Translation3d(0, 0, climb.getPosition() / 16.0), new Rotation3d()));
    }

    private boolean isStalled() {
        return Constants.kMode == Mode.SIM || climb.getCurrentAmps() > kZeroCurrentThreshold.get();
    }

    private void restoreCurrentLimit() {
        climb.setCurrentLimit(kCurrentLimit);
    }

    @Override
    protected void onDisable() {
        restoreCurrentLimit();
    }

    public Command zero() {
        return transitionCommand(State.ZEROING, false);
    }

    public double getLeftSensorDistance() {
        return leftSensor.getDistanceMeters();
    }

    public double getRightSensorDistance() {
        return rightSensor.getDistanceMeters();
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }
}
