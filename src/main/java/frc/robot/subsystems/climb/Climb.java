package frc.robot.subsystems.climb;

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
import frc.robot.util.motor.Motor;
import frc.robot.util.state.StateMachine;

public class Climb extends StateMachine<Climb.State> {
    private final Motor climb = new Motor(ClimbConstants.kClimb);
    private final BeamBreakerIO leftSensor = newSensor(ClimbConstants.kLeftSensorId);
    private final BeamBreakerIO rightSensor = newSensor(ClimbConstants.kRightSensorId);
    private final BeamBreakerInputsAutoLogged leftInputs = new BeamBreakerInputsAutoLogged();
    private final BeamBreakerInputsAutoLogged rightInputs = new BeamBreakerInputsAutoLogged();
    private Runnable override;

    public Climb() {
        super("Climb", State.UNDETERMINED, State.class);
        addOmniTransitions(State.STOW, State.IDLE, State.UP, State.DOWN);
        SmartDashboard.putData("Climb Zero", zero().withName("Climb Zero"));
        enable();
    }

    private static BeamBreakerIO newSensor(int canId) {
        return Constants.kMode == Mode.REAL ? new BeamBreakerTOF(canId) : new BeamBreakerIO() {};
    }

    @Override
    protected void update() {
        climb.update();
        leftSensor.updateInputs(leftInputs);
        rightSensor.updateInputs(rightInputs);
        Logger.processInputs("Climb/Left Sensor", leftInputs);
        Logger.processInputs("Climb/Right Sensor", rightInputs);

        if (override != null) {
            override.run();
        } else {
            switch (getState()) {
                case STOW -> stow();
                case UP -> up();
                case DOWN -> down();
                default -> stop();
            }
        }

        Logger.recordOutput("Climb/Overriden", override != null);
        Logger.recordOutput("Climb/Pose", new Pose3d(
                new Translation3d(0, 0, climb.getPosition() / 16.0),
                new Rotation3d()));
    }

    public void stow() {
        climb.setPosition(ClimbConstants.kStowSetpoint.get());
    }

    public void up() {
        climb.setPosition(ClimbConstants.kUpSetpoint.get());
    }

    public void down() {
        climb.setPosition(ClimbConstants.kDownSetpoint.get(), 0, 1);
    }

    public void stop() {
        climb.stop();
    }

    public Command zero() {
        return run(() -> climb.setOutput(ClimbConstants.kZeroMotorOutput.get()))
                .beforeStarting(() -> climb.setCurrentLimit((int) ClimbConstants.kZeroCurrentLimit.get()))
                .until(() -> climb.getCurrentAmps() > ClimbConstants.kZeroCurrentThreshold.get())
                .finallyDo(() -> {
                    climb.stop();
                    climb.setEncoderPosition(0);
                    climb.setCurrentLimit(ClimbConstants.kCurrentLimit);
                    requestTransition(State.STOW);
                    Elastic.sendNotification(
                            new Notification().withTitle("Climb Zero").withDescription("Climb has been zeroed!"));
                });
    }

    public double getLeftSensorDistance() {
        return leftInputs.distanceMeters;
    }

    public double getRightSensorDistance() {
        return rightInputs.distanceMeters;
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
        STOW,
        IDLE,
        UP,
        DOWN
    }
}
