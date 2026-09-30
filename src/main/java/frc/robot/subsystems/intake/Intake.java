package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.util.TrenchZone;
import frc.robot.util.motor.Motor;
import frc.robot.util.state.StateMachine;

public class Intake extends StateMachine<Intake.State> {
    private final RobotState state;
    private final Motor rollers = new Motor(IntakeConstants.kRollers);
    private final Motor extension = new Motor(IntakeConstants.kExtension);
    private Runnable override;

    public Intake(RobotState state) {
        super("Intake", State.UNDETERMINED, State.class);
        this.state = state;

        addOmniTransitions(State.STOW, State.IDLE, State.INTAKE, State.OUTAKE, State.CLIMB_TOW, State.SHAKE);

        SmartDashboard.putData("Intake Zero", Commands.runOnce(() -> extension.setEncoderPosition(0))
                .ignoringDisable(true)
                .withName("Intake Zero"));
        enable();
    }

    @Override
    protected void update() {
        rollers.update();
        extension.update();

        if (TrenchZone.intakeLowerRequired(state)) {
            extension.setPosition(IntakeConstants.kIntakeSetpoint.get());
        } else if (override != null) {
            override.run();
        } else {
            switch (getState()) {
                case STOW -> stow();
                case IDLE -> rest();
                case INTAKE -> intake();
                case OUTAKE -> outtake();
                case CLIMB_TOW -> tow();
                case SHAKE -> shake();
                default -> stop();
            }
        }

        Logger.recordOutput("Intake/Overriden", override != null);
        Logger.recordOutput("Intake/Pose", new Pose3d()
                .plus(IntakeConstants.kOrigin)
                .plus(new Transform3d(
                        new Translation3d(),
                        new Rotation3d(0, -Units.degreesToRadians(extension.getPosition() + 90), 0))));
        Logger.recordOutput("Intake/ExtensionPose", new Pose3d(
                new Translation3d(
                        getState() != State.STOW ? Units.inchesToMeters(11) : 0,
                        0,
                        Units.inchesToMeters(-2.7)),
                new Rotation3d()));
    }

    public void stow() {
        extension.setPosition(IntakeConstants.kStowSetpoint.get());
        rollers.setVelocity(0);
    }

    public void rest() {
        extension.setPosition(IntakeConstants.kIntakeSetpoint.get());
        rollers.setVelocity(0);
    }

    public void intake() {
        extension.setPosition(IntakeConstants.kIntakeSetpoint.get());
        rollIn();
    }

    public void outtake() {
        extension.setPosition(IntakeConstants.kOuttakeSetpoint.get());
        rollOut();
    }

    public void tow() {
        extension.setPosition(IntakeConstants.kClimbTowSetpoint.get());
        rollers.setVelocity(0);
    }

    public void shake() {
        extension.setPosition(IntakeConstants.kShakeSetpoint.get());
        rollIn();
    }

    public void rollIn() {
        rollers.setVelocity(IntakeConstants.kIntakeRollerSpeed.get());
    }

    public void rollOut() {
        rollers.setVelocity(IntakeConstants.kOuttakeRollerSpeed.get());
    }

    public void stop() {
        extension.stop();
        rollers.setVelocity(0);
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
        INTAKE,
        OUTAKE,
        CLIMB_TOW,
        SHAKE
    }
}
