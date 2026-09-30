package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.game.TrenchZone;
import frc.robot.util.motor.PosMotor;
import frc.robot.util.motor.SpinMotor;

public class IntakeComp extends Intake {
    protected final RobotState robotState;
    protected final SpinMotor rollers = new SpinMotor(kRollers);
    protected final PosMotor extension = new PosMotor(kExtension);

    public IntakeComp(RobotState robotState) {
        this.robotState = robotState;
        addHardware(rollers, extension);
        allowAllTransitions();

        SmartDashboard.putData("Intake Zero", Commands.runOnce(() -> extension.resetPosition(0))
                .ignoringDisable(true)
                .withName("Intake Zero"));
        enable();
    }

    @Override
    protected void applyState(State state) {
        switch (state) {
            case STOW -> goTo(kStowSetpoint.get(), 0);
            case IDLE -> goTo(kIntakeSetpoint.get(), 0);
            case INTAKE -> goTo(kIntakeSetpoint.get(), kIntakeRollerSpeed.get());
            case OUTAKE -> goTo(kOuttakeSetpoint.get(), kOuttakeRollerSpeed.get());
            case SHAKE -> goTo(kShakeSetpoint.get(), kIntakeRollerSpeed.get());
            case UNDETERMINED -> {
                extension.stop();
                rollers.set(0);
            }
        }
    }

    @Override
    protected void applyConstraints() {
        if (TrenchZone.intakeLowerRequired(robotState)) {
            extension.set(kIntakeSetpoint.get());
        }
    }

    @Override
    protected void update() {
        Logger.recordOutput("Intake/Pose", new Pose3d()
                .plus(kOrigin)
                .plus(new Transform3d(
                        new Translation3d(),
                        new Rotation3d(0, -Units.degreesToRadians(extension.getPosition() + 90), 0))));
        Logger.recordOutput("Intake/ExtensionPose", new Pose3d(
                new Translation3d(getState() != State.STOW ? Units.inchesToMeters(11) : 0, 0, Units.inchesToMeters(-2.7)),
                new Rotation3d()));
    }

    protected void goTo(double extensionPosition, double rollerSpeed) {
        extension.set(extensionPosition);
        rollers.set(rollerSpeed);
    }

    @Override
    public void rollIn() {
        rollers.set(kIntakeRollerSpeed.get());
    }

    @Override
    public void rollOut() {
        rollers.set(kOuttakeRollerSpeed.get());
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }
}
