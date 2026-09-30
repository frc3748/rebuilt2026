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
import frc.robot.game.TrenchZone;
import frc.robot.util.TunableNumber;
import frc.robot.util.motor.PosMotor;
import frc.robot.util.motor.SpinMotor;

public class IntakeComp extends Intake {
    protected final RobotState robotState;
    protected final IntakeConstants constants;
    protected final SpinMotor rollers;
    protected final PosMotor extension;
    protected final TunableNumber stowSetpoint;
    protected final TunableNumber intakeSetpoint;
    protected final TunableNumber outtakeSetpoint;
    protected final TunableNumber shakeSetpoint;
    protected final TunableNumber intakeRollerSpeed;
    protected final TunableNumber outtakeRollerSpeed;

    public IntakeComp(RobotState robotState, IntakeConstants constants) {
        this.robotState = robotState;
        this.constants = constants;
        rollers = new SpinMotor(constants.rollers);
        extension = new PosMotor(constants.extension);
        stowSetpoint = new TunableNumber("Intake/Extension Stow Setpoint", constants.stowSetpoint);
        intakeSetpoint = new TunableNumber("Intake/Extension Intake Setpoint", constants.intakeSetpoint);
        outtakeSetpoint = new TunableNumber("Intake/Extension Outtake Setpoint", constants.outtakeSetpoint);
        shakeSetpoint = new TunableNumber("Intake/Extension Shake Setpoint", constants.shakeSetpoint);
        intakeRollerSpeed = new TunableNumber("Intake/Roller Intake Speed", constants.intakeRollerSpeed);
        outtakeRollerSpeed = new TunableNumber("Intake/Roller Outtake Speed", constants.outtakeRollerSpeed);

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
            case STOW -> goTo(stowSetpoint.get(), 0);
            case IDLE -> goTo(intakeSetpoint.get(), 0);
            case INTAKE -> goTo(intakeSetpoint.get(), intakeRollerSpeed.get());
            case OUTAKE -> goTo(outtakeSetpoint.get(), outtakeRollerSpeed.get());
            case SHAKE -> goTo(shakeSetpoint.get(), intakeRollerSpeed.get());
            case UNDETERMINED -> {
                extension.stop();
                rollers.set(0);
            }
        }
    }

    @Override
    protected void applyConstraints() {
        if (TrenchZone.intakeLowerRequired(robotState)) {
            extension.set(intakeSetpoint.get());
        }
    }

    @Override
    protected void update() {
        Logger.recordOutput("Intake/Pose", new Pose3d()
                .plus(constants.origin)
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
        rollers.set(intakeRollerSpeed.get());
    }

    @Override
    public void rollOut() {
        rollers.set(outtakeRollerSpeed.get());
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }
}
