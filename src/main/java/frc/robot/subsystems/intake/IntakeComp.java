package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.util.TunableNumber;
import frc.robot.util.Visuals;
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
        extension = new PosMotor(constants.extension.tuneRange(constants.stowSetpoint, constants.intakeSetpoint));
        stowSetpoint = TunableNumber.field("Intake/Extension Stow Setpoint", constants, "stowSetpoint");
        intakeSetpoint = TunableNumber.field("Intake/Extension Intake Setpoint", constants, "intakeSetpoint");
        outtakeSetpoint = TunableNumber.field("Intake/Extension Outtake Setpoint", constants, "outtakeSetpoint");
        shakeSetpoint = TunableNumber.field("Intake/Extension Shake Setpoint", constants, "shakeSetpoint");
        intakeRollerSpeed = TunableNumber.field("Intake/Roller Intake Speed", constants, "intakeRollerSpeed");
        outtakeRollerSpeed = TunableNumber.field("Intake/Roller Outtake Speed", constants, "outtakeRollerSpeed");

        addHardware(rollers, extension);
        allowAllTransitions();

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
    protected void update() {
        Visuals.record("Intake/Pose", new Pose3d()
                .plus(constants.origin)
                .plus(new Transform3d(
                        new Translation3d(),
                        new Rotation3d(0, -Units.degreesToRadians(extension.getPosition() + 90), 0))));
        Visuals.record("Intake/ExtensionPose", new Pose3d(
                new Translation3d(getState() != State.STOW ? Units.inchesToMeters(11) : 0, 0, Units.inchesToMeters(-2.7)),
                new Rotation3d()));
    }

    protected void goTo(double extensionPosition, double rollerSpeed) {
        extension.set(extensionPosition);
        rollers.set(rollerSpeed);
    }

    @Override
    public void zero() {
        extension.resetPosition(0);
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
