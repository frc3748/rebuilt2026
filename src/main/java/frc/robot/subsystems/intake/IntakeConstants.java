package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TunableNumber;
import frc.robot.util.motor.MotorConfig;
import frc.robot.util.motor.MotorConfig.Controller;

public final class IntakeConstants {
    public static final MotorConfig kRollers = new MotorConfig("Intake Roller", 48, Controller.SPARK_FLEX)
            .currentLimit(80)
            .conversion(1.0, 1.0 / 60.0)
            .pid(0.022, 0, 0)
            .tunable(false, false);

    public static final MotorConfig kExtension = new MotorConfig("Intake Extension", 46, Controller.SPARK_MAX)
            .follower(47, true)
            .currentLimit(60)
            .conversion(360.0 / 23.0, 360.0 / 23.0 / 60.0)
            .pid(0.09, 0, 0)
            .feedforward(0.17, 0.00131, 0)
            .cosineGravity(0.24)
            .maxMotion(600, 130, 2)
            .tunable(true, true);

    public static final TunableNumber kStowSetpoint = new TunableNumber("Intake/Extension Stow Setpoint", -93);
    public static final TunableNumber kIntakeSetpoint = new TunableNumber("Intake/Extension Intake Setpoint", 0);
    public static final TunableNumber kOuttakeSetpoint = new TunableNumber("Intake/Extension Outtake Setpoint", 0);
    public static final TunableNumber kClimbTowSetpoint = new TunableNumber("Intake/Extension Tow Setpoint", -30);
    public static final TunableNumber kShakeSetpoint = new TunableNumber("Intake/Extension Shake Setpoint", -30);

    public static final TunableNumber kIntakeRollerSpeed = new TunableNumber("Intake/Roller Intake Speed", -40);
    public static final TunableNumber kOuttakeRollerSpeed = new TunableNumber("Intake/Roller Outtake Speed", 40);

    public static final Transform3d kOrigin = new Transform3d(
            new Translation3d(Units.inchesToMeters(5), 0, Units.inchesToMeters(6.7)),
            new Rotation3d());

    private IntakeConstants() {}
}
