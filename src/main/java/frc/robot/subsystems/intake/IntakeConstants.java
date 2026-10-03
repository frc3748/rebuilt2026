package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.motor.MotorConfig;
import frc.robot.util.motor.MotorConfig.Controller;

public class IntakeConstants {
    public MotorConfig rollers = new MotorConfig("Intake Roller", 48, Controller.SPARK_FLEX)
            .currentLimit(80)
            .conversion(1.0, 1.0 / 60.0)
            .pid(0.022, 0, 0);

    public MotorConfig extension = new MotorConfig("Intake Extension", 46, Controller.SPARK_MAX)
            .follower(47, true)
            .currentLimit(60)
            .conversion(360.0 / 23.0, 360.0 / 23.0 / 60.0)
            .pid(0.09, 0, 0)
            .feedforward(0.17, 0.00131, 0)
            .cosineGravity(0.24, 360.0)
            .maxMotion(600, 130, 2)
            .selfTestPosition(-45);

    public double stowSetpoint = -93;
    public double intakeSetpoint = 0;
    public double outtakeSetpoint = 0;
    public double shakeSetpoint = -30;
    public double intakeRollerSpeed = -40;
    public double outtakeRollerSpeed = 40;

    public Transform3d origin = new Transform3d(
            new Translation3d(Units.inchesToMeters(5), 0, Units.inchesToMeters(6.7)),
            new Rotation3d());
}
