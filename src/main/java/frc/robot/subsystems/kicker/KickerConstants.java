package frc.robot.subsystems.kicker;

import frc.robot.util.motor.MotorConfig;
import frc.robot.util.motor.MotorConfig.Controller;

public class KickerConstants {
    public MotorConfig motor = new MotorConfig("Kicker", 42, Controller.SPARK_MAX)
            .currentLimit(40)
            .conversion(1.0 / 3.0, 1.0 / 3.0 / 60.0)
            .pid(0.025, 0, 0)
            .maxMotion(100000, 100000, 0);

    public double shootSpeed = -40;
    public double outtakeSpeed = 30;
}
