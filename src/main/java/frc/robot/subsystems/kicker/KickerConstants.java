package frc.robot.subsystems.kicker;

import frc.robot.util.TunableNumber;
import frc.robot.util.motor.MotorConfig;
import frc.robot.util.motor.MotorConfig.Controller;

public final class KickerConstants {
    public static final MotorConfig kKicker = new MotorConfig("Kicker", 42, Controller.SPARK_MAX)
            .currentLimit(40)
            .conversion(1.0 / 3.0, 1.0 / 3.0 / 60.0)
            .pid(0.025, 0, 0)
            .maxMotion(100000, 100000, 0);

    public static final TunableNumber kShootSpeed = new TunableNumber("Kicker/Shot Speed", -40);
    public static final TunableNumber kOuttakeSpeed = new TunableNumber("Kicker/Outtake Speed", 30);

    private KickerConstants() {}
}
