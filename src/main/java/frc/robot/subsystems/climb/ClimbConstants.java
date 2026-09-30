package frc.robot.subsystems.climb;

import frc.robot.util.TunableNumber;
import frc.robot.util.motor.MotorConfig;
import frc.robot.util.motor.MotorConfig.Controller;

public final class ClimbConstants {
    public static final int kCurrentLimit = 50;

    public static final MotorConfig kClimb = new MotorConfig("Climb", 13, Controller.SPARK_FLEX)
            .currentLimit(kCurrentLimit)
            .conversion(1.0 / 16.0, 1.0 / 16.0 / 60.0)
            .pid(20, 0, 0)
            .maxMotion(4000, 7000, 0.05)
            .slot1(gains -> {
                gains.kP = 7;
                gains.maxAccel = 600;
                gains.cruiseVel = 600;
            });

    public static final TunableNumber kStowSetpoint = new TunableNumber("Climb/Stow Setpoint", 0);
    public static final TunableNumber kUpSetpoint = new TunableNumber("Climb/Up Setpoint", 4.1);
    public static final TunableNumber kDownSetpoint = new TunableNumber("Climb/Down Setpoint", 1.5);

    public static final TunableNumber kZeroCurrentLimit = new TunableNumber("Climb/Lower Current Limit", 30);
    public static final TunableNumber kZeroMotorOutput = new TunableNumber("Climb/Lower Motor Output", -0.08);
    public static final TunableNumber kZeroCurrentThreshold = new TunableNumber("Climb/Zero Current Threshold", 30);

    public static final int kLeftSensorId = 1;
    public static final int kRightSensorId = 2;

    private ClimbConstants() {}
}
