package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.TunableNumber;
import frc.robot.util.motor.MotorConfig;
import frc.robot.util.motor.MotorConfig.Controller;

public final class FlywheelConstants {
    public static final Distance kFlywheelRadius = Inches.of(2);
    private static final double kMetersPerRotation = kFlywheelRadius.in(Meters) * Math.PI * 2.0;

    public static final MotorConfig kFlywheel = new MotorConfig("Flywheel", 55, Controller.SPARK_FLEX)
            .follower(56, false)
            .coast()
            .currentLimit(60)
            .conversion(kMetersPerRotation, kMetersPerRotation / 60.0)
            .pid(0.5, 0, 0)
            .feedforward(0.0, 0.38, 0.0)
            .maxMotion(100000000, 100000000, 0.01)
            .outputRange(0, 1)
            .tunable(true, false);

    public static final TunableNumber kSpeedTolerance = new TunableNumber("Flywheel/Speed Tolerance", 2);
    public static final TunableNumber kCustomSetpoint = new TunableNumber("Flywheel/Custom Setpoint", 100.0);

    public static final double kSlowSpeed = 0.0;
    public static final double kPassMaxApexHeight = Units.inchesToMeters(160);

    private FlywheelConstants() {}
}
