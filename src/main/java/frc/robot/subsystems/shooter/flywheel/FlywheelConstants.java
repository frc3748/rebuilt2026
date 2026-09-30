package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import frc.robot.util.motor.MotorConfig;
import frc.robot.util.motor.MotorConfig.Controller;

public class FlywheelConstants {
    public Distance radius = Inches.of(2);

    public MotorConfig motor = new MotorConfig("Flywheel", 55, Controller.SPARK_FLEX)
            .follower(56, false)
            .coast()
            .currentLimit(60)
            .pid(0.5, 0, 0)
            .feedforward(0.0, 0.38, 0.0)
            .maxMotion(100000000, 100000000, 0.01)
            .outputRange(0, 1)
            .tunable(true, false);

    public double speedTolerance = 2;
    public double customSetpoint = 100.0;
    public double slowSpeed = 0.0;

    public double metersPerRotation() {
        return radius.in(Meters) * Math.PI * 2.0;
    }
}
