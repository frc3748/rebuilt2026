package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.motor.MotorConfig;
import frc.robot.util.motor.MotorConfig.Controller;

public class HoodConstants {
    public double radiansPerRotation = 2.0 * Math.PI / 3.0 / (367.0 / 32.0);
    public double minLimit = Units.degreesToRadians(25.0);
    public double maxLimit = Units.degreesToRadians(50.0);
    public double readyTolerance = Units.degreesToRadians(1.5);

    public MotorConfig motor = new MotorConfig("Hood", 54, Controller.SPARK_MAX)
            .currentLimit(40)
            .pid(0.8, 0, 0)
            .feedforward(0.195, 0, 0)
            .gravity(0.401)
            .maxMotion(100000, 3000, 0)
            .quadratureFilter(2, 10);

    public double customSetpoint = 0.0;

    public Transform3d shooterToHood = new Transform3d(
            new Translation3d(Units.inchesToMeters(4.145), Units.inchesToMeters(0.954), Units.inchesToMeters(2.260)),
            new Rotation3d());
}
