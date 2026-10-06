package frc.robot.subsystems.hopper;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.motor.MotorConfig;
import frc.robot.util.motor.MotorConfig.Controller;

public class HopperConstants {
    public MotorConfig motor = new MotorConfig("Hopper", 15, Controller.SPARK_FLEX)
            .currentLimit(40)
            .conversion(1.0 / 9.0, 0.0001852)
            .pid(0.2, 0, 0)
            .maxMotion(200, 1000, 0)
            .quadratureFilter(2, 10);

    public double shootSpeed = -30;
    public double outtakeSpeed = 15;
    public double rollerRadiusMeters = 0.0254;

    public Transform3d origin = new Transform3d(
            new Translation3d(Units.inchesToMeters(2.5), Units.inchesToMeters(0.4), 0),
            new Rotation3d());
}
