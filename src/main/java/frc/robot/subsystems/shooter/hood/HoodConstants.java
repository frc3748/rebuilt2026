package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TunableNumber;
import frc.robot.util.motor.MotorConfig;
import frc.robot.util.motor.MotorConfig.Controller;

public final class HoodConstants {
    private static final double kRadiansPerRotation = 2.0 * Math.PI / 3.0 / (367.0 / 32.0);

    public static final double kMinLimit = Units.degreesToRadians(25.0);
    public static final double kMaxLimit = Units.degreesToRadians(25.0) + kMinLimit;
    public static final double kMaxSetpointUnderTrench = Units.degreesToRadians(25.0);

    public static final MotorConfig kHood = new MotorConfig("Hood", 54, Controller.SPARK_MAX)
            .currentLimit(40)
            .conversion(kRadiansPerRotation, kRadiansPerRotation / 60.0)
            .pid(0.8, 0, 0)
            .feedforward(0.195, 0, 0)
            .gravity(0.401)
            .maxMotion(100000, 3000, 0)
            .softLimits(kMinLimit, kMaxLimit)
            .startingPosition(kMinLimit)
            .tunable(true, true);

    public static final TunableNumber kCustomSetpoint = new TunableNumber("Hood/Custom Setpoint", 0.0);

    public static final Transform3d kShooterToHood = new Transform3d(
            new Translation3d(Units.inchesToMeters(4.145), Units.inchesToMeters(0.954), Units.inchesToMeters(2.260)),
            new Rotation3d());

    private HoodConstants() {}
}
