package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {
    public static final Pose2d kLimelightErrorPose = new Pose2d(8.364, 4.141, Rotation2d.kZero);

    public static final double kLinearStdDevBaseline = 0.02;
    public static final double kAngularStdDevBaseline = 0.1;
    public static final double kMaxYawRateRadPerSec = Units.degreesToRadians(360.0);
    public static final double kStabilityWindowSeconds = 0.1;
    public static final double kMaxAmbiguity = 0.3;
    public static final double kMaxZErrorMeters = 0.75;
    public static final double kObjectMemorySeconds = 0.2;

    private VisionConstants() {}
}
