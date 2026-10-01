package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TunableNumber;

public final class VisionConstants {
    public static final Pose2d kLimelightErrorPose = new Pose2d(8.364, 4.141, Rotation2d.kZero);

    public static final TunableNumber kLinearStdDevBaseline = new TunableNumber("Vision/Linear Std Dev", 0.02);
    public static final TunableNumber kAngularStdDevBaseline = new TunableNumber("Vision/Angular Std Dev", 0.1);
    public static final double kMaxYawRateRadPerSec = Units.degreesToRadians(360.0);
    public static final double kStabilityWindowSeconds = 0.1;
    public static final TunableNumber kMaxAmbiguity = new TunableNumber("Vision/Max Ambiguity", 0.3);
    public static final TunableNumber kMaxZErrorMeters = new TunableNumber("Vision/Max Z Error", 0.75);
    public static final double kObjectMemorySeconds = 0.5;
    public static final double kObjectMergeMeters = 0.15;
    public static final double kSimObjectRangeMeters = 5.0;
    public static final int kSimMaxObjects = 16;

    public static final int kStrictHeadingMinTags = 2;
    public static final double kStrictHeadingMaxDistanceMeters = 4.0;
    public static final double kStrictHeadingMaxAmbiguity = 0.15;
    public static final double kStrictHeadingMaxYawRateRadPerSec = Units.degreesToRadians(30.0);
    public static final int kHeadingSamples = 5;
    public static final double kHeadingWindowSeconds = 1.0;
    public static final double kHeadingAgreementDegrees = 1.0;
    public static final double kHeadingCorrectionDegrees = 0.25;
    public static final double kNoVisionSeconds = 5.0;
    public static final double kDisagreeMeters = 0.5;

    private VisionConstants() {}
}
