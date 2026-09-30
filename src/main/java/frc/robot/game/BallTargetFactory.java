package frc.robot.game;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;

import org.littletonrobotics.junction.Logger;

public class BallTargetFactory {
    static InterpolatingTreeMap<Double, Double> heightMap = new InterpolatingTreeMap<Double, Double>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());
    static {
        double scale = 0.5;

        heightMap.put(5.34, 5.34 * Math.tan(Math.toRadians(27)) * scale);
        heightMap.put(4.90, 4.90 * Math.tan(Math.toRadians(26)) * scale);
        heightMap.put(4.44, 4.44 * Math.tan(Math.toRadians(25.5)) * scale);
        heightMap.put(4.05, 4.05 * Math.tan(Math.toRadians(25)) * scale);
        heightMap.put(3.74, 3.74 * Math.tan(Math.toRadians(24)) * scale);
        heightMap.put(3.42, 3.42 * Math.tan(Math.toRadians(23)) * scale);
        heightMap.put(3.06, 3.06 * Math.tan(Math.toRadians(22)) * scale);
        heightMap.put(2.73, 2.73 * Math.tan(Math.toRadians(20.5)) * scale);
        heightMap.put(2.45, 2.45 * Math.tan(Math.toRadians(19.5)) * scale);
        heightMap.put(2.14, 2.14 * Math.tan(Math.toRadians(18)) * scale);
        heightMap.put(1.86, 1.86 * Math.tan(Math.toRadians(17)) * scale);
        heightMap.put(1.55, 1.55 * Math.tan(Math.toRadians(15)) * scale);
    }
    static InterpolatingTreeMap<Double, Double> distanceOffsetMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());
    static {
        distanceOffsetMap.put(1.4, Units.inchesToMeters(0.0));
        distanceOffsetMap.put(3.0, Units.inchesToMeters(0.0));
    }

    static Double kXDistanceOffset = Units.inchesToMeters(0);

    public static Translation3d generate(RobotState robotState) {
        var speakerPose = AllianceFlip.isRed() ? FieldConstants.HUB_RED : FieldConstants.HUB_BLUE;

        double distance = new Translation2d(speakerPose.getX(), speakerPose.getY()).getDistance(
                robotState.getLatestFieldToRobot().getValue().getTranslation());

        double distanceOffset = distanceOffsetMap.get(distance);
        var offSet = new Translation2d(kXDistanceOffset, -distanceOffset);

        if (AllianceFlip.isRed()) {
            offSet = new Translation2d(-offSet.getX(), offSet.getY());
        }

        Logger.recordOutput("BallTargetFactory/distanceFromTarget", distance);
        speakerPose = new Translation3d(
                speakerPose.getX() + offSet.getX(), speakerPose.getY() + offSet.getY(),
                speakerPose.getZ() + heightMap.get(distance));

        Logger.recordOutput("targetPose", speakerPose);
        Logger.recordOutput("targetPose2d", new Translation2d(speakerPose.getX(), speakerPose.getY()));
        return speakerPose;
    }
}