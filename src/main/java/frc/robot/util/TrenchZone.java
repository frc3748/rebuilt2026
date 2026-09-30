package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.FieldConstants;

public class TrenchZone {
    private static final double kHoodLowerRadius = 0.8;
    private static final double kIntakeLowerRadius = 1.0;

    public static double getDistanceToClosestTrench(RobotState state) {
        Translation2d robot = state.getLatestFieldToRobot().getValue().getTranslation();

        double nearY = FieldConstants.TRENCH_CENTER.in(Meters);
        double farY = FieldConstants.FIELD_WIDTH.in(Meters) - nearY;
        double allianceX = FieldConstants.TRENCH_BUMP_X.in(Meters);
        double opponentX = FieldConstants.FIELD_LENGTH.in(Meters) - allianceX;

        Translation2d[] trenches = {
                new Translation2d(allianceX, nearY),
                new Translation2d(allianceX, farY),
                new Translation2d(opponentX, nearY),
                new Translation2d(opponentX, farY)
        };

        double closest = Double.MAX_VALUE;
        for (Translation2d trench : trenches) {
            closest = Math.min(closest, robot.getDistance(trench));
        }
        return closest;
    }

    public static double getDistanceToClosestShootingPose(RobotState state) {
        Translation2d shooter = state.getLatestFieldToRobot().getValue().getTranslation()
                .plus(VisionConstants.kShooterToRobotCenter.getTranslation().toTranslation2d());
        double blueHub = VisionConstants.kBlueHubPose.toTranslation2d().getDistance(shooter);
        double redHub = VisionConstants.kRedHubPose.toTranslation2d().getDistance(shooter);
        return Math.min(blueHub, redHub);
    }

    public static boolean intakeLowerRequired(RobotState state) {
        return getDistanceToClosestTrench(state) < kIntakeLowerRadius;
    }

    public static boolean driveRotationOverrideRequired(RobotState state) {
        return intakeLowerRequired(state);
    }

    public static boolean hoodLowerRequired(RobotState state) {
        return getDistanceToClosestTrench(state) < kHoodLowerRadius;
    }
}
