package frc.robot.game;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotState;
import frc.robot.util.TunableNumber;

public class TrenchZone {
    private static final TunableNumber kHoodLowerRadius = new TunableNumber("Trench/Hood Down Radius", 0.8);
    private static final TunableNumber kIntakeLowerRadius = new TunableNumber("Trench/Intake Down Radius", 1.0);

    public static double getDistanceToClosestTrench(RobotState state) {
        return state.getLatestFieldToRobot().getValue().getTranslation().getDistance(closestTrench(state));
    }

    public static Translation2d closestTrench(RobotState state) {
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

        Translation2d closest = trenches[0];
        for (Translation2d trench : trenches) {
            if (robot.getDistance(trench) < robot.getDistance(closest)) {
                closest = trench;
            }
        }
        return closest;
    }

    public static double hoodLowerRadius() {
        return kHoodLowerRadius.get();
    }

    public static double getDistanceToClosestShootingPose(RobotState state) {
        Translation2d shooter = state.getLatestFieldToRobot().getValue().getTranslation()
                .plus(state.getShooterConstants().shooterToRobotCenter.getTranslation().toTranslation2d());
        double blueHub = FieldConstants.HUB_BLUE.toTranslation2d().getDistance(shooter);
        double redHub = FieldConstants.HUB_RED.toTranslation2d().getDistance(shooter);
        return Math.min(blueHub, redHub);
    }

    public static boolean intakeLowerRequired(RobotState state) {
        return getDistanceToClosestTrench(state) < kIntakeLowerRadius.get();
    }

    public static boolean driveRotationOverrideRequired(RobotState state) {
        return intakeLowerRequired(state);
    }

    public static boolean hoodLowerRequired(RobotState state) {
        return getDistanceToClosestTrench(state) < kHoodLowerRadius.get();
    }
}
