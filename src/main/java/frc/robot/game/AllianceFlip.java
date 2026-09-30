package frc.robot.game;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class AllianceFlip {
    public static boolean isRed() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }

    public static Pose2d flip(Pose2d pose) {
        return new Pose2d(
                new Translation2d(
                        FieldConstants.LAYOUT_LENGTH_METERS - pose.getX(),
                        FieldConstants.LAYOUT_WIDTH_METERS - pose.getY()),
                pose.getRotation().rotateBy(Rotation2d.k180deg));
    }

    public static Translation3d flip(Translation3d translation) {
        return new Translation3d(
                FieldConstants.LAYOUT_LENGTH_METERS - translation.getX(),
                FieldConstants.LAYOUT_WIDTH_METERS - translation.getY(),
                translation.getZ());
    }

    public static Pose2d forAlliance(Pose2d bluePose) {
        return isRed() ? flip(bluePose) : bluePose;
    }

    private AllianceFlip() {}
}
