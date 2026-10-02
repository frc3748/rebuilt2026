package frc.robot.game;

import static edu.wpi.first.units.Units.Meters;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class FieldObstacles {
    private static final double kTrenchWallHalfX = Units.inchesToMeters(53.0) / 2.0;
    private static final double kTrenchWallHalfY = Units.inchesToMeters(12.0) / 2.0;
    private static final double kHubHalf = Units.inchesToMeters(47.0) / 2.0;

    public record Box(double centerX, double centerY, double halfX, double halfY) {}

    public static final List<Box> BOXES = boxes();

    private FieldObstacles() {}

    private static List<Box> boxes() {
        double length = FieldConstants.LAYOUT_LENGTH_METERS;
        double width = FieldConstants.LAYOUT_WIDTH_METERS;
        double trenchX = FieldConstants.TRENCH_BUMP_X.in(Meters);
        double wallY = FieldConstants.TRENCH_WIDTH.in(Meters) + kTrenchWallHalfY;
        return List.of(
                new Box(trenchX, wallY, kTrenchWallHalfX, kTrenchWallHalfY),
                new Box(trenchX, width - wallY, kTrenchWallHalfX, kTrenchWallHalfY),
                new Box(length - trenchX, wallY, kTrenchWallHalfX, kTrenchWallHalfY),
                new Box(length - trenchX, width - wallY, kTrenchWallHalfX, kTrenchWallHalfY),
                new Box(FieldConstants.HUB_BLUE.getX(), FieldConstants.HUB_BLUE.getY(), kHubHalf, kHubHalf),
                new Box(FieldConstants.HUB_RED.getX(), FieldConstants.HUB_RED.getY(), kHubHalf, kHubHalf));
    }

    public static double halfX(Rotation2d heading, double length, double width) {
        return length / 2.0 * Math.abs(heading.getCos()) + width / 2.0 * Math.abs(heading.getSin());
    }

    public static double halfY(Rotation2d heading, double length, double width) {
        return length / 2.0 * Math.abs(heading.getSin()) + width / 2.0 * Math.abs(heading.getCos());
    }

    public static Translation2d insideWalls(Translation2d center, Rotation2d heading, double length, double width, double margin) {
        double halfX = halfX(heading, length, width) + margin;
        double halfY = halfY(heading, length, width) + margin;
        return new Translation2d(
                MathUtil.clamp(center.getX(), halfX, FieldConstants.LAYOUT_LENGTH_METERS - halfX),
                MathUtil.clamp(center.getY(), halfY, FieldConstants.LAYOUT_WIDTH_METERS - halfY));
    }

    public static double clearance(Translation2d center, Rotation2d heading, double length, double width) {
        double halfX = halfX(heading, length, width);
        double halfY = halfY(heading, length, width);
        double gap = Math.min(
                Math.min(center.getX() - halfX, FieldConstants.LAYOUT_LENGTH_METERS - center.getX() - halfX),
                Math.min(center.getY() - halfY, FieldConstants.LAYOUT_WIDTH_METERS - center.getY() - halfY));
        for (Box box : BOXES) {
            gap = Math.min(gap, Math.max(Math.abs(center.getX() - box.centerX()) - box.halfX() - halfX,
                    Math.abs(center.getY() - box.centerY()) - box.halfY() - halfY));
        }
        return gap;
    }
}
