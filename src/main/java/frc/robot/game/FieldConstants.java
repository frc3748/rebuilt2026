package frc.robot.game;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

public final class FieldConstants {
    public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    public static final int[] TAG_IDS = {29, 13, 30, 14, 31, 15, 32, 16, 28, 12, 17, 1, 23, 7, 22, 6, 26, 10, 25, 9, 21, 5, 24, 8, 18, 2, 27, 11, 20, 4, 19, 3};
    public static final double LAYOUT_LENGTH_METERS = TAG_LAYOUT.getFieldLength();
    public static final double LAYOUT_WIDTH_METERS = TAG_LAYOUT.getFieldWidth();

    public static final Distance FIELD_LENGTH = Inches.of(650.12);
    public static final Distance FIELD_WIDTH = Inches.of(316.64);

    public static final Translation3d HUB_BLUE = new Translation3d(Inches.of(181.56), FIELD_WIDTH.div(2), Inches.of(56.4));
    public static final Translation3d HUB_RED = new Translation3d(FIELD_LENGTH.minus(Inches.of(181.56)), FIELD_WIDTH.div(2), Inches.of(56.4));
    public static final Pose2d HUB_NEAR_FACE = TAG_LAYOUT.getTagPose(26).get().toPose2d();
    public static final Distance FUNNEL_RADIUS = Inches.of(24);
    public static final Distance FUNNEL_HEIGHT = Inches.of(72 - 56.4);

    public static final Distance TRENCH_BUMP_X = Inches.of(181.56);
    public static final Distance TRENCH_WIDTH = Inches.of(49.86);
    public static final Distance TRENCH_CENTER = TRENCH_WIDTH.div(2);

    public static final Distance FUEL_DIAMETER = Inches.of(5.91);

    private FieldConstants() {}
}
