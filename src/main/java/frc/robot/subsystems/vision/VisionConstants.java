package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public class VisionConstants {
        public static final int[] kValidTagIds = new int[] {29,13,30,14,31,15, 32,16, 28,12,17,1, 23,7, 22,6, 26,10, 25,9,21,5,24,8, 18,2, 27,11, 20,4, 19,3};
        public static final Pose2d kErrorPoseRed = new Pose2d(8.364, 4.141, Rotation2d.fromDegrees(0));

        public static final double kLinearStdDevBaseline = 0.02;
        public static final double kAngularStdDevBaseline = 0.1;
        public static final double kLinearStdDevMegatag2Factor = 0.5;
        public static final double kMaxYawRateRadPerSec = Units.degreesToRadians(360.0);
        public static final double kStabilityWindowSeconds = 0.1;

        public static final Transform3d kShooterToRobotCenter = new Transform3d(
                new Translation3d(Units.inchesToMeters(-3.290), Units.inchesToMeters(-4.750), Units.inchesToMeters(13.735 - 0.45)),
                Rotation3d.kZero);

        public static final CameraConfig kShooterCamera = new CameraConfig("Shooter Camera", "limelight-turret", CameraConfig.Type.LIMELIGHT)
                .robotToCamera(kShooterToRobotCenter.plus(new Transform3d(
                        new Translation3d(Units.inchesToMeters(4.594), Units.inchesToMeters(4.270), Units.inchesToMeters(4.181)),
                        new Rotation3d(0, Units.degreesToRadians(-20.5), 0))))
                .stdDevFactor(1.3);

        public static final CameraConfig kChassisCamera = new CameraConfig("Chassis Camera", "limelight", CameraConfig.Type.LIMELIGHT)
                .robotToCamera(new Transform3d(
                        new Translation3d(Units.inchesToMeters(13), Units.inchesToMeters(0.75), Units.inchesToMeters(5.75)),
                        new Rotation3d(0, Units.degreesToRadians(-45), Units.degreesToRadians(180))));

        public static final CameraConfig[] kCameras = { kChassisCamera, kShooterCamera };

        public static final AprilTagFieldLayout kAprilTagLayout = AprilTagFieldLayout
                        .loadField(AprilTagFields.k2026RebuiltAndymark);

        public static final double kFieldWidthMeters = kAprilTagLayout.getFieldWidth();
        public static final double kFieldLengthMeters = kAprilTagLayout.getFieldLength();

        public static final Translation3d kBlueHubPose = FieldConstants.HUB_BLUE;
        public static final Translation3d kRedHubPose = FieldConstants.HUB_RED;

        public static final int aprilTagCount = kAprilTagLayout.getTags().size();
        public static final double aprilTagWidth = Units.inchesToMeters(6.5);

        public static final double fieldLength = kAprilTagLayout.getFieldLength();
        public static final double fieldWidth = kAprilTagLayout.getFieldWidth();

        public static class LinesVertical {
                public static final double center = fieldLength / 2.0;
                public static final double starting = kAprilTagLayout.getTagPose(26).get().getX();
                public static final double allianceZone = starting;
                public static final double hubCenter = kAprilTagLayout.getTagPose(26).get().getX() + Hub.width / 2.0;
                public static final double neutralZoneNear = center - Units.inchesToMeters(120);
                public static final double neutralZoneFar = center + Units.inchesToMeters(120);
                public static final double oppHubCenter = kAprilTagLayout.getTagPose(4).get().getX() + Hub.width / 2.0;
                public static final double oppAllianceZone = kAprilTagLayout.getTagPose(10).get().getX();
        }

        public static class LinesHorizontal {
                public static final double center = fieldWidth / 2.0;

                public static final double rightBumpStart = Hub.nearRightCorner.getY();
                public static final double rightBumpEnd = rightBumpStart - RightBump.width;
                public static final double rightTrenchOpenStart = rightBumpEnd - Units.inchesToMeters(12.0);
                public static final double rightTrenchOpenEnd = 0;

                public static final double leftBumpEnd = Hub.nearLeftCorner.getY();
                public static final double leftBumpStart = leftBumpEnd + LeftBump.width;
                public static final double leftTrenchOpenEnd = leftBumpStart + Units.inchesToMeters(12.0);
                public static final double leftTrenchOpenStart = fieldWidth;
        }

        public static class Hub {
                public static final double width = Units.inchesToMeters(47.0);
                public static final double height = Units.inchesToMeters(72.0);
                public static final double innerWidth = Units.inchesToMeters(41.7);
                public static final double innerHeight = Units.inchesToMeters(56.5);

                public static final Translation3d topCenterPoint = new Translation3d(
                                kAprilTagLayout.getTagPose(26).get().getX() + width / 2.0,
                                fieldWidth / 2.0,
                                height);
                public static final Translation3d innerCenterPoint = new Translation3d(
                                kAprilTagLayout.getTagPose(26).get().getX() + width / 2.0,
                                fieldWidth / 2.0,
                                innerHeight);

                public static final Translation2d nearLeftCorner = new Translation2d(
                                topCenterPoint.getX() - width / 2.0,
                                fieldWidth / 2.0 + width / 2.0);
                public static final Translation2d nearRightCorner = new Translation2d(
                                topCenterPoint.getX() - width / 2.0,
                                fieldWidth / 2.0 - width / 2.0);
                public static final Translation2d farLeftCorner = new Translation2d(topCenterPoint.getX() + width / 2.0,
                                fieldWidth / 2.0 + width / 2.0);
                public static final Translation2d farRightCorner = new Translation2d(
                                topCenterPoint.getX() + width / 2.0,
                                fieldWidth / 2.0 - width / 2.0);

                public static final Translation3d oppTopCenterPoint = new Translation3d(
                                kAprilTagLayout.getTagPose(4).get().getX() + width / 2.0,
                                fieldWidth / 2.0,
                                height);
                public static final Translation2d oppNearLeftCorner = new Translation2d(
                                oppTopCenterPoint.getX() - width / 2.0,
                                fieldWidth / 2.0 + width / 2.0);
                public static final Translation2d oppNearRightCorner = new Translation2d(
                                oppTopCenterPoint.getX() - width / 2.0,
                                fieldWidth / 2.0 - width / 2.0);
                public static final Translation2d oppFarLeftCorner = new Translation2d(
                                oppTopCenterPoint.getX() + width / 2.0,
                                fieldWidth / 2.0 + width / 2.0);
                public static final Translation2d oppFarRightCorner = new Translation2d(
                                oppTopCenterPoint.getX() + width / 2.0,
                                fieldWidth / 2.0 - width / 2.0);

                public static final Pose2d nearFace = kAprilTagLayout.getTagPose(26).get().toPose2d();
                public static final Pose2d farFace = kAprilTagLayout.getTagPose(20).get().toPose2d();
                public static final Pose2d rightFace = kAprilTagLayout.getTagPose(18).get().toPose2d();
                public static final Pose2d leftFace = kAprilTagLayout.getTagPose(21).get().toPose2d();
        }

        public static class LeftBump {
                public static final double width = Units.inchesToMeters(73.0);
                public static final double height = Units.inchesToMeters(6.513);
                public static final double depth = Units.inchesToMeters(44.4);

                public static final Translation2d nearLeftCorner = new Translation2d(
                                LinesVertical.hubCenter - width / 2,
                                Units.inchesToMeters(255));
                public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
                public static final Translation2d farLeftCorner = new Translation2d(LinesVertical.hubCenter + width / 2,
                                Units.inchesToMeters(255));
                public static final Translation2d farRightCorner = Hub.farLeftCorner;

                public static final Translation2d oppNearLeftCorner = new Translation2d(
                                LinesVertical.hubCenter - width / 2,
                                Units.inchesToMeters(255));
                public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
                public static final Translation2d oppFarLeftCorner = new Translation2d(
                                LinesVertical.hubCenter + width / 2,
                                Units.inchesToMeters(255));
                public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
        }

        public static class RightBump {
                public static final double width = Units.inchesToMeters(73.0);
                public static final double height = Units.inchesToMeters(6.513);
                public static final double depth = Units.inchesToMeters(44.4);

                public static final Translation2d nearLeftCorner = new Translation2d(
                                LinesVertical.hubCenter + width / 2,
                                Units.inchesToMeters(255));
                public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
                public static final Translation2d farLeftCorner = new Translation2d(LinesVertical.hubCenter - width / 2,
                                Units.inchesToMeters(255));
                public static final Translation2d farRightCorner = Hub.farLeftCorner;

                public static final Translation2d oppNearLeftCorner = new Translation2d(
                                LinesVertical.hubCenter + width / 2,
                                Units.inchesToMeters(255));
                public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
                public static final Translation2d oppFarLeftCorner = new Translation2d(
                                LinesVertical.hubCenter - width / 2,
                                Units.inchesToMeters(255));
                public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
        }

        public static class LeftTrench {
                public static final double width = Units.inchesToMeters(65.65);
                public static final double depth = Units.inchesToMeters(47.0);
                public static final double height = Units.inchesToMeters(40.25);
                public static final double openingWidth = Units.inchesToMeters(50.34);
                public static final double openingHeight = Units.inchesToMeters(22.25);

                public static final Translation3d openingTopLeft = new Translation3d(LinesVertical.hubCenter,
                                fieldWidth,
                                openingHeight);
                public static final Translation3d openingTopRight = new Translation3d(LinesVertical.hubCenter,
                                fieldWidth - openingWidth, openingHeight);

                public static final Translation3d oppOpeningTopLeft = new Translation3d(LinesVertical.oppHubCenter,
                                fieldWidth,
                                openingHeight);
                public static final Translation3d oppOpeningTopRight = new Translation3d(LinesVertical.oppHubCenter,
                                fieldWidth - openingWidth, openingHeight);
        }

        public static class RightTrench {
                public static final double width = Units.inchesToMeters(65.65);
                public static final double depth = Units.inchesToMeters(47.0);
                public static final double height = Units.inchesToMeters(40.25);
                public static final double openingWidth = Units.inchesToMeters(50.34);
                public static final double openingHeight = Units.inchesToMeters(22.25);

                public static final Translation3d openingTopLeft = new Translation3d(LinesVertical.hubCenter,
                                openingWidth,
                                openingHeight);
                public static final Translation3d openingTopRight = new Translation3d(LinesVertical.hubCenter, 0,
                                openingHeight);

                public static final Translation3d oppOpeningTopLeft = new Translation3d(LinesVertical.oppHubCenter,
                                openingWidth, openingHeight);
                public static final Translation3d oppOpeningTopRight = new Translation3d(LinesVertical.oppHubCenter, 0,
                                openingHeight);
        }

        public static class Tower {
                public static final double width = Units.inchesToMeters(49.25);
                public static final double depth = Units.inchesToMeters(45.0);
                public static final double height = Units.inchesToMeters(78.25);
                public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
                public static final double frontFaceX = Units.inchesToMeters(43.51);

                public static final double uprightHeight = Units.inchesToMeters(72.1);

                public static final double lowRungHeight = Units.inchesToMeters(27.0);
                public static final double midRungHeight = Units.inchesToMeters(45.0);
                public static final double highRungHeight = Units.inchesToMeters(63.0);

                public static final Translation2d centerPoint = new Translation2d(
                                frontFaceX, kAprilTagLayout.getTagPose(31).get().getY());
                public static final Translation2d leftUpright = new Translation2d(
                                frontFaceX,
                                (kAprilTagLayout.getTagPose(31).get().getY())
                                                + innerOpeningWidth / 2
                                                + Units.inchesToMeters(0.75));
                public static final Translation2d rightUpright = new Translation2d(
                                frontFaceX,
                                (kAprilTagLayout.getTagPose(31).get().getY())
                                                - innerOpeningWidth / 2
                                                - Units.inchesToMeters(0.75));

                public static final Translation2d oppCenterPoint = new Translation2d(
                                fieldLength - frontFaceX,
                                kAprilTagLayout.getTagPose(15).get().getY());
                public static final Translation2d oppLeftUpright = new Translation2d(
                                fieldLength - frontFaceX,
                                (kAprilTagLayout.getTagPose(15).get().getY())
                                                + innerOpeningWidth / 2
                                                + Units.inchesToMeters(0.75));
                public static final Translation2d oppRightUpright = new Translation2d(
                                fieldLength - frontFaceX,
                                (kAprilTagLayout.getTagPose(15).get().getY())
                                                - innerOpeningWidth / 2
                                                - Units.inchesToMeters(0.75));
        }

        public static class Depot {
                public static final double width = Units.inchesToMeters(42.0);
                public static final double depth = Units.inchesToMeters(27.0);
                public static final double height = Units.inchesToMeters(1.125);
                public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

                public static final Translation3d depotCenter = new Translation3d(depth,
                                (fieldWidth / 2) + distanceFromCenterY,
                                height);
                public static final Translation3d leftCorner = new Translation3d(depth,
                                (fieldWidth / 2) + distanceFromCenterY + (width / 2), height);
                public static final Translation3d rightCorner = new Translation3d(depth,
                                (fieldWidth / 2) + distanceFromCenterY - (width / 2), height);
        }

        public static class Outpost {
                public static final double width = Units.inchesToMeters(31.8);
                public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
                public static final double height = Units.inchesToMeters(7.0);

                public static final Translation2d centerPoint = new Translation2d(0,
                                kAprilTagLayout.getTagPose(29).get().getY());
        }

        public static class FieldConstants {
        public static final Distance FIELD_LENGTH = Inches.of(650.12);
        public static final Distance FIELD_WIDTH = Inches.of(316.64);

        public static final Distance ALLIANCE_ZONE = Inches.of(156.06);

        public static final Translation3d HUB_BLUE =
                new Translation3d(Inches.of(181.56), FIELD_WIDTH.div(2), Inches.of(56.4));
        public static final Translation3d HUB_RED =
                new Translation3d(FIELD_LENGTH.minus(Inches.of(181.56)), FIELD_WIDTH.div(2), Inches.of(56.4));
        public static final Distance FUNNEL_RADIUS = Inches.of(24);
        public static final Distance FUNNEL_HEIGHT = Inches.of(72 - 56.4);

        public static final Distance TRENCH_BUMP_X = Inches.of(181.56);
        public static final Distance TRENCH_WIDTH = Inches.of(49.86);
        private static final Distance BUMP_INSET = TRENCH_WIDTH.plus(Inches.of(12));
        private static final Distance BUMP_LENGTH = Inches.of(73);

        private static final Distance TRENCH_ZONE_EXTENSION = Inches.of(60);
        private static final Distance BUMP_ZONE_EXTENSION = Inches.of(60);
        private static final Distance TRENCH_BUMP_ZONE_TRANSITION =
                TRENCH_WIDTH.plus(BUMP_INSET).div(2);

        public static final Translation2d[][] TRENCH_ZONES = {
            new Translation2d[] {
                new Translation2d(TRENCH_BUMP_X.minus(TRENCH_ZONE_EXTENSION), Inches.zero()),
                new Translation2d(TRENCH_BUMP_X.plus(TRENCH_ZONE_EXTENSION), TRENCH_BUMP_ZONE_TRANSITION)
            },
            new Translation2d[] {
                new Translation2d(
                        TRENCH_BUMP_X.minus(TRENCH_ZONE_EXTENSION), FIELD_WIDTH.minus(TRENCH_BUMP_ZONE_TRANSITION)),
                new Translation2d(TRENCH_BUMP_X.plus(TRENCH_ZONE_EXTENSION), FIELD_WIDTH)
            },
            new Translation2d[] {
                new Translation2d(FIELD_LENGTH.minus(TRENCH_BUMP_X.plus(TRENCH_ZONE_EXTENSION)), Inches.zero()),
                new Translation2d(
                        FIELD_LENGTH.minus(TRENCH_BUMP_X.minus(TRENCH_ZONE_EXTENSION)), TRENCH_BUMP_ZONE_TRANSITION)
            },
            new Translation2d[] {
                new Translation2d(
                        FIELD_LENGTH.minus(TRENCH_BUMP_X.plus(TRENCH_ZONE_EXTENSION)),
                        FIELD_WIDTH.minus(TRENCH_BUMP_ZONE_TRANSITION)),
                new Translation2d(FIELD_LENGTH.minus(TRENCH_BUMP_X.minus(TRENCH_ZONE_EXTENSION)), FIELD_WIDTH)
            }
        };

        public static final Translation2d[][] BUMP_ZONES = {
            new Translation2d[] {
                new Translation2d(TRENCH_BUMP_X.minus(BUMP_ZONE_EXTENSION), TRENCH_BUMP_ZONE_TRANSITION),
                new Translation2d(TRENCH_BUMP_X.plus(BUMP_ZONE_EXTENSION), BUMP_INSET.plus(BUMP_LENGTH))
            },
            new Translation2d[] {
                new Translation2d(
                        TRENCH_BUMP_X.minus(BUMP_ZONE_EXTENSION), FIELD_WIDTH.minus(BUMP_INSET.plus(BUMP_LENGTH))),
                new Translation2d(
                        TRENCH_BUMP_X.plus(BUMP_ZONE_EXTENSION), FIELD_WIDTH.minus(TRENCH_BUMP_ZONE_TRANSITION))
            },
            new Translation2d[] {
                new Translation2d(
                        FIELD_LENGTH.minus(TRENCH_BUMP_X.plus(BUMP_ZONE_EXTENSION)),
                        FIELD_WIDTH.minus(BUMP_INSET.plus(BUMP_LENGTH))),
                new Translation2d(
                        FIELD_LENGTH.minus(TRENCH_BUMP_X.minus(BUMP_ZONE_EXTENSION)),
                        FIELD_WIDTH.minus(TRENCH_BUMP_ZONE_TRANSITION))
            },
            new Translation2d[] {
                new Translation2d(
                        FIELD_LENGTH.minus(TRENCH_BUMP_X.plus(BUMP_ZONE_EXTENSION)), TRENCH_BUMP_ZONE_TRANSITION),
                new Translation2d(
                        FIELD_LENGTH.minus(TRENCH_BUMP_X.minus(BUMP_ZONE_EXTENSION)), BUMP_INSET.plus(BUMP_LENGTH))
            }
        };

        public static final Distance TRENCH_CENTER = TRENCH_WIDTH.div(2);
    }
}
