package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {

        public static final String kLimelightTableName = "limelight-turret";
        public static final String kLimelightBTableName = "limelight";

        public static final int kExpectedStdDevArrayLength = 12;
        // Large variance used to downweight unreliable vision measurements
        public static final double kLargeVariance = 1e6;

        // Limelight constants
        // TURRET LIMELIGHT
        // Pitch angle: How many radians the camera is pitched up around Y axis. 0 is
        // looking straight ahead, +is nodding up.
        public static final double kCameraPitchDegrees = 20.5;
        public static final double kCameraHeightOffGroundMeters = Units.inchesToMeters(4.181);

        // // Distance from turret center to camera le#ns in X axis (straight into lens)
        public static final double kTurretToCameraX = Units.inchesToMeters(7.834);
        // // Distance from turret center to camera lens in Y
        public static final double kTurretToCameraY = 0;

        // // CHASSIS LIMELIGHT
        public static final double kCameraBPitchDegrees = 20.0;
        public static final double kCameraBYawDegrees = 90; // this is added to robot orientation
        public static final double kCameraBForwardMeters = 0;
        public static final double kCameraBRightMeters = -0.25;
        public static final double kCameraBHeightOffGroundMeters = 0.2; // verify for practice

        public static final double kTurretToCameraBX = Units.inchesToMeters(14.882); // verify for practice
        // Distance from turret center to camera lens in Y
        public static final double kTurretToCameraBY = 0;

        // THIS IS SOMETHING WE NEED TO DO
        public static final double kTurretToRobotCenterX = Units.inchesToMeters(2.3115);
        public static final double kTurretToRobotCenterY = 0;
        public static final Transform2d kTurretToRobotCenter = new Transform2d(
                        new Translation2d(VisionConstants.kTurretToRobotCenterX, VisionConstants.kTurretToRobotCenterY),
                        new Rotation2d());

        public static final Transform2d kTurretCameraToRobotCenter = new Transform2d();
        public static final Transform2d kBCameraToRobotCenter = new Transform2d();

        // April Tag Layout
        public static final AprilTagFieldLayout kAprilTagLayout = AprilTagFieldLayout
                        .loadField(AprilTagFields.k2026RebuiltAndymark);

        public static final double kFieldWidthMeters = kAprilTagLayout.getFieldWidth(); // distance between field walls,
                                                                                        // 8.211m
        public static final double kFieldLengthMeters = kAprilTagLayout.getFieldLength(); // distance between driver
                                                                                          // station

        public static final Translation3d kBluePassArea = new Translation3d();

        public static final Translation3d kBlueHubPose = new Translation3d();
        public static final Translation3d kRedHubPose = new Translation3d();

        public static final int aprilTagCount = kAprilTagLayout.getTags().size();
        public static final double aprilTagWidth = Units.inchesToMeters(6.5);

        // Field dimensions
        public static final double fieldLength = kAprilTagLayout.getFieldLength();
        public static final double fieldWidth = kAprilTagLayout.getFieldWidth();

        /**
         * Officially defined and relevant vertical lines found on the field (defined by
         * X-axis offset)
         */
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

        /**
         * Officially defined and relevant horizontal lines found on the field (defined
         * by Y-axis offset)
         *
         * <p>
         * NOTE: The field element start and end are always left to right from the
         * perspective of the
         * alliance station
         */
        public static class LinesHorizontal {

                public static final double center = fieldWidth / 2.0;

                // Right of hub
                public static final double rightBumpStart = Hub.nearRightCorner.getY();
                public static final double rightBumpEnd = rightBumpStart - RightBump.width;
                public static final double rightTrenchOpenStart = rightBumpEnd - Units.inchesToMeters(12.0);
                public static final double rightTrenchOpenEnd = 0;

                // Left of hub
                public static final double leftBumpEnd = Hub.nearLeftCorner.getY();
                public static final double leftBumpStart = leftBumpEnd + LeftBump.width;
                public static final double leftTrenchOpenEnd = leftBumpStart + Units.inchesToMeters(12.0);
                public static final double leftTrenchOpenStart = fieldWidth;
        }

        /** Hub related constants */
        public static class Hub {

                // Dimensions
                public static final double width = Units.inchesToMeters(47.0);
                public static final double height = Units.inchesToMeters(72.0); // includes the catcher at the top
                public static final double innerWidth = Units.inchesToMeters(41.7);
                public static final double innerHeight = Units.inchesToMeters(56.5);

                // Relevant reference points on alliance side
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

                // Relevant reference points on the opposite side
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

                // Hub faces
                public static final Pose2d nearFace = kAprilTagLayout.getTagPose(26).get().toPose2d();
                public static final Pose2d farFace = kAprilTagLayout.getTagPose(20).get().toPose2d();
                public static final Pose2d rightFace = kAprilTagLayout.getTagPose(18).get().toPose2d();
                public static final Pose2d leftFace = kAprilTagLayout.getTagPose(21).get().toPose2d();
        }

        /** Left Bump related constants */
        public static class LeftBump {

                // Dimensions
                public static final double width = Units.inchesToMeters(73.0);
                public static final double height = Units.inchesToMeters(6.513);
                public static final double depth = Units.inchesToMeters(44.4);

                // Relevant reference points on alliance side
                public static final Translation2d nearLeftCorner = new Translation2d(
                                LinesVertical.hubCenter - width / 2,
                                Units.inchesToMeters(255));
                public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
                public static final Translation2d farLeftCorner = new Translation2d(LinesVertical.hubCenter + width / 2,
                                Units.inchesToMeters(255));
                public static final Translation2d farRightCorner = Hub.farLeftCorner;

                // Relevant reference points on opposing side
                public static final Translation2d oppNearLeftCorner = new Translation2d(
                                LinesVertical.hubCenter - width / 2,
                                Units.inchesToMeters(255));
                public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
                public static final Translation2d oppFarLeftCorner = new Translation2d(
                                LinesVertical.hubCenter + width / 2,
                                Units.inchesToMeters(255));
                public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
        }

        /** Right Bump related constants */
        public static class RightBump {
                // Dimensions
                public static final double width = Units.inchesToMeters(73.0);
                public static final double height = Units.inchesToMeters(6.513);
                public static final double depth = Units.inchesToMeters(44.4);

                // Relevant reference points on alliance side
                public static final Translation2d nearLeftCorner = new Translation2d(
                                LinesVertical.hubCenter + width / 2,
                                Units.inchesToMeters(255));
                public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
                public static final Translation2d farLeftCorner = new Translation2d(LinesVertical.hubCenter - width / 2,
                                Units.inchesToMeters(255));
                public static final Translation2d farRightCorner = Hub.farLeftCorner;

                // Relevant reference points on opposing side
                public static final Translation2d oppNearLeftCorner = new Translation2d(
                                LinesVertical.hubCenter + width / 2,
                                Units.inchesToMeters(255));
                public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
                public static final Translation2d oppFarLeftCorner = new Translation2d(
                                LinesVertical.hubCenter - width / 2,
                                Units.inchesToMeters(255));
                public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
        }

        /** Left Trench related constants */
        public static class LeftTrench {
                // Dimensions
                public static final double width = Units.inchesToMeters(65.65);
                public static final double depth = Units.inchesToMeters(47.0);
                public static final double height = Units.inchesToMeters(40.25);
                public static final double openingWidth = Units.inchesToMeters(50.34);
                public static final double openingHeight = Units.inchesToMeters(22.25);

                // Relevant reference points on alliance side
                public static final Translation3d openingTopLeft = new Translation3d(LinesVertical.hubCenter,
                                fieldWidth,
                                openingHeight);
                public static final Translation3d openingTopRight = new Translation3d(LinesVertical.hubCenter,
                                fieldWidth - openingWidth, openingHeight);

                // Relevant reference points on opposing side
                public static final Translation3d oppOpeningTopLeft = new Translation3d(LinesVertical.oppHubCenter,
                                fieldWidth,
                                openingHeight);
                public static final Translation3d oppOpeningTopRight = new Translation3d(LinesVertical.oppHubCenter,
                                fieldWidth - openingWidth, openingHeight);
        }

        public static class RightTrench {

                // Dimensions
                public static final double width = Units.inchesToMeters(65.65);
                public static final double depth = Units.inchesToMeters(47.0);
                public static final double height = Units.inchesToMeters(40.25);
                public static final double openingWidth = Units.inchesToMeters(50.34);
                public static final double openingHeight = Units.inchesToMeters(22.25);

                // Relevant reference points on alliance side
                public static final Translation3d openingTopLeft = new Translation3d(LinesVertical.hubCenter,
                                openingWidth,
                                openingHeight);
                public static final Translation3d openingTopRight = new Translation3d(LinesVertical.hubCenter, 0,
                                openingHeight);

                // Relevant reference points on opposing side
                public static final Translation3d oppOpeningTopLeft = new Translation3d(LinesVertical.oppHubCenter,
                                openingWidth, openingHeight);
                public static final Translation3d oppOpeningTopRight = new Translation3d(LinesVertical.oppHubCenter, 0,
                                openingHeight);
        }

        /** Tower related constants */
        public static class Tower {
                // Dimensions
                public static final double width = Units.inchesToMeters(49.25);
                public static final double depth = Units.inchesToMeters(45.0);
                public static final double height = Units.inchesToMeters(78.25);
                public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
                public static final double frontFaceX = Units.inchesToMeters(43.51);

                public static final double uprightHeight = Units.inchesToMeters(72.1);

                // Rung heights from the floor
                public static final double lowRungHeight = Units.inchesToMeters(27.0);
                public static final double midRungHeight = Units.inchesToMeters(45.0);
                public static final double highRungHeight = Units.inchesToMeters(63.0);

                // Relevant reference points on alliance side
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

                // Relevant reference points on opposing side
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
                // Dimensions
                public static final double width = Units.inchesToMeters(42.0);
                public static final double depth = Units.inchesToMeters(27.0);
                public static final double height = Units.inchesToMeters(1.125);
                public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

                // Relevant reference points on alliance side
                public static final Translation3d depotCenter = new Translation3d(depth,
                                (fieldWidth / 2) + distanceFromCenterY,
                                height);
                public static final Translation3d leftCorner = new Translation3d(depth,
                                (fieldWidth / 2) + distanceFromCenterY + (width / 2), height);
                public static final Translation3d rightCorner = new Translation3d(depth,
                                (fieldWidth / 2) + distanceFromCenterY - (width / 2), height);
        }

        public static class Outpost {
                // Dimensions
                public static final double width = Units.inchesToMeters(31.8);
                public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
                public static final double height = Units.inchesToMeters(7.0);

                // Relevant reference points on alliance side
                public static final Translation2d centerPoint = new Translation2d(0,
                                kAprilTagLayout.getTagPose(29).get().getY());
        }
}
