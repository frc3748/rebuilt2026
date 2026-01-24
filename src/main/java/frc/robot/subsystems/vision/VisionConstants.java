package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    
    public static final String kLimelightTableName = "limelight-turret";
    public static final String kLimelightBTableName = "limelight";

     // Limelight constants
    // TURRET LIMELIGHT
    // Pitch angle: How many radians the camera is pitched up around Y axis. 0 is
    // looking straight ahead, +is nodding up.
    public static final double kCameraPitchDegrees = 20.5;
    public static final double kCameraPitchRads = Units.degreesToRadians(kCameraPitchDegrees);
    public static final double kCameraHeightOffGroundMeters = Units.inchesToMeters(4.181);
    public static final double kImageCaptureLatency = 11.0; // milliseconds
    public static final double kLimelightTransmissionTimeLatency = 0.0; // milliseconds
    // Distance from turret center to camera le#ns in X axis (straight into lens)
    public static final double kTurretToCameraX = Units.inchesToMeters(7.834);
    // Distance from turret center to camera lens in Y
    public static final double kTurretToCameraY = 0;

    // ELEVATOR LIMELIGHT
    public static final double kCameraBPitchDegrees = 20.0;
    public static final double kCameraBPitchRads = Units.degreesToRadians(kCameraBPitchDegrees);
    public static final double kCameraBRollDegrees = 0.0;
    public static final double kCameraBRollRads = Units.degreesToRadians(kCameraBRollDegrees);
    public static final double kCameraBYawDegrees = 90; // this is added to robot orientation
    public static final double kCameraBYawRads = Units.degreesToRadians(kCameraBYawDegrees);
    public static final double kCameraBRightMeters = -0.25;
    public static final double kCameraBHeightOffGroundMeters = 0.2; // verify for practice
   
   
    // Distance from turret center to camera lens in X axis (straight into lens)
    public static final double kTurretToCameraBX = Units.inchesToMeters(14.882); // verify for practice
    // Distance from turret center to camera lens in Y
    public static final double kTurretToCameraBY = 0;

    public static final double kTurretToRobotCenterX = Units.inchesToMeters(2.3115);
    public static final double kTurretToRobotCenterY = 0;
    public static final Transform2d kTurretToRobotCenter = new Transform2d(
            new Translation2d(VisionConstants.kTurretToRobotCenterX, VisionConstants.kTurretToRobotCenterY),
            new Rotation2d());
    public static final Rotation2d kCameraYawOffset = new Rotation2d(0);

    // April Tag Layout
    public static final AprilTagFieldLayout kAprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public static final double kFieldWidthMeters = kAprilTagLayout.getFieldWidth(); // distance between field walls,
                                                                                    // 8.211m
    public static final double kFieldLengthMeters = kAprilTagLayout.getFieldLength(); // distance between driver station

    public static final Translation3d kBluePassArea = new Translation3d();

    public static final Translation3d kBlueHubPose = new Translation3d();
    public static final Translation3d kRedHubPose = new Translation3d();
}
