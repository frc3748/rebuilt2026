package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;
import frc.robot.util.LimelightHelpers;

public class VisionIOHardwareLimelight implements VisionIO {

    NetworkTable tableA = NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightTableName);
    NetworkTable tableB = NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightBTableName);
    RobotState robotState;
    int imuMode = 1;

    private static final double[] DEFAULT_STDDEVS = new double[VisionConstants.kExpectedStdDevArrayLength];

    /** Creates a new Limelight vision IO instance. */
    public VisionIOHardwareLimelight(RobotState robotState) {
        this.robotState = robotState;
        setLLSettings();
    }

    /** Configures Limelight camera poses in robot coordinate system. */
    private void setLLSettings() {
        double[] cameraAPose = {
                0,
                0,
                VisionConstants.kCameraHeightOffGroundMeters,
                0.0,
                VisionConstants.kCameraPitchDegrees,
                0
        };

        tableA.getEntry("camerapose_robotspace_set").setDoubleArray(cameraAPose);

        double[] cameraBPose = {
                VisionConstants.kCameraBForwardMeters,
                VisionConstants.kCameraBRightMeters,
                VisionConstants.kCameraBHeightOffGroundMeters,
                0.0,
                VisionConstants.kCameraBPitchDegrees,
                VisionConstants.kCameraBYawDegrees
        };

        // tableB.getEntry("camerapose_robotspace_set").setDoubleArray(cameraBPose);

    }

    @Override
    public void readInputs(CameraInputsAutoLogged turretCamera, CameraInputsAutoLogged chassisCamera) {
        readCameraData(tableA, turretCamera, VisionConstants.kLimelightTableName);
        readCameraData(tableB, chassisCamera, VisionConstants.kLimelightBTableName);

        LimelightHelpers.SetIMUMode(VisionConstants.kLimelightTableName,
                DriverStation.isEnabled() ? 4 : 1);
        LimelightHelpers.SetIMUMode(VisionConstants.kLimelightBTableName,
                DriverStation.isEnabled() ? 4 : 1);

        var gyroAngle = robotState.getLatestFieldToRobot().getValue().getRotation();
        var gyroAngularVelocity = Units.radiansToDegrees(robotState.getLatestRobotRelativeChassisSpeed().omegaRadiansPerSecond);
        LimelightHelpers.SetRobotOrientation(VisionConstants.kLimelightBTableName, gyroAngle.getDegrees(),
                gyroAngularVelocity, 0, 0, 0, 0);
        
        Logger.processInputs("Vision/Turret Camera", turretCamera);
        Logger.processInputs("Vision/Chassis Camera", chassisCamera);
    }

    private void readCameraData(
            NetworkTable table, VisionIO.CameraInputs camera, String limelightName) {
        camera.seesTarget = table.getEntry("tv").getDouble(0) == 1.0;
        if (camera.seesTarget) {
            try {
                var megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
                var robotPose3d = LimelightHelpers.toPose3D(
                        LimelightHelpers.getBotPose(limelightName));

                if (megatag != null) {
                    camera.megatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
                    camera.megatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
                    camera.megatag2Count = megatag.tagCount;
                    camera.fiducialObservations = FiducialObservation.fromLimelight(megatag.rawFiducials);
                }
                if (robotPose3d != null) {
                    camera.pose3d = robotPose3d;
                }

                camera.standardDeviations =
                table.getEntry("stddevs").getDoubleArray(DEFAULT_STDDEVS);
            } catch (Exception e) {
                System.err.println("Error processing Limelight data: " + e.getMessage());
            }
        }
    }

}
