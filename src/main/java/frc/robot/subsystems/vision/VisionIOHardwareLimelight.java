package frc.robot.subsystems.vision;

import java.util.concurrent.atomic.AtomicReference;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotState;
import frc.robot.util.LimelightHelpers;

public class VisionIOHardwareLimelight implements VisionIO {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightTableName);
    NetworkTable tableB = NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightBTableName);
    
    RobotState robotState;
    AtomicReference<VisionIOInputs> latestInputs = new AtomicReference<>(new VisionIOInputs());

    public VisionIOHardwareLimelight(RobotState robotState) {
        this.robotState = robotState;
        setLLSettings();
    }

    private void setLLSettings() {
        double[] camerapose = { 0.0, 0.0, VisionConstants.kCameraHeightOffGroundMeters, 0.0, VisionConstants.kCameraPitchDegrees,
                0.0 };
        table.getEntry("camerapose_robotspace_set").setDoubleArray(camerapose);

        double[] cameraBpose = { 0.0, 0.0, VisionConstants.kCameraBHeightOffGroundMeters, VisionConstants.kCameraBRollDegrees,
                VisionConstants.kCameraBPitchDegrees, 0.0 };
        tableB.getEntry("camerapose_robotspace_set").setDoubleArray(cameraBpose);

        var fieldToTurretRotation = robotState.getLatestFieldToRobot().getValue().getRotation()
                .rotateBy(robotState.getLatestRobotToTurret().getValue());
        var fieldToTurretVelocity = Units.radiansToDegrees(robotState.getLatestTurretAngularVelocity()
                + robotState.getLatestRobotRelativeChassisSpeed().omegaRadiansPerSecond);
        LimelightHelpers.SetRobotOrientation(VisionConstants.kLimelightTableName, fieldToTurretRotation.getDegrees(),
                fieldToTurretVelocity, 0, 0, 0, 0);

        var gyroAngle = robotState.getLatestFieldToRobot().getValue().getRotation();
        var gyroAngularVelocity = Units
                .radiansToDegrees(robotState.getLatestRobotRelativeChassisSpeed().omegaRadiansPerSecond);
        LimelightHelpers.SetRobotOrientation(VisionConstants.kLimelightBTableName, gyroAngle.getDegrees(),
                gyroAngularVelocity, 0, 0, 0, 0);
    }

    @Override
    public void readInputs(VisionIOInputs inputs) {
        inputs.turretCameraSeesTarget = table.getEntry("tv").getDouble(0) == 1.0;
        inputs.elevatorCameraSeesTarget = tableB.getEntry("tv").getDouble(0) == 1.0;
        if (inputs.turretCameraSeesTarget) {
            // Read megatag updates
            var megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.kLimelightTableName);
            var megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.kLimelightTableName);
            inputs.turretCameraMegatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
            inputs.turretCameraMegatagCount = megatag.tagCount;
            inputs.turretCameraMegatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag2);
            inputs.turretCameraFiducialObservations = FiducialObservation.fromLimelight(megatag.rawFiducials);
        }
        if (inputs.elevatorCameraSeesTarget) {
            // Read megatag updates
            var megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.kLimelightBTableName);
            var megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.kLimelightBTableName);
            inputs.elevatorCameraMegatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
            inputs.elevatorCameraMegatagCount = megatag.tagCount;
            inputs.elevatorCameraMegatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag2);
            inputs.elevatorCameraFiducialObservations = FiducialObservation.fromLimelight(megatag.rawFiducials);
        }
        latestInputs.set(inputs);

        // Set the persistent settings into NT
        setLLSettings();
    }

    @Override
    public void pollNetworkTables() {
        VisionIOInputs inputs = new VisionIOInputs();

        // See if we see the target
        inputs.turretCameraSeesTarget = table.getEntry("tv").getDouble(0) == 1.0;
        inputs.elevatorCameraSeesTarget = tableB.getEntry("tv").getDouble(0) == 1.0;
        if (inputs.turretCameraSeesTarget) {
            // Read megatag updates
            var megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.kLimelightTableName);
            var megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.kLimelightTableName);
            inputs.turretCameraMegatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
            inputs.turretCameraMegatagCount = megatag.tagCount;
            inputs.turretCameraMegatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag2);
            inputs.turretCameraFiducialObservations = FiducialObservation.fromLimelight(megatag.rawFiducials);
        }
        if (inputs.elevatorCameraSeesTarget) {
            // Read megatag updates
            var megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.kLimelightBTableName);
            var megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.kLimelightBTableName);
            inputs.elevatorCameraMegatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
            inputs.elevatorCameraMegatagCount = megatag.tagCount;
            inputs.elevatorCameraMegatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag2);
            inputs.elevatorCameraFiducialObservations = FiducialObservation.fromLimelight(megatag.rawFiducials);
        }
        latestInputs.set(inputs);

        // Set the persistent settings into NT
        setLLSettings();
    }
}