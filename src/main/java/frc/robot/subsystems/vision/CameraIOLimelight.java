package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class CameraIOLimelight implements CameraIO {
    private final String name;
    private final NetworkTable table;

    public CameraIOLimelight(CameraConfig config) {
        name = config.networkName();
        table = NetworkTableInstance.getDefault().getTable(name);
    }

    private void configure() {
        LimelightHelpers.SetIMUMode(name, 1);
        LimelightHelpers.SetIMUAssistAlpha(name, 0.01);
        LimelightHelpers.SetFiducialIDFiltersOverride(name, VisionConstants.kValidTagIds);
    }

    @Override
    public void setRobotOrientation(Rotation2d fieldToRobot, double yawRateDegreesPerSecond) {
        LimelightHelpers.SetRobotOrientation(name, fieldToRobot.getDegrees(), yawRateDegreesPerSecond, 0, 0, 0, 0);
    }

    @Override
    public void updateInputs(CameraInputs inputs) {
        configure();

        inputs.connected = table.containsKey("tv");
        inputs.seesTarget = table.getEntry("tv").getDouble(0) == 1.0;
        inputs.megatagCount = 0;
        inputs.megatag2Count = 0;
        if (!inputs.seesTarget) {
            return;
        }

        try {
            PoseEstimate megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
            PoseEstimate megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

            if (isValid(megatag)) {
                inputs.megatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
                inputs.megatagCount = megatag.tagCount;
                inputs.megatagAvgDist = megatag.avgTagDist;
                inputs.fiducialObservations = FiducialObservation.fromLimelight(megatag.rawFiducials);
            }
            if (isValid(megatag2)) {
                inputs.megatag2PoseEstimate = MegatagPoseEstimate.fromLimelight(megatag2);
                inputs.megatag2Count = megatag2.tagCount;
                inputs.megatag2AvgDist = megatag2.avgTagDist;
                inputs.fiducialObservations = FiducialObservation.fromLimelight(megatag2.rawFiducials);
            }

            inputs.fieldToRobot3d = LimelightHelpers.getBotPose3d_wpiBlue(name);
        } catch (Exception e) {
            DriverStation.reportError("Limelight " + name + ": " + e.getMessage(), false);
        }
    }

    private static boolean isValid(PoseEstimate estimate) {
        return estimate != null
                && estimate.pose != null
                && !estimate.pose.getTranslation().equals(VisionConstants.kErrorPoseRed.getTranslation());
    }
}
