package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class CameraIOPhoton implements CameraIO {
    protected final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;

    public CameraIOPhoton(CameraConfig config) {
        camera = new PhotonCamera(config.networkName());
        estimator = new PhotonPoseEstimator(VisionConstants.kAprilTagLayout, config.robotToCamera());
    }

    @Override
    public void setRobotOrientation(Rotation2d fieldToRobot, double yawRateDegreesPerSecond) {
        estimator.addHeadingData(Timer.getFPGATimestamp(), fieldToRobot);
    }

    @Override
    public void updateInputs(CameraInputs inputs) {
        inputs.connected = camera.isConnected();

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.isEmpty()) {
            return;
        }
        PhotonPipelineResult result = results.get(results.size() - 1);

        inputs.seesTarget = result.hasTargets();
        inputs.megatagCount = 0;
        inputs.megatag2Count = 0;
        inputs.fiducialObservations = result.getTargets().stream()
                .map(target -> new FiducialObservation(
                        target.getFiducialId(),
                        target.getYaw(),
                        target.getPitch(),
                        target.getPoseAmbiguity(),
                        target.getArea()))
                .toArray(FiducialObservation[]::new);
        if (!inputs.seesTarget) {
            return;
        }

        double avgDist = result.getTargets().stream()
                .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
                .average()
                .orElse(0.0);
        double avgArea = result.getTargets().stream()
                .mapToDouble(PhotonTrackedTarget::getArea)
                .average()
                .orElse(0.0);

        estimator.estimateCoprocMultiTagPose(result)
                .or(() -> estimator.estimateLowestAmbiguityPose(result))
                .ifPresent(pose -> {
                    inputs.megatagPoseEstimate = toEstimate(pose, avgArea);
                    inputs.megatagCount = pose.targetsUsed.size();
                    inputs.megatagAvgDist = avgDist;
                    inputs.fieldToRobot3d = pose.estimatedPose;
                });

        estimator.estimatePnpDistanceTrigSolvePose(result).ifPresent(pose -> {
            inputs.megatag2PoseEstimate = toEstimate(pose, avgArea);
            inputs.megatag2Count = pose.targetsUsed.size();
            inputs.megatag2AvgDist = avgDist;
        });
    }

    private static MegatagPoseEstimate toEstimate(EstimatedRobotPose pose, double avgArea) {
        int[] ids = pose.targetsUsed.stream().mapToInt(PhotonTrackedTarget::getFiducialId).toArray();
        return new MegatagPoseEstimate(
                pose.estimatedPose.toPose2d(),
                pose.timestampSeconds,
                Timer.getFPGATimestamp() - pose.timestampSeconds,
                avgArea,
                ids.length,
                ids);
    }
}
