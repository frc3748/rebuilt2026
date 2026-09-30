package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.game.FieldConstants;

public class CameraIOPhoton implements CameraIO {
    protected final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;

    public CameraIOPhoton(CameraConfig config) {
        camera = new PhotonCamera(config.networkName());
        estimator = new PhotonPoseEstimator(FieldConstants.TAG_LAYOUT, config.robotToCamera());
    }

    @Override
    public void setRobotOrientation(Rotation2d heading, double yawRateDegreesPerSecond) {
        estimator.addHeadingData(Timer.getFPGATimestamp(), heading);
    }

    @Override
    public void setRobotToCamera(Transform3d robotToCamera) {
        estimator.setRobotToCameraTransform(robotToCamera);
    }

    @Override
    public void setPipeline(int index) {
        camera.setPipelineIndex(index);
    }

    @Override
    public void updateInputs(CameraInputs inputs) {
        inputs.connected = camera.isConnected();
        inputs.pipeline = camera.getPipelineIndex();

        List<PoseObservation> poses = new ArrayList<>();
        List<ObjectObservation> objects = new ArrayList<>();
        List<Integer> tagIds = new ArrayList<>();

        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            double timestamp = result.getTimestampSeconds();
            List<PhotonTrackedTarget> tags = new ArrayList<>();
            for (PhotonTrackedTarget target : result.getTargets()) {
                if (target.getFiducialId() >= 0) {
                    tags.add(target);
                    tagIds.add(target.getFiducialId());
                } else {
                    objects.add(new ObjectObservation(
                            timestamp,
                            Math.max(target.getDetectedObjectClassID(), 0),
                            target.getYaw(),
                            target.getPitch(),
                            target.getArea(),
                            target.getDetectedObjectConfidence() < 0 ? 1.0 : target.getDetectedObjectConfidence()));
                }
            }
            if (tags.isEmpty()) {
                continue;
            }

            double averageDistance = tags.stream()
                    .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
                    .average()
                    .orElse(0.0);
            double ambiguity = tags.size() == 1 ? tags.get(0).getPoseAmbiguity() : 0.0;

            Optional<EstimatedRobotPose> multiTag = estimator.estimateCoprocMultiTagPose(result);
            if (multiTag.isPresent()) {
                poses.add(toObservation(multiTag.get(), ambiguity, averageDistance, PoseSource.MULTI_TAG));
            } else {
                estimator.estimateLowestAmbiguityPose(result).ifPresent(pose ->
                        poses.add(toObservation(pose, ambiguity, averageDistance, PoseSource.SINGLE_TAG)));
            }
            estimator.estimatePnpDistanceTrigSolvePose(result).ifPresent(pose ->
                    poses.add(toObservation(pose, ambiguity, averageDistance, PoseSource.TRIG_SOLVE)));
        }

        inputs.poseObservations = poses.toArray(PoseObservation[]::new);
        inputs.objectObservations = objects.toArray(ObjectObservation[]::new);
        inputs.tagIds = tagIds.stream().distinct().mapToInt(Integer::intValue).toArray();
    }

    private static PoseObservation toObservation(EstimatedRobotPose pose, double ambiguity, double averageDistance,
            PoseSource source) {
        return new PoseObservation(
                pose.timestampSeconds,
                pose.estimatedPose,
                ambiguity,
                pose.targetsUsed.size(),
                averageDistance,
                source);
    }
}
