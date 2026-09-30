package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDouble;
import frc.robot.game.FieldConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawDetection;

public class CameraIOLimelight implements CameraIO {
    private static final String kColorPipeline = "pipe_color";

    private final String name;
    private final boolean hasImu;
    private final NetworkTable table;
    private final DoubleSubscriber heartbeat;
    private double lastFrame = Double.NaN;

    public CameraIOLimelight(CameraConfig config, boolean hasImu) {
        name = config.networkName();
        this.hasImu = hasImu;
        table = NetworkTableInstance.getDefault().getTable(name);
        heartbeat = table.getDoubleTopic("hb").subscribe(Double.NaN);
    }

    @Override
    public void setRobotOrientation(Rotation2d heading, double yawRateDegreesPerSecond) {
        LimelightHelpers.SetRobotOrientation(name, heading.getDegrees(), yawRateDegreesPerSecond, 0, 0, 0, 0);
    }

    @Override
    public void setPipeline(int index) {
        LimelightHelpers.setPipelineIndex(name, index);
    }

    @Override
    public void updateInputs(CameraInputs inputs) {
        if (hasImu) {
            LimelightHelpers.SetIMUMode(name, 1);
            LimelightHelpers.SetIMUAssistAlpha(name, 0.01);
        }
        LimelightHelpers.SetFiducialIDFiltersOverride(name, FieldConstants.TAG_IDS);

        inputs.connected = table.containsKey("tv");
        inputs.pipeline = (int) LimelightHelpers.getCurrentPipelineIndex(name);

        List<PoseObservation> poses = new ArrayList<>();
        boolean seesTag = table.getEntry("tv").getDouble(0) == 1.0;
        PoseEstimate megatag1 = seesTag ? LimelightHelpers.getBotPoseEstimate_wpiBlue(name) : null;
        PoseEstimate megatag2 = seesTag ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name) : null;
        addObservation(poses, megatag1, PoseSource.MEGATAG_1);
        addObservation(poses, megatag2, PoseSource.MEGATAG_2);
        inputs.poseObservations = poses.toArray(PoseObservation[]::new);
        inputs.tagIds = isValid(megatag1)
                ? Arrays.stream(megatag1.rawFiducials).mapToInt(fiducial -> fiducial.id).toArray()
                : new int[0];

        inputs.objectObservations = readObjects(seesTag);
    }

    private ObjectObservation[] readObjects(boolean hasTarget) {
        TimestampedDouble frame = heartbeat.getAtomic();
        if (Double.isNaN(frame.value) || frame.value == lastFrame) {
            return new ObjectObservation[0];
        }
        lastFrame = frame.value;
        double latencySeconds = (LimelightHelpers.getLatency_Pipeline(name) + LimelightHelpers.getLatency_Capture(name)) / 1000.0;
        double timestamp = frame.timestamp / 1e6 - latencySeconds;

        RawDetection[] detections = LimelightHelpers.getRawDetections(name);
        if (detections.length > 0) {
            return Arrays.stream(detections)
                    .map(detection -> toObject(detection, timestamp))
                    .toArray(ObjectObservation[]::new);
        }
        if (hasTarget && kColorPipeline.equals(LimelightHelpers.getCurrentPipelineType(name))) {
            return new ObjectObservation[] {new ObjectObservation(timestamp, 0, LimelightHelpers.getTX(name),
                    LimelightHelpers.getTY(name), LimelightHelpers.getTA(name), 1.0)};
        }
        return new ObjectObservation[0];
    }

    private static void addObservation(List<PoseObservation> poses, PoseEstimate estimate, PoseSource source) {
        if (!isValid(estimate)) {
            return;
        }
        double ambiguity = Arrays.stream(estimate.rawFiducials).mapToDouble(fiducial -> fiducial.ambiguity).max().orElse(0.0);
        poses.add(new PoseObservation(
                estimate.timestampSeconds,
                new Pose3d(estimate.pose),
                ambiguity,
                estimate.tagCount,
                estimate.avgTagDist,
                source));
    }

    private static boolean isValid(PoseEstimate estimate) {
        return estimate != null
                && estimate.pose != null
                && estimate.tagCount > 0
                && !estimate.pose.getTranslation().equals(VisionConstants.kLimelightErrorPose.getTranslation());
    }

    private static ObjectObservation toObject(RawDetection detection, double timestamp) {
        return new ObjectObservation(timestamp, detection.classId, detection.txnc, detection.tync, detection.ta, 1.0);
    }
}
