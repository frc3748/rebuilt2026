package frc.robot.subsystems.vision;

import java.util.Arrays;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotState;

public class Camera {
    private final CameraConfig config;
    private final CameraIO io;
    private final CameraInputsAutoLogged inputs = new CameraInputsAutoLogged();
    private final String logKey;
    private double lastTimestamp;

    public Camera(CameraConfig config, CameraIO io) {
        this.config = config;
        this.io = io;
        logKey = "Vision/" + config.name();
    }

    public static Camera of(CameraConfig config, RobotState state) {
        CameraIO io = switch (Constants.kMode) {
            case REAL -> switch (config.type()) {
                case LIMELIGHT -> new CameraIOLimelight(config);
                case PHOTON -> new CameraIOPhoton(config);
            };
            case SIM -> new CameraIOPhotonSim(config, () -> state.getSimRobot().getLatestFieldToRobot());
            case REPLAY -> new CameraIO() {};
        };
        return new Camera(config, io);
    }

    public Optional<VisionFieldPoseEstimate> update(RobotState state) {
        Pose2d fieldToRobot = state.getLatestFieldToRobot().getValue();
        io.setRobotOrientation(
                fieldToRobot.getRotation(),
                Units.radiansToDegrees(state.getLatestRobotRelativeChassisSpeed().omegaRadiansPerSecond));
        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);

        Logger.recordOutput(logKey + "/CameraPose", new Pose3d(fieldToRobot).plus(config.robotToCamera()));
        Logger.recordOutput(logKey + "/Targets", targetPoses());

        return config.usedForPoseEstimation() ? process(state) : Optional.empty();
    }

    private Optional<VisionFieldPoseEstimate> process(RobotState state) {
        if (!inputs.seesTarget || inputs.fiducialObservations.length == 0) {
            return Optional.empty();
        }

        boolean useMegatag2 = inputs.megatag2Count > 0;
        if (!useMegatag2 && inputs.megatagCount <= 0) {
            return Optional.empty();
        }

        MegatagPoseEstimate estimate = useMegatag2 ? inputs.megatag2PoseEstimate : inputs.megatagPoseEstimate;
        if (estimate.fieldToRobot().equals(Pose2d.kZero)) {
            return Optional.empty();
        }

        double timestamp = estimate.timestampSeconds();
        if (timestamp == lastTimestamp || !isStable(state, timestamp)) {
            return Optional.empty();
        }
        lastTimestamp = timestamp;

        Rotation2d heading = inputs.megatagCount > 0
                ? inputs.megatagPoseEstimate.fieldToRobot().getRotation()
                : estimate.fieldToRobot().getRotation();
        Pose2d fieldToRobot = new Pose2d(estimate.fieldToRobot().getTranslation(), heading);

        return Optional.of(new VisionFieldPoseEstimate(
                fieldToRobot, timestamp, stdDevs(useMegatag2), estimate.fiducialIds().length));
    }

    private boolean isStable(RobotState state, double timestamp) {
        double yawRate = Math.abs(state
                .getMaxAbsDriveYawAngularVelocityInRange(timestamp - VisionConstants.kStabilityWindowSeconds, timestamp)
                .orElse(0.0));
        boolean stable = yawRate < VisionConstants.kMaxYawRateRadPerSec;
        Logger.recordOutput(logKey + "/YawRate", yawRate);
        Logger.recordOutput(logKey + "/Stable", stable);
        return stable;
    }

    private Matrix<N3, N1> stdDevs(boolean useMegatag2) {
        int tagCount = useMegatag2 ? inputs.megatag2Count : inputs.megatagCount;
        double avgDist = useMegatag2 ? inputs.megatag2AvgDist : inputs.megatagAvgDist;

        double factor = avgDist * avgDist / tagCount * config.stdDevFactor();
        double linear = VisionConstants.kLinearStdDevBaseline * factor;
        double angular = VisionConstants.kAngularStdDevBaseline * factor;
        if (useMegatag2) {
            linear *= VisionConstants.kLinearStdDevMegatag2Factor;
        }
        return VecBuilder.fill(linear, linear, angular);
    }

    private Pose3d[] targetPoses() {
        if (!inputs.seesTarget) {
            return new Pose3d[0];
        }
        MegatagPoseEstimate estimate = inputs.megatag2Count > 0 ? inputs.megatag2PoseEstimate : inputs.megatagPoseEstimate;
        return Arrays.stream(estimate.fiducialIds())
                .mapToObj(VisionConstants.kAprilTagLayout::getTagPose)
                .flatMap(Optional::stream)
                .toArray(Pose3d[]::new);
    }

    public CameraConfig getConfig() {
        return config;
    }
}
