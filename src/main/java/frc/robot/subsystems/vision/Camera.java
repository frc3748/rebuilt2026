package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.game.FieldConstants;
import frc.robot.subsystems.vision.CameraIO.ObjectObservation;
import frc.robot.subsystems.vision.CameraIO.PoseObservation;
import frc.robot.subsystems.vision.CameraIO.PoseSource;

public class Camera {
    private final CameraConfig config;
    private final CameraIO io;
    private final CameraInputsAutoLogged inputs = new CameraInputsAutoLogged();
    private final String logKey;
    private final Map<PoseSource, Double> lastTimestamps = new EnumMap<>(PoseSource.class);
    private final List<VisionMeasurement> measurements = new ArrayList<>();
    private final List<DetectedObject> objects = new ArrayList<>();
    private int requestedPipeline = -1;

    public Camera(CameraConfig config, CameraIO io) {
        this.config = config;
        this.io = io;
        logKey = "Vision/" + config.name();
    }

    public static Camera of(CameraConfig config, RobotState state) {
        CameraIO io = switch (Constants.kMode) {
            case REAL -> config.type().create(config);
            case SIM -> new CameraIOPhotonSim(config, () -> state.getSimRobot().getLatestFieldToRobot());
            case REPLAY -> new CameraIO() {};
        };
        return new Camera(config, io);
    }

    public void update(RobotState state, boolean detecting) {
        int pipeline = config.pipelineFor(detecting);
        if (config.canDetectObjects() && pipeline != requestedPipeline) {
            io.setPipeline(pipeline);
            requestedPipeline = pipeline;
        }

        Pose2d robot = state.getLatestFieldToRobot().getValue();
        io.setRobotToCamera(config.robotToCamera());
        io.setRobotOrientation(
                robot.getRotation(),
                Units.radiansToDegrees(state.getLatestRobotRelativeChassisSpeed().omegaRadiansPerSecond));
        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);

        measurements.clear();
        objects.clear();
        List<Pose2d> accepted = new ArrayList<>();
        List<Pose2d> rejected = new ArrayList<>();

        for (PoseObservation observation : inputs.poseObservations) {
            Pose2d pose = observation.robotPose().toPose2d().plus(config.reportedPoseOffset());
            if (isValid(observation, pose, state)) {
                measurements.add(toMeasurement(observation, pose));
                accepted.add(pose);
            } else {
                rejected.add(pose);
            }
        }
        for (ObjectObservation observation : inputs.objectObservations) {
            locate(observation, state).ifPresent(objects::add);
        }

        Logger.recordOutput(logKey + "/CameraPose", new Pose3d(robot).plus(config.robotToCamera()));
        Logger.recordOutput(logKey + "/AcceptedPoses", accepted.toArray(Pose2d[]::new));
        Logger.recordOutput(logKey + "/RejectedPoses", rejected.toArray(Pose2d[]::new));
        Logger.recordOutput(logKey + "/Tags", Arrays.stream(inputs.tagIds)
                .mapToObj(FieldConstants.TAG_LAYOUT::getTagPose)
                .flatMap(Optional::stream)
                .toArray(Pose3d[]::new));
        Logger.recordOutput(logKey + "/Objects", objects.stream()
                .map(DetectedObject::fieldPosition)
                .toArray(Translation2d[]::new));
    }

    private boolean isValid(PoseObservation observation, Pose2d pose, RobotState state) {
        boolean ambiguous = observation.source().rejectAmbiguous
                && observation.tagCount() == 1
                && observation.ambiguity() > kMaxAmbiguity;
        boolean offField = pose.getX() < 0 || pose.getX() > FieldConstants.LAYOUT_LENGTH_METERS
                || pose.getY() < 0 || pose.getY() > FieldConstants.LAYOUT_WIDTH_METERS;
        boolean repeated = observation.timestamp() == lastTimestamps.getOrDefault(observation.source(), -1.0);

        if (observation.tagCount() == 0
                || ambiguous
                || offField
                || repeated
                || pose.getTranslation().equals(Translation2d.kZero)
                || Math.abs(observation.robotPose().getZ()) > kMaxZErrorMeters
                || !isStable(state, observation.timestamp())) {
            return false;
        }
        lastTimestamps.put(observation.source(), observation.timestamp());
        return true;
    }

    private boolean isStable(RobotState state, double timestamp) {
        double yawRate = Math.abs(state
                .getMaxAbsDriveYawAngularVelocityInRange(timestamp - kStabilityWindowSeconds, timestamp)
                .orElse(0.0));
        return yawRate < kMaxYawRateRadPerSec;
    }

    private VisionMeasurement toMeasurement(PoseObservation observation, Pose2d pose) {
        double distance = observation.averageTagDistance();
        double factor = distance * distance / observation.tagCount() * config.stdDevFactor();
        double linear = kLinearStdDevBaseline * factor * observation.source().linearStdDevFactor;
        double angular = kAngularStdDevBaseline * factor * observation.source().angularStdDevFactor;
        return new VisionMeasurement(pose, observation.timestamp(), VecBuilder.fill(linear, linear, angular));
    }

    private Optional<DetectedObject> locate(ObjectObservation observation, RobotState state) {
        Transform3d robotToCamera = config.robotToCamera();
        double elevation = -robotToCamera.getRotation().getY() + Units.degreesToRadians(observation.pitchDegrees());
        double drop = robotToCamera.getZ() - config.objectHeightMeters();
        if (elevation >= 0 || drop <= 0) {
            return Optional.empty();
        }

        double range = drop / Math.tan(-elevation);
        Rotation2d bearing = new Rotation2d(
                robotToCamera.getRotation().getZ() - Units.degreesToRadians(observation.yawDegrees()));
        Translation2d robotRelative = robotToCamera.getTranslation().toTranslation2d()
                .plus(new Translation2d(range, bearing));
        Pose2d robot = state.getFieldToRobot(observation.timestamp())
                .orElse(state.getLatestFieldToRobot().getValue());
        Translation2d field = robot.transformBy(new Transform2d(robotRelative, Rotation2d.kZero)).getTranslation();

        return Optional.of(new DetectedObject(observation.timestamp(), observation.classId(), field, observation.confidence()));
    }

    public List<VisionMeasurement> getMeasurements() {
        return measurements;
    }

    public List<DetectedObject> getObjects() {
        return objects;
    }

    public CameraConfig getConfig() {
        return config;
    }
}
