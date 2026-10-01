package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.game.FieldConstants;
import frc.robot.util.TunableNumber;
import frc.robot.util.Visuals;
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
    private final List<HeadingSample> headings = new ArrayList<>();
    private int requestedPipeline = -1;
    private double lastFixTime = Double.NEGATIVE_INFINITY;
    private final Alert disconnectedAlert;
    private final DoubleSupplier stdDevFactor;

    public Camera(CameraConfig config, CameraIO io) {
        this.config = config;
        this.io = io;
        logKey = "Vision/" + config.name();
        disconnectedAlert = new Alert(config.name() + " disconnected", AlertType.kWarning);
        stdDevFactor = config.stdDevSource().isKnown()
                ? new TunableNumber("Vision/" + config.name() + " Std Dev Factor", config.stdDevFactor(), config.stdDevSource())::get
                : config::stdDevFactor;
    }

    public static Camera of(CameraConfig config, RobotState state) {
        CameraIO io = switch (Constants.kMode) {
            case REAL -> config.type().create(config);
            case SIM -> new CameraIOPhotonSim(config, state.getSimRobot());
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
        disconnectedAlert.set(!inputs.connected);

        measurements.clear();
        objects.clear();
        headings.clear();
        if (config.estimatesPose()) {
            estimatePose(state);
        }
        if (config.detectsWith(pipeline)) {
            for (ObjectObservation observation : inputs.objectObservations) {
                locate(observation, state).ifPresent(objects::add);
            }
        }
        if (config.canDetectObjects()) {
            Logger.recordOutput(logKey + "/Objects", objects.stream()
                    .map(DetectedObject::fieldPosition)
                    .toArray(Translation2d[]::new));
        }
        if (Visuals.enabled()) {
            Visuals.record(logKey + "/CameraPose", new Pose3d(robot).plus(config.robotToCamera()));
        }
    }

    private void estimatePose(RobotState state) {
        List<Pose2d> accepted = new ArrayList<>();
        List<Pose2d> rejected = new ArrayList<>();
        List<String> reasons = new ArrayList<>();
        for (PoseObservation observation : inputs.poseObservations) {
            Pose2d pose = observation.robotPose().toPose2d().plus(config.reportedPoseOffset());
            Optional<String> reason = rejectionReason(observation, pose, state);
            if (reason.isEmpty()) {
                lastTimestamps.put(observation.source(), observation.timestamp());
                measurements.add(toMeasurement(observation, pose));
                lastFixTime = Timer.getFPGATimestamp();
                accepted.add(pose);
                if (observation.source().headingSource && isStrictHeading(observation, state)) {
                    headings.add(new HeadingSample(observation.timestamp(), pose.getRotation()));
                }
            } else {
                rejected.add(pose);
                reasons.add(reason.get());
            }
        }

        Logger.recordOutput(logKey + "/AcceptedPoses", accepted.toArray(Pose2d[]::new));
        Logger.recordOutput(logKey + "/RejectedPoses", rejected.toArray(Pose2d[]::new));
        Logger.recordOutput(logKey + "/RejectReasons", reasons.toArray(String[]::new));
        if (Visuals.enabled()) {
            Visuals.record(logKey + "/Tags", Arrays.stream(inputs.tagIds)
                    .mapToObj(FieldConstants.TAG_LAYOUT::getTagPose)
                    .flatMap(Optional::stream)
                    .toArray(Pose3d[]::new));
        }
    }

    private Optional<String> rejectionReason(PoseObservation observation, Pose2d pose, RobotState state) {
        if (observation.tagCount() == 0) {
            return Optional.of("no tags");
        }
        if (observation.source().rejectAmbiguous && observation.tagCount() == 1 && observation.ambiguity() > kMaxAmbiguity.get()) {
            return Optional.of("ambiguous");
        }
        if (pose.getX() < 0 || pose.getX() > FieldConstants.LAYOUT_LENGTH_METERS
                || pose.getY() < 0 || pose.getY() > FieldConstants.LAYOUT_WIDTH_METERS) {
            return Optional.of("off field");
        }
        if (observation.timestamp() == lastTimestamps.getOrDefault(observation.source(), -1.0)) {
            return Optional.of("repeated");
        }
        if (pose.getTranslation().equals(Translation2d.kZero)) {
            return Optional.of("zero pose");
        }
        if (Math.abs(observation.robotPose().getZ()) > kMaxZErrorMeters.get()) {
            return Optional.of("height");
        }
        if (!isStable(state, observation.timestamp())) {
            return Optional.of("spinning");
        }
        if (observation.source() == PoseSource.MEGATAG_1 && !isStrictHeading(observation, state)) {
            return Optional.of("not strict");
        }
        return Optional.empty();
    }

    private boolean isStrictHeading(PoseObservation observation, RobotState state) {
        double yawRate = Math.abs(state
                .getMaxAbsDriveYawAngularVelocityInRange(observation.timestamp() - kStabilityWindowSeconds, observation.timestamp())
                .orElse(0.0));
        return observation.tagCount() >= kStrictHeadingMinTags
                && observation.averageTagDistance() <= kStrictHeadingMaxDistanceMeters
                && observation.ambiguity() <= kStrictHeadingMaxAmbiguity
                && yawRate <= kStrictHeadingMaxYawRateRadPerSec;
    }

    private boolean isStable(RobotState state, double timestamp) {
        double yawRate = Math.abs(state
                .getMaxAbsDriveYawAngularVelocityInRange(timestamp - kStabilityWindowSeconds, timestamp)
                .orElse(0.0));
        return yawRate < kMaxYawRateRadPerSec;
    }

    private VisionMeasurement toMeasurement(PoseObservation observation, Pose2d pose) {
        double distance = observation.averageTagDistance();
        double factor = distance * distance / observation.tagCount() * stdDevFactor.getAsDouble();
        double linear = kLinearStdDevBaseline.get() * factor * observation.source().linearStdDevFactor;
        double angular = kAngularStdDevBaseline.get() * factor * observation.source().angularStdDevFactor;
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

    public List<HeadingSample> getHeadings() {
        return headings;
    }

    public int tagCount() {
        return inputs.tagIds.length;
    }

    public double secondsSinceFix() {
        return Timer.getFPGATimestamp() - lastFixTime;
    }

    public boolean isConnected() {
        return inputs.connected;
    }

    public CameraConfig getConfig() {
        return config;
    }
}
