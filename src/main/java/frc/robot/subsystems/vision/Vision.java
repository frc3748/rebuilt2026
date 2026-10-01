package frc.robot.subsystems.vision;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Deque;
import java.util.List;
import java.util.Locale;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import frc.robot.util.cockpit.Cockpit;
import frc.robot.util.cockpit.Cockpit.Level;
import frc.robot.util.state.StateMachine;

public class Vision extends StateMachine<Vision.State> {
    public enum State {
        UNDETERMINED,
        APRIL_TAGS,
        OBJECTS
    }

    private final RobotState robotState;
    private final Camera[] cameras;
    private final List<DetectedObject> objects = new ArrayList<>();
    private final Deque<HeadingSample> headings = new ArrayDeque<>();
    private Optional<Rotation2d> headingFix = Optional.empty();
    private static final double kAnnounceCorrectionDegrees = 1.0;
    private boolean headingConfirmed;
    private boolean estimating = true;
    private double lastVisionTime = Double.NEGATIVE_INFINITY;
    private final Alert noVisionAlert = new Alert("No vision pose for " + (int) VisionConstants.kNoVisionSeconds + " s", AlertType.kWarning);
    private final Alert disagreeAlert = new Alert("Vision and odometry disagree", AlertType.kError);

    public Vision(RobotState robotState, CameraConfig... configs) {
        this(robotState, Arrays.stream(configs).map(config -> Camera.of(config, robotState)).toArray(Camera[]::new));
    }

    public Vision(RobotState robotState, Camera... cameras) {
        super("Vision", State.UNDETERMINED, State.class);
        this.robotState = robotState;
        this.cameras = cameras;
        allowAllTransitions();
        enable();
        for (Camera camera : cameras) {
            CameraConfig config = camera.getConfig();
            boolean limelight = config.type() != CameraConfig.Type.PHOTON;
            Cockpit.camera(config.networkName(), config.name(),
                    Constants.kMode == Mode.SIM ? config.networkName() + "-processed" : config.networkName(),
                    limelight && Constants.kMode == Mode.REAL ? "http://" + config.networkName() + ".local:5800/stream.mjpg" : "",
                    camera::isConnected);
        }
    }

    @Override
    protected void applyState(State state) {
        boolean detecting = state == State.OBJECTS;
        double now = Timer.getFPGATimestamp();
        objects.removeIf(object -> now - object.timestamp() > VisionConstants.kObjectMemorySeconds);

        double disagreement = 0.0;
        for (Camera camera : cameras) {
            camera.update(robotState, detecting);
            for (VisionMeasurement measurement : camera.getMeasurements()) {
                if (Double.isFinite(measurement.stdDevs().get(0, 0))) {
                    lastVisionTime = now;
                    disagreement = Math.max(disagreement, robotState.getFieldToRobot(measurement.timestamp())
                            .map(pose -> pose.getTranslation().getDistance(measurement.robotPose().getTranslation()))
                            .orElse(0.0));
                }
            }
            if (estimating) {
                camera.getMeasurements().forEach(robotState::addVisionMeasurement);
            }
            camera.getObjects().forEach(this::remember);
            headings.addAll(camera.getHeadings());
        }
        checkHeading(now);
        boolean watching = DriverStation.isEnabled() && estimating && Arrays.stream(cameras).anyMatch(camera -> camera.getConfig().estimatesPose());
        noVisionAlert.set(watching && now - lastVisionTime > VisionConstants.kNoVisionSeconds);
        disagreeAlert.setText(String.format("Vision and odometry disagree by %.1f m", disagreement));
        disagreeAlert.set(watching && disagreement > VisionConstants.kDisagreeMeters);
        Logger.recordOutput("Vision/DisagreementMeters", disagreement);

        Logger.recordOutput("Vision/Estimating", estimating);
        Logger.recordOutput("Cockpit/Vision", Arrays.stream(cameras)
                .filter(camera -> camera.getConfig().estimatesPose())
                .map(camera -> String.join("\t", camera.getConfig().name(), camera.isConnected() ? "1" : "0",
                        Integer.toString(camera.tagCount()), String.format(Locale.ROOT, "%.2f", camera.secondsSinceFix())))
                .toArray(String[]::new));
        Logger.recordOutput("Vision/Objects", objects.stream()
                .map(DetectedObject::fieldPosition)
                .toArray(Translation2d[]::new));
        Logger.recordOutput("Vision/ClosestObject", getClosestObjectPose().stream().toArray(Pose2d[]::new));
    }

    private void checkHeading(double now) {
        headings.removeIf(sample -> now - sample.timestamp() > VisionConstants.kHeadingWindowSeconds);
        headingFix = agreedHeading();
        Pose2d robot = robotState.getLatestFieldToRobot().getValue();
        headingFix.ifPresent(fix -> {
            double error = Math.abs(fix.minus(robot.getRotation()).getDegrees());
            if (DriverStation.isDisabled() && estimating && error > VisionConstants.kHeadingCorrectionDegrees) {
                if (error > kAnnounceCorrectionDegrees) {
                    Cockpit.toast(Level.INFO, "Heading fixed by MegaTag 1", String.format("Corrected %.1f°", error));
                }
                robotState.getDrive().setPose(new Pose2d(robot.getTranslation(), fix));
                headings.clear();
                error = 0.0;
            }
            headingConfirmed = error <= VisionConstants.kHeadingAgreementDegrees;
        });
        Logger.recordOutput("Vision/HeadingConfirmed", headingConfirmed);
        Logger.recordOutput("Vision/HeadingSamples", headings.size());
        headingFix.ifPresent(fix -> Logger.recordOutput("Vision/HeadingFix", fix));
    }

    private Optional<Rotation2d> agreedHeading() {
        if (headings.size() < VisionConstants.kHeadingSamples) {
            return Optional.empty();
        }
        double sin = 0.0;
        double cos = 0.0;
        for (HeadingSample sample : headings) {
            sin += sample.heading().getSin();
            cos += sample.heading().getCos();
        }
        Rotation2d mean = new Rotation2d(cos, sin);
        for (HeadingSample sample : headings) {
            if (Math.abs(sample.heading().minus(mean).getDegrees()) > VisionConstants.kHeadingAgreementDegrees) {
                return Optional.empty();
            }
        }
        return Optional.of(mean);
    }

    public void setEstimating(boolean estimating) {
        this.estimating = estimating;
    }

    public boolean isEstimating() {
        return estimating;
    }

    public List<String> disconnectedCameras() {
        return Arrays.stream(cameras).filter(camera -> !camera.isConnected()).map(camera -> camera.getConfig().name()).toList();
    }

    public int cameraCount() {
        return cameras.length;
    }

    public boolean isHeadingConfirmed() {
        return headingConfirmed;
    }

    public Optional<Rotation2d> getHeadingFix() {
        return headingFix;
    }

    private void remember(DetectedObject object) {
        objects.removeIf(known -> known.distanceTo(object.fieldPosition()) < VisionConstants.kObjectMergeMeters);
        objects.add(object);
    }

    public List<DetectedObject> getObjects() {
        return objects;
    }

    public List<Pose2d> getObjectPoses() {
        Translation2d robot = robotTranslation();
        return objects.stream().map(object -> object.poseFrom(robot)).toList();
    }

    public Optional<DetectedObject> getClosestObject() {
        return getClosestObject(robotTranslation());
    }

    public Optional<DetectedObject> getClosestObject(Translation2d point) {
        return objects.stream().min(Comparator.comparingDouble(object -> object.distanceTo(point)));
    }

    public Optional<Pose2d> getClosestObjectPose() {
        Translation2d robot = robotTranslation();
        return getClosestObject(robot).map(object -> object.poseFrom(robot));
    }

    public boolean seesObjects() {
        return !objects.isEmpty();
    }

    private Translation2d robotTranslation() {
        return robotState.getLatestFieldToRobot().getValue().getTranslation();
    }

    public Camera[] getCameras() {
        return cameras;
    }

    @Override
    protected void determineSelf() {
        setState(State.APRIL_TAGS);
    }
}
