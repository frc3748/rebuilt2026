package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.Comparator;
import java.util.List;

import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.game.FieldConstants;
import frc.robot.util.SimulatedRobotState;

public class CameraIOPhotonSim extends CameraIOPhoton {
    private static final String kGamePieceType = "gamepiece";
    private static final TargetModel kGamePieceModel = new TargetModel(FieldConstants.FUEL_DIAMETER.in(Meters));

    private static VisionSystemSim visionSim;
    private static double lastSimUpdate = -1.0;
    private static boolean simulatesGamePieces = false;

    private final SimulatedRobotState truth;
    private final PhotonCameraSim cameraSim;

    public CameraIOPhotonSim(CameraConfig config, SimulatedRobotState truth) {
        super(config);
        this.truth = truth;

        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(FieldConstants.TAG_LAYOUT);
        }
        simulatesGamePieces |= config.canDetectObjects();

        SimCameraProperties properties = new SimCameraProperties();
        properties.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
        properties.setFPS(30);
        properties.setAvgLatencyMs(20);

        cameraSim = new PhotonCameraSim(camera, properties);
        cameraSim.enableRawStream(false);
        cameraSim.enableProcessedStream(false);
        visionSim.addCamera(cameraSim, config.robotToCamera());
    }

    @Override
    public void setRobotToCamera(Transform3d robotToCamera) {
        super.setRobotToCamera(robotToCamera);
        visionSim.adjustCamera(cameraSim, robotToCamera);
    }

    @Override
    public void updateInputs(CameraInputs inputs) {
        Pose2d pose = truth.getLatestFieldToRobot();
        double now = Timer.getFPGATimestamp();
        if (pose != null && now != lastSimUpdate) {
            if (simulatesGamePieces) {
                placeGamePieces(pose);
            }
            visionSim.update(pose);
            lastSimUpdate = now;
        }
        super.updateInputs(inputs);
    }

    private void placeGamePieces(Pose2d robot) {
        Translation3d center = new Translation3d(robot.getX(), robot.getY(), 0.0);
        List<Translation3d> nearby = truth.getGamePieces().stream()
                .filter(piece -> piece.getDistance(center) < kSimObjectRangeMeters)
                .sorted(Comparator.comparingDouble(piece -> piece.getDistance(center)))
                .limit(kSimMaxObjects)
                .toList();
        visionSim.removeVisionTargets(kGamePieceType);
        visionSim.addVisionTargets(kGamePieceType, nearby.stream()
                .map(piece -> new VisionTargetSim(new Pose3d(piece, Rotation3d.kZero), kGamePieceModel))
                .toArray(VisionTargetSim[]::new));
    }
}
