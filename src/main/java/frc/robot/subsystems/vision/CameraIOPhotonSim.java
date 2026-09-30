package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.game.FieldConstants;

public class CameraIOPhotonSim extends CameraIOPhoton {
    private static VisionSystemSim visionSim;
    private static double lastSimUpdate = -1.0;

    private final Supplier<Pose2d> groundTruth;
    private final PhotonCameraSim cameraSim;

    public CameraIOPhotonSim(CameraConfig config, Supplier<Pose2d> groundTruth) {
        super(config);
        this.groundTruth = groundTruth;

        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(FieldConstants.TAG_LAYOUT);
        }

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
        Pose2d pose = groundTruth.get();
        double now = Timer.getFPGATimestamp();
        if (pose != null && now != lastSimUpdate) {
            visionSim.update(pose);
            lastSimUpdate = now;
        }
        super.updateInputs(inputs);
    }
}
