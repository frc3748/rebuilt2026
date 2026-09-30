package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class CameraIOPhotonSim extends CameraIOPhoton {
    private static VisionSystemSim visionSim;
    private static double lastSimUpdate = -1.0;

    private final Supplier<Pose2d> groundTruth;

    public CameraIOPhotonSim(CameraConfig config, Supplier<Pose2d> groundTruth) {
        super(config);
        this.groundTruth = groundTruth;

        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(VisionConstants.kAprilTagLayout);
        }

        SimCameraProperties properties = new SimCameraProperties();
        properties.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
        properties.setFPS(30);
        properties.setAvgLatencyMs(20);

        visionSim.addCamera(new PhotonCameraSim(camera, properties), config.robotToCamera());
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
