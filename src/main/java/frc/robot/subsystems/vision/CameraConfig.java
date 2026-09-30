package frc.robot.subsystems.vision;

import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

public class CameraConfig {
    public enum Type {
        LIMELIGHT_3(config -> new CameraIOLimelight(config, false)),
        LIMELIGHT_3G(config -> new CameraIOLimelight(config, false)),
        LIMELIGHT_4(config -> new CameraIOLimelight(config, true)),
        PHOTON(CameraIOPhoton::new);

        private final Function<CameraConfig, CameraIO> factory;

        Type(Function<CameraConfig, CameraIO> factory) {
            this.factory = factory;
        }

        public CameraIO create(CameraConfig config) {
            return factory.apply(config);
        }
    }

    private final String name;
    private final String networkName;
    private final Type type;
    private Supplier<Transform3d> robotToCamera = Transform3d::new;
    private Transform2d reportedPoseOffset = new Transform2d();
    private double stdDevFactor = 1.0;
    private int aprilTagPipeline = 0;
    private int detectionPipeline = -1;
    private double objectHeightMeters = 0.0;

    public CameraConfig(String name, String networkName, Type type) {
        this.name = name;
        this.networkName = networkName;
        this.type = type;
    }

    public CameraConfig robotToCamera(Transform3d robotToCamera) {
        return robotToCamera(() -> robotToCamera);
    }

    public CameraConfig robotToCamera(Supplier<Transform3d> robotToCamera) {
        this.robotToCamera = robotToCamera;
        return this;
    }

    public CameraConfig reportedPoseOffset(Transform2d offset) {
        reportedPoseOffset = offset;
        return this;
    }

    public CameraConfig stdDevFactor(double factor) {
        stdDevFactor = factor;
        return this;
    }

    public CameraConfig pipelines(int aprilTagPipeline, int detectionPipeline) {
        this.aprilTagPipeline = aprilTagPipeline;
        this.detectionPipeline = detectionPipeline;
        return this;
    }

    public CameraConfig objectHeight(double meters) {
        objectHeightMeters = meters;
        return this;
    }

    public String name() {
        return name;
    }

    public String networkName() {
        return networkName;
    }

    public Type type() {
        return type;
    }

    public Transform3d robotToCamera() {
        return robotToCamera.get();
    }

    public Transform2d reportedPoseOffset() {
        return reportedPoseOffset;
    }

    public double stdDevFactor() {
        return stdDevFactor;
    }

    public boolean canDetectObjects() {
        return detectionPipeline >= 0;
    }

    public int pipelineFor(boolean detecting) {
        return detecting && canDetectObjects() ? detectionPipeline : aprilTagPipeline;
    }

    public double objectHeightMeters() {
        return objectHeightMeters;
    }
}
