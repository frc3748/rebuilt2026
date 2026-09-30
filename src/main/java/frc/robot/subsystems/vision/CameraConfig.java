package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class CameraConfig {
    public enum Type {
        LIMELIGHT,
        PHOTON
    }

    private final String name;
    private final String networkName;
    private final Type type;
    private Transform3d robotToCamera = new Transform3d();
    private double stdDevFactor = 1.0;
    private boolean usedForPoseEstimation = true;

    public CameraConfig(String name, String networkName, Type type) {
        this.name = name;
        this.networkName = networkName;
        this.type = type;
    }

    public CameraConfig robotToCamera(Transform3d robotToCamera) {
        this.robotToCamera = robotToCamera;
        return this;
    }

    public CameraConfig stdDevFactor(double factor) {
        stdDevFactor = factor;
        return this;
    }

    public CameraConfig usedForPoseEstimation(boolean used) {
        usedForPoseEstimation = used;
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
        return robotToCamera;
    }

    public double stdDevFactor() {
        return stdDevFactor;
    }

    public boolean usedForPoseEstimation() {
        return usedForPoseEstimation;
    }
}
