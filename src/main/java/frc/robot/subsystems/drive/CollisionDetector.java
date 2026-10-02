package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.TunableNumber;

public class CollisionDetector {
    private static final double kGravity = 9.80665;

    private final TunableNumber threshold = new TunableNumber("Collision/Threshold Gs", 1.0);
    private final TunableNumber hardHit = new TunableNumber("Collision/Hard Hit Gs", 2.0);
    private final TunableNumber maxTilt = new TunableNumber("Collision/Max Tilt", Math.toRadians(6.0)).degrees();
    private final TunableNumber trustSeconds = new TunableNumber("Collision/Vision Seconds", 1.0);
    private final TunableNumber visionScale = new TunableNumber("Collision/Vision Std Dev Scale", 0.25);

    private double lastVx;
    private double lastVy;
    private double jolt;
    private boolean hit;
    private boolean tilted;
    private double lastUpsetTime = Double.NEGATIVE_INFINITY;
    private double now;
    private int events;

    public void update(ChassisSpeeds wheels, boolean imuConnected, double accelXGs, double accelYGs, double pitchRadians,
            double rollRadians, double timestamp, double dt) {
        double wheelGs = Math.hypot(wheels.vxMetersPerSecond - lastVx, wheels.vyMetersPerSecond - lastVy) / dt / kGravity;
        lastVx = wheels.vxMetersPerSecond;
        lastVy = wheels.vyMetersPerSecond;
        now = timestamp;
        tilted = imuConnected && (Math.abs(pitchRadians) > maxTilt.get() || Math.abs(rollRadians) > maxTilt.get());
        double imuGs = imuConnected ? Math.hypot(accelXGs, accelYGs) : 0.0;
        jolt = Math.max(0.0, imuGs - wheelGs);
        boolean wasHit = hit;
        hit = !tilted && (jolt > threshold.get() || imuGs > hardHit.get());
        if (hit && !wasHit && !isUpset()) {
            events++;
        }
        if (hit || tilted) {
            lastUpsetTime = timestamp;
        }
    }

    public boolean isUpset() {
        return now - lastUpsetTime < trustSeconds.get();
    }

    public double visionStdDevScale() {
        return isUpset() ? visionScale.get() : 1.0;
    }

    public boolean isHit() {
        return hit;
    }

    public boolean isTilted() {
        return tilted;
    }

    public double jolt() {
        return jolt;
    }

    public int events() {
        return events;
    }
}
