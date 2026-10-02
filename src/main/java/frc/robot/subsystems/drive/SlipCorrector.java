package frc.robot.subsystems.drive;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.util.TunableNumber;

public class SlipCorrector {
    private static final double kGravity = 9.80665;
    private static final double kMovingSpeed = 0.25;
    private static final double kMaxTrustedAccelGs = 1.5;
    private static final double kImuSmoothing = 0.5;
    private static final double kMaxScale = 2.0;
    private static final double kSpeedFraction = 0.3;
    private static final double kStandOut = 2.0;

    private final Translation2d[] modules;
    private final TunableNumber moduleThreshold = new TunableNumber("Slip/Module Threshold", 0.5);
    private final TunableNumber robotThreshold = new TunableNumber("Slip/Robot Threshold", 3.0);
    private final TunableNumber maxTilt = new TunableNumber("Slip/Max Tilt", Math.toRadians(6.0)).degrees();

    private final boolean[] slipping = new boolean[4];
    private boolean robotSlipping;
    private boolean wasSlipping;
    private double lastWheelSpeed;
    private double estimatedSpeed;
    private double imuAccel;
    private double scale = 1.0;
    private int events;

    public SlipCorrector(Translation2d[] modules) {
        this.modules = modules;
    }

    public void update(ChassisSpeeds wheels, boolean imuConnected, double accelXGs, double accelYGs, double pitchRadians,
            double rollRadians, double dt) {
        double wheelSpeed = Math.hypot(wheels.vxMetersPerSecond, wheels.vyMetersPerSecond);
        double wheelAccel = (wheelSpeed - lastWheelSpeed) / dt;
        lastWheelSpeed = wheelSpeed;
        double measured = Math.hypot(accelXGs, accelYGs) * kGravity;
        imuAccel += kImuSmoothing * (measured - imuAccel);
        boolean trusted = imuConnected && Math.abs(pitchRadians) < maxTilt.get() && Math.abs(rollRadians) < maxTilt.get()
                && measured < kMaxTrustedAccelGs * kGravity;
        robotSlipping = trusted && wheelSpeed > kMovingSpeed && Math.abs(wheelAccel) > imuAccel + robotThreshold.get();
        if (robotSlipping) {
            estimatedSpeed = Math.max(0.0, estimatedSpeed + Math.signum(wheelAccel) * imuAccel * dt);
            scale = Math.min(kMaxScale, estimatedSpeed / wheelSpeed);
        } else {
            estimatedSpeed = wheelSpeed;
            scale = 1.0;
        }
    }

    public SwerveModulePosition[] correct(SwerveModulePosition[] deltas, double dtheta, double dt) {
        Translation2d[] rotation = new Translation2d[4];
        Translation2d[] implied = new Translation2d[4];
        for (int i = 0; i < 4; i++) {
            rotation[i] = new Translation2d(-dtheta * modules[i].getY(), dtheta * modules[i].getX());
            implied[i] = new Translation2d(deltas[i].distanceMeters, deltas[i].angle).minus(rotation[i]);
        }
        Translation2d consensus = median(implied);
        double speed = consensus.getNorm() / dt;
        SwerveModulePosition[] corrected = new SwerveModulePosition[4];
        boolean anySlipping = robotSlipping;
        double[] residuals = new double[4];
        for (int i = 0; i < 4; i++) {
            residuals[i] = implied[i].minus(consensus).getNorm() / dt;
        }
        for (int i = 0; i < 4; i++) {
            double others = 0.0;
            for (int j = 0; j < 4; j++) {
                if (j != i) {
                    others = Math.max(others, residuals[j]);
                }
            }
            double residual = residuals[i];
            slipping[i] = residual > Math.max(moduleThreshold.get(), kSpeedFraction * speed) && residual > kStandOut * others;
            anySlipping |= slipping[i];
            if (!slipping[i] && scale == 1.0) {
                corrected[i] = deltas[i];
                continue;
            }
            Translation2d move = (slipping[i] ? consensus : implied[i]).times(scale).plus(rotation[i]);
            Rotation2d angle = deltas[i].angle;
            corrected[i] = new SwerveModulePosition(move.getX() * angle.getCos() + move.getY() * angle.getSin(), angle);
        }
        if (anySlipping && !wasSlipping) {
            events++;
        }
        wasSlipping = anySlipping;
        return corrected;
    }

    public boolean[] slippingModules() {
        return slipping.clone();
    }

    public boolean isRobotSlipping() {
        return robotSlipping;
    }

    public double scale() {
        return scale;
    }

    public double estimatedSpeed() {
        return estimatedSpeed;
    }

    public int events() {
        return events;
    }

    private static Translation2d median(Translation2d[] values) {
        double[] xs = Arrays.stream(values).mapToDouble(Translation2d::getX).sorted().toArray();
        double[] ys = Arrays.stream(values).mapToDouble(Translation2d::getY).sorted().toArray();
        return new Translation2d((xs[1] + xs[2]) / 2.0, (ys[1] + ys[2]) / 2.0);
    }
}
