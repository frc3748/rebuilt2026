package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class HeadingLock {
    private final Supplier<Rotation2d> heading;
    private final DoubleSupplier yawRate;
    private final PIDController controller;
    private final double toleranceRadians;
    private final double captureRadPerSec;
    private Rotation2d target;

    public HeadingLock(DriveConfig config, Supplier<Rotation2d> heading, DoubleSupplier yawRate) {
        this.heading = heading;
        this.yawRate = yawRate;
        controller = new PIDController(config.headingLockP, 0.0, config.headingLockD);
        controller.enableContinuousInput(-Math.PI, Math.PI);
        toleranceRadians = config.headingLockToleranceRadians;
        captureRadPerSec = config.headingLockCaptureRadPerSec;
        DogLog.tunable("Drive/Heading Lock kP", config.headingLockP, controller::setP);
    }

    public double hold() {
        Rotation2d current = heading.get();
        if (target == null) {
            if (Math.abs(yawRate.getAsDouble()) > captureRadPerSec) {
                return 0.0;
            }
            target = current;
            controller.reset();
        }

        double error = target.minus(current).getRadians();
        Logger.recordOutput("Drive/HeadingLock/Target", target);
        Logger.recordOutput("Drive/HeadingLock/ErrorDegrees", Math.toDegrees(error));
        return Math.abs(error) < toleranceRadians ? 0.0 : controller.calculate(current.getRadians(), target.getRadians());
    }

    public void release() {
        target = null;
    }

    public boolean isLocked() {
        return target != null;
    }
}
