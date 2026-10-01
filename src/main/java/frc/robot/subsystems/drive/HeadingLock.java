package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.TunableNumber;

public class HeadingLock {
    private final Supplier<Rotation2d> heading;
    private final DoubleSupplier yawRate;
    private final PIDController controller;
    private final TunableNumber toleranceRadians;
    private final TunableNumber captureRadPerSec;
    private static boolean engaged;
    private Rotation2d target;

    public HeadingLock(DriveConfig config, Supplier<Rotation2d> heading, DoubleSupplier yawRate) {
        this.heading = heading;
        this.yawRate = yawRate;
        controller = new PIDController(config.headingLockP, 0.0, config.headingLockD);
        controller.enableContinuousInput(-Math.PI, Math.PI);
        toleranceRadians = TunableNumber.field("Drive/Heading Lock Tolerance", config, "headingLockToleranceRadians").degrees();
        captureRadPerSec = TunableNumber.field("Drive/Heading Lock Capture", config, "headingLockCaptureRadPerSec").degrees();
        TunableNumber.field("Drive/Heading Lock kP", config, "headingLockP").onChange(controller::setP);
        TunableNumber.field("Drive/Heading Lock kD", config, "headingLockD").onChange(controller::setD);
    }

    public double hold() {
        Rotation2d current = heading.get();
        if (target == null) {
            if (Math.abs(yawRate.getAsDouble()) > captureRadPerSec.get()) {
                engaged = false;
                Logger.recordOutput("Drive/HeadingLock/Locked", false);
                return 0.0;
            }
            target = current;
            controller.reset();
        }
        engaged = true;
        Logger.recordOutput("Drive/HeadingLock/Locked", true);

        double error = target.minus(current).getRadians();
        Logger.recordOutput("Drive/HeadingLock/Target", target);
        Logger.recordOutput("Drive/HeadingLock/ErrorDegrees", Math.toDegrees(error));
        return Math.abs(error) < toleranceRadians.get() ? 0.0 : controller.calculate(current.getRadians(), target.getRadians());
    }

    public static boolean isEngaged() {
        return engaged;
    }

    public void release() {
        engaged = false;
        target = null;
        Logger.recordOutput("Drive/HeadingLock/Locked", false);
    }

    public boolean isLocked() {
        return target != null;
    }
}
