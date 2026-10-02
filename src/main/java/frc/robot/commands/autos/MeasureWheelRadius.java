package frc.robot.commands.autos;

import java.util.Locale;
import java.util.Map;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;

public class MeasureWheelRadius extends MeasureAuto {
    private static final double kSpinRadPerSec = 1.0;
    private static final double kRampRadPerSecSq = 0.5;
    private static final double kSettleSeconds = 1.0;
    private static final double kTurns = 1.5;
    private static final double kTimeoutSeconds = 20.0;
    private static final double kSameFraction = 0.01;

    private final SlewRateLimiter limiter = new SlewRateLimiter(kRampRadPerSecSq);
    private double[] startPositions;
    private Rotation2d lastAngle;
    private double gyroDelta;

    public MeasureWheelRadius(RobotState state) {
        super(state, "WheelRadius", "Measure: wheel radius");
    }

    @Override
    protected Command measure() {
        Drive drive = state.getDrive();
        return Commands.sequence(
                Commands.runOnce(() -> limiter.reset(0.0)),
                Commands.run(this::spin, drive).withTimeout(kSpinRadPerSec / kRampRadPerSecSq + kSettleSeconds),
                Commands.runOnce(() -> {
                    startPositions = drive.getWheelRadiusCharacterizationPositions();
                    lastAngle = drive.getGyroRotation();
                    gyroDelta = 0.0;
                }),
                Commands.run(() -> {
                    spin();
                    Rotation2d angle = drive.getGyroRotation();
                    gyroDelta += Math.abs(angle.minus(lastAngle).getRadians());
                    lastAngle = angle;
                }, drive).until(() -> gyroDelta >= kTurns * 2.0 * Math.PI).withTimeout(kTimeoutSeconds),
                Commands.runOnce(drive::stop, drive),
                Commands.runOnce(this::finish));
    }

    private void spin() {
        state.getDrive().runVelocity(new ChassisSpeeds(0.0, 0.0, limiter.calculate(kSpinRadPerSec)));
    }

    private void finish() {
        Drive drive = state.getDrive();
        double[] positions = drive.getWheelRadiusCharacterizationPositions();
        double wheelDelta = 0.0;
        for (int i = 0; i < 4; i++) {
            wheelDelta += Math.abs(positions[i] - startPositions[i]) / 4.0;
        }
        if (gyroDelta < Math.PI || wheelDelta <= 0.0) {
            fail("It didn't spin far enough to measure. Check the gyro and give it room to turn.");
            return;
        }
        double code = drive.getConfig().wheelRadiusMeters;
        double radius = gyroDelta * drive.getConfig().driveBaseRadius() / wheelDelta;
        boolean same = Math.abs(radius - code) / code <= kSameFraction;
        String summary = String.format(Locale.ROOT, "%.4f m (%.3f in), code says %.4f m", radius, Units.metersToInches(radius), code);
        report(same ? Outcome.SAME : Outcome.CHANGED, summary,
                Map.of("RadiusMeters", radius, "GyroRadians", gyroDelta, "WheelRadians", wheelDelta),
                Map.of("Drive/Wheel Radius", Math.round(radius * 1e5) / 1e5));
    }
}
