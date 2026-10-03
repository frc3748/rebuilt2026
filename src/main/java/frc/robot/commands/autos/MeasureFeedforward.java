package frc.robot.commands.autos;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConfig;
import frc.robot.util.motor.AutoTune;
import frc.robot.util.motor.AutoTune.Feedback;
import frc.robot.util.motor.MotorAutoTune;

public class MeasureFeedforward extends MeasureAuto {
    private static final double kPointSeconds = 0.5;
    private static final double kRampVoltsPerSec = 1.0;
    private static final double kStepVolts = 3.0;
    private static final double kDistanceMeters = 2.0;
    private static final double kReturnMeters = 0.25;
    private static final double kStopSeconds = 1.0;
    private static final double kRampTimeoutSeconds = 10.0;
    private static final double kStepTimeoutSeconds = 3.0;
    private static final double kMaxVolts = 12.0;
    private static final double kMovingSpeed = 0.05;
    private static final double kDrivenVolts = 0.05;
    private static final double kCurrentFraction = 0.8;
    private static final double kMaxResidualVolts = 0.3;
    private static final int kMinSamples = 20;
    private static final double kSameKs = 0.05;
    private static final double kSameKv = 0.05;
    private static final double kSameKa = 0.05;
    private static final double kVelocityDelaySeconds = 0.02;

    private record Sample(double time, double volts, double velocity, double amps) {}

    private record Fit(double kS, double kV, double kA, double residual, int samples) {}

    private final List<Sample> samples = new ArrayList<>();
    private final Timer timer = new Timer();
    private Pose2d start = Pose2d.kZero;

    public MeasureFeedforward(RobotState state) {
        super(state, "Feedforward", "Measure: drive feedforward");
    }

    @Override
    protected Command measure() {
        Drive drive = state.getDrive();
        return Commands.sequence(
                Commands.runOnce(() -> {
                    samples.clear();
                    start = drive.getPose();
                }),
                Commands.run(() -> drive.runCharacterization(0.0), drive).withTimeout(kPointSeconds),
                Commands.runOnce(timer::restart),
                Commands.run(() -> drive(Math.min(kMaxVolts, timer.get() * kRampVoltsPerSec)), drive)
                        .until(() -> traveled() >= kDistanceMeters || timer.get() * kRampVoltsPerSec >= kMaxVolts)
                        .withTimeout(kRampTimeoutSeconds),
                Commands.run(() -> drive(0.0), drive).withTimeout(kStopSeconds),
                Commands.run(() -> drive(-kStepVolts), drive).until(() -> traveled() <= kReturnMeters)
                        .withTimeout(kStepTimeoutSeconds),
                Commands.run(() -> drive(0.0), drive).withTimeout(kStopSeconds),
                Commands.runOnce(this::finish));
    }

    private void drive(double volts) {
        Drive drive = state.getDrive();
        samples.add(new Sample(Timer.getFPGATimestamp(), drive.getDriveAppliedVolts(), drive.getWheelSpeedMetersPerSec(),
                drive.getDriveCurrentAmps()));
        drive.runCharacterization(volts);
    }

    private double traveled() {
        Pose2d pose = state.getDrive().getPose();
        return pose.getTranslation().minus(start.getTranslation()).rotateBy(start.getRotation().unaryMinus()).getX();
    }

    private void finish() {
        DriveConfig config = state.getDrive().getConfig();
        Fit best = null;
        for (int shift = 0; shift <= 1; shift++) {
            Fit fit = fit(shift, config.driveCurrentLimit * kCurrentFraction);
            if (fit != null && (best == null || fit.residual() < best.residual())) {
                best = fit;
            }
        }
        if (best == null || best.kV() <= 0.0) {
            failed("Not enough clean data. Give it " + kDistanceMeters + " m of open floor in front and try again.");
            return;
        }
        if (best.residual() > kMaxResidualVolts) {
            failed(String.format(Locale.ROOT, "The data was too noisy to trust (off by %.2f V on average)", best.residual()));
            return;
        }
        double kS = Math.max(0.0, best.kS());
        double kV = best.kV();
        double kA = Math.max(0.0, best.kA());
        double topSpeed = (kMaxVolts - kS) / kV;
        boolean same = Math.abs(kS - config.driveKs) <= kSameKs && Math.abs(kV - config.driveKv) <= kSameKv * config.driveKv
                && Math.abs(kA - config.driveKa) <= kSameKa;
        String summary = String.format(Locale.ROOT,
                "kS %.3f V, kV %.3f V per m/s, kA %.3f V per m/s², top speed about %.2f m/s (code says kS %.3f, kV %.3f, kA %.3f, %.2f m/s)",
                kS, kV, kA, topSpeed, config.driveKs, config.driveKv, config.driveKa, config.maxSpeedMetersPerSec);
        Map<String, Double> measured = new LinkedHashMap<>();
        measured.put("kS", kS);
        measured.put("kV", kV);
        measured.put("kA", kA);
        measured.put("TopSpeed", topSpeed);
        measured.put("ResidualVolts", best.residual());
        measured.put("Samples", (double) best.samples());
        Feedback feedback = AutoTune.startingVelocity(new AutoTune.Model(kS, kV, kA, 0.0, best.residual(), best.samples()),
                kVelocityDelaySeconds);
        double wheelRadius = config.wheelRadiusMeters;
        measured.put("kP", feedback.kP());
        Map<String, Double> proposed = new LinkedHashMap<>();
        proposed.put("Drive PID/kS", round(kS));
        proposed.put("Drive PID/kV", round(kV));
        proposed.put("Drive PID/kA", round(kA));
        proposed.put("Drive PID/kP", AutoTune.round(AutoTune.forSpark(feedback).kP() * wheelRadius));
        proposed.put("Drive Sim/kP", AutoTune.round(feedback.kP() * wheelRadius));
        report(same ? Outcome.SAME : Outcome.CHANGED, summary, measured, proposed);
        MotorAutoTune.record("Drive PID", true, summary);
        MotorAutoTune.record("Drive Sim", true, summary);
    }

    private void failed(String reason) {
        fail(reason);
        MotorAutoTune.record("Drive PID", false, reason);
        MotorAutoTune.record("Drive Sim", false, reason);
    }

    private Fit fit(int shift, double maxAmps) {
        List<double[]> rows = new ArrayList<>();
        for (int k = 0; k + 1 + shift < samples.size(); k++) {
            Sample from = samples.get(k);
            Sample to = samples.get(k + 1);
            double volts = samples.get(k + shift).volts();
            double dt = to.time() - from.time();
            double velocity = (from.velocity() + to.velocity()) / 2.0;
            if (dt <= 0.0 || Math.abs(velocity) < kMovingSpeed || Math.abs(volts) < kDrivenVolts
                    || Math.max(from.amps(), to.amps()) > maxAmps) {
                continue;
            }
            rows.add(new double[] {Math.signum(velocity), velocity, (to.velocity() - from.velocity()) / dt, volts});
        }
        if (rows.size() < kMinSamples) {
            return null;
        }
        SimpleMatrix x = new SimpleMatrix(rows.size(), 3);
        SimpleMatrix y = new SimpleMatrix(rows.size(), 1);
        for (int i = 0; i < rows.size(); i++) {
            double[] row = rows.get(i);
            x.setRow(i, 0, row[0], row[1], row[2]);
            y.set(i, 0, row[3]);
        }
        SimpleMatrix gains = x.solve(y);
        double residual = Math.sqrt(x.mult(gains).minus(y).elementPower(2).elementSum() / rows.size());
        return new Fit(gains.get(0), gains.get(1), gains.get(2), residual, rows.size());
    }

    private static double round(double value) {
        return Math.round(value * 1000.0) / 1000.0;
    }
}
