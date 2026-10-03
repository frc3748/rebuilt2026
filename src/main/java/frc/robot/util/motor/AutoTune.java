package frc.robot.util.motor;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.ejml.simple.SimpleMatrix;


public final class AutoTune {
    private static final double kNominalVolts = 12.0;
    private static final double kSparkPeriodSeconds = 0.001;
    private static final double kDelayFactor = 4.0;
    private static final double kMinTimeConstant = 0.05;
    private static final double kMinVelocityRatio = 0.2;
    private static final double kMinDrivenVolts = 0.05;
    private static final int kMinRows = 20;
    private static final int kSignificantDigits = 4;

    public enum Gravity {
        NONE,
        CONSTANT,
        COSINE
    }

    public record Sample(double time, double volts, double position, double velocity, double amps) {}

    public record Model(double kS, double kV, double kA, double kG, double residual, int rows) {}

    public record Feedback(double kP, double kD) {}

    private AutoTune() {}

    public static Optional<Model> fit(List<Sample> samples, Gravity gravity, double unitsPerRotation, double minSpeed,
            double maxAmps) {
        Model best = null;
        for (int shift = 0; shift <= 1; shift++) {
            Optional<Model> model = fit(samples, gravity, unitsPerRotation, minSpeed, maxAmps, shift);
            if (model.isPresent() && (best == null || model.get().residual() < best.residual())) {
                best = model.get();
            }
        }
        return Optional.ofNullable(best);
    }

    private static Optional<Model> fit(List<Sample> samples, Gravity gravity, double unitsPerRotation, double minSpeed,
            double maxAmps, int shift) {
        List<double[]> rows = new ArrayList<>();
        for (int k = 0; k + 1 + shift < samples.size(); k++) {
            Sample from = samples.get(k);
            Sample to = samples.get(k + 1);
            double volts = samples.get(k + shift).volts();
            double dt = to.time() - from.time();
            double velocity = (from.velocity() + to.velocity()) / 2.0;
            if (dt <= 0.0 || Math.abs(velocity) < minSpeed || Math.abs(volts) < kMinDrivenVolts
                    || Math.max(from.amps(), to.amps()) > maxAmps) {
                continue;
            }
            double position = (from.position() + to.position()) / 2.0;
            double load = switch (gravity) {
                case NONE -> 0.0;
                case CONSTANT -> 1.0;
                case COSINE -> Math.cos(position / unitsPerRotation * 2.0 * Math.PI);
            };
            rows.add(new double[] {Math.signum(velocity), velocity, (to.velocity() - from.velocity()) / dt, load, volts});
        }
        int columns = gravity == Gravity.NONE ? 3 : 4;
        if (rows.size() < kMinRows) {
            return Optional.empty();
        }
        SimpleMatrix x = new SimpleMatrix(rows.size(), columns);
        SimpleMatrix y = new SimpleMatrix(rows.size(), 1);
        for (int i = 0; i < rows.size(); i++) {
            double[] row = rows.get(i);
            for (int j = 0; j < columns; j++) {
                x.set(i, j, row[j]);
            }
            y.set(i, 0, row[4]);
        }
        SimpleMatrix gains = x.solve(y);
        double residual = Math.sqrt(x.mult(gains).minus(y).elementPower(2).elementSum() / rows.size());
        double kV = gains.get(1);
        double kA = gains.get(2);
        if (!(kV > 0.0) || !(kA > 0.0)) {
            return Optional.empty();
        }
        return Optional.of(new Model(Math.max(0.0, gains.get(0)), kV, kA, columns == 4 ? gains.get(3) : 0.0, residual, rows.size()));
    }

    public static Feedback startingVelocity(Model model, double delaySeconds) {
        double timeConstant = Math.max(kDelayFactor * delaySeconds, kMinTimeConstant);
        return new Feedback(Math.max(model.kA() / timeConstant - model.kV(), kMinVelocityRatio * model.kV()), 0.0);
    }

    public static Feedback startingPosition(Model model, double bandwidth, double damping, boolean derivative) {
        double kP = model.kA() * bandwidth * bandwidth;
        double kD = derivative ? Math.max(0.0, 2.0 * damping * bandwidth * model.kA() - model.kV()) : 0.0;
        return new Feedback(kP, kD);
    }

    public static Feedback scaled(Feedback feedback, double factor) {
        return new Feedback(feedback.kP() * factor, feedback.kD() * factor);
    }

    public static double round(double value) {
        if (value == 0.0 || !Double.isFinite(value)) {
            return 0.0;
        }
        double scale = Math.pow(10, kSignificantDigits - 1 - (int) Math.floor(Math.log10(Math.abs(value))));
        return Math.round(value * scale) / scale;
    }

    public static Feedback forSpark(Feedback volts) {
        return new Feedback(volts.kP() / kNominalVolts, volts.kD() / kNominalVolts / kSparkPeriodSeconds);
    }
}
