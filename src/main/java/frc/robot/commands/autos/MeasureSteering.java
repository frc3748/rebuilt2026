package frc.robot.commands.autos;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConfig;
import frc.robot.subsystems.drive.Module;
import frc.robot.util.motor.AutoTune;
import frc.robot.util.motor.AutoTune.Feedback;
import frc.robot.util.motor.AutoTune.Gravity;
import frc.robot.util.motor.AutoTune.Model;
import frc.robot.util.motor.AutoTune.Sample;
import frc.robot.util.motor.MotorAutoTune;
import frc.robot.util.motor.MotorConfig;

public class MeasureSteering extends MeasureAuto {
    private static final double kRampVoltsPerSec = 1.0;
    private static final double kMaxRampVolts = 3.0;
    private static final double kStepVolts = 2.0;
    private static final double kStepSeconds = 0.6;
    private static final double kRestSeconds = 0.6;
    private static final double kBandwidth = 25.0;
    private static final double kDamping = 0.8;
    private static final double kCurrentFraction = 0.8;
    private static final double kMinSpeedFraction = 0.02;
    private static final double kMaxResidualVolts = 0.5;
    private static final double kSameFraction = 0.1;

    private final List<List<Sample>> samples = new ArrayList<>();
    private final double[] unwrapped = new double[4];
    private final Rotation2d[] last = new Rotation2d[4];
    private final Timer timer = new Timer();
    private double peakSpeed;

    public MeasureSteering(RobotState state) {
        super(state, "Steering", "Measure: steering");
    }

    @Override
    protected Command measure() {
        Drive drive = state.getDrive();
        return Commands.sequence(
                Commands.runOnce(this::reset),
                rest(),
                step(() -> Math.min(kMaxRampVolts, timer.get() * kRampVoltsPerSec), kMaxRampVolts / kRampVoltsPerSec),
                rest(),
                step(() -> kStepVolts, kStepSeconds),
                rest(),
                step(() -> -Math.min(kMaxRampVolts, timer.get() * kRampVoltsPerSec), kMaxRampVolts / kRampVoltsPerSec),
                rest(),
                step(() -> -kStepVolts, kStepSeconds),
                rest(),
                Commands.runOnce(drive::stop, drive),
                Commands.runOnce(this::finish));
    }

    private void reset() {
        samples.clear();
        Module[] modules = state.getDrive().modules();
        for (int i = 0; i < modules.length; i++) {
            samples.add(new ArrayList<>());
            unwrapped[i] = modules[i].getAngle().getRadians();
            last[i] = modules[i].getAngle();
        }
        peakSpeed = 0.0;
    }

    private Command rest() {
        return step(() -> 0.0, kRestSeconds);
    }

    private Command step(DoubleSupplier volts, double seconds) {
        Drive drive = state.getDrive();
        return Commands.run(() -> {
            record();
            drive.runSteerVoltage(volts.getAsDouble());
        }, drive).beforeStarting(timer::restart).withTimeout(seconds);
    }

    private void record() {
        Module[] modules = state.getDrive().modules();
        double now = Timer.getFPGATimestamp();
        for (int i = 0; i < modules.length; i++) {
            Rotation2d angle = modules[i].getAngle();
            unwrapped[i] += angle.minus(last[i]).getRadians();
            last[i] = angle;
            peakSpeed = Math.max(peakSpeed, Math.abs(modules[i].getSteerVelocity()));
            samples.get(i).add(new Sample(now, modules[i].getSteerAppliedVolts(), unwrapped[i], modules[i].getSteerVelocity(),
                    modules[i].getSteerCurrentAmps()));
        }
    }

    private void finish() {
        DriveConfig config = state.getDrive().getConfig();
        double maxAmps = kCurrentFraction * MotorConfig.safeCurrentLimit(config.turnCurrentLimit);
        List<Model> models = new ArrayList<>();
        for (List<Sample> module : samples) {
            Optional<Model> model = AutoTune.fit(module, Gravity.NONE, 1.0, kMinSpeedFraction * peakSpeed, maxAmps);
            if (model.isEmpty() || model.get().residual() > kMaxResidualVolts) {
                String reason = "A module didn't steer cleanly enough to measure. Check every module turns freely.";
                fail(reason);
                MotorAutoTune.record("Turn PID", false, reason);
                MotorAutoTune.record("Turn Sim", false, reason);
                return;
            }
            models.add(model.get());
        }
        double kS = models.stream().mapToDouble(Model::kS).average().orElse(0.0);
        double kV = models.stream().mapToDouble(Model::kV).average().orElse(0.0);
        double kA = models.stream().mapToDouble(Model::kA).average().orElse(0.0);
        double spread = models.stream().mapToDouble(Model::kV).max().orElse(0.0) / models.stream().mapToDouble(Model::kV).min().orElse(1.0);
        Model average = new Model(kS, kV, kA, 0.0, 0.0, 0);
        Feedback volts = AutoTune.startingPosition(average, kBandwidth, kDamping, false);
        Feedback spark = AutoTune.forSpark(volts);
        double steerFeedforward = kV / config.steerKv();

        Map<String, Double> measured = new LinkedHashMap<>();
        measured.put("kS", kS);
        measured.put("kV", kV);
        measured.put("kA", kA);
        measured.put("Spread", spread);
        Map<String, Double> proposed = new LinkedHashMap<>();
        proposed.put("Turn PID/kP", AutoTune.round(spark.kP()));
        proposed.put("Turn PID/kD", AutoTune.round(spark.kD()));
        proposed.put("Turn PID/Steer FF", AutoTune.round(steerFeedforward));
        proposed.put("Turn Sim/kP", AutoTune.round(volts.kP()));
        proposed.put("Turn Sim/kD", AutoTune.round(volts.kD()));
        boolean same = Math.abs(spark.kP() - config.turnKp) <= kSameFraction * Math.abs(config.turnKp);
        String summary = String.format(Locale.ROOT,
                "Steering kS %.3f V, kV %.3f V per rad/s, kA %.4f; modules within %.0f%% of each other. kP %.4g, kD %.4g (code says kP %.4g)",
                kS, kV, kA, (spread - 1.0) * 100.0, spark.kP(), spark.kD(), config.turnKp);
        report(same ? Outcome.SAME : Outcome.CHANGED, summary, measured, proposed);
        MotorAutoTune.record("Turn PID", true, summary);
        MotorAutoTune.record("Turn Sim", true, summary);
    }
}
