package frc.robot.util.motor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.cockpit.Cockpit;
import frc.robot.util.cockpit.Cockpit.Level;
import frc.robot.util.motor.AutoTune.Feedback;
import frc.robot.util.motor.AutoTune.Gravity;
import frc.robot.util.motor.AutoTune.Model;
import frc.robot.util.motor.AutoTune.Sample;
import frc.robot.util.state.StateMachine;
import frc.robot.util.tuning.Tuning;

public final class MotorAutoTune {
    private static final double kTestCurrentLimit = 30.0;
    private static final double kCurrentFraction = 0.8;
    private static final double kMinSpeedFraction = 0.02;
    private static final double kMaxResidualVolts = 0.5;

    private static final double kRampVoltsPerSec = 2.0;
    private static final double kMaxRampVolts = 6.0;
    private static final double kStepVolts = 4.0;
    private static final double kCoastSeconds = 1.5;
    private static final double kStepSeconds = 1.0;
    private static final double kVerifySpeedFraction = 0.5;

    private static final double kEdgeFraction = 0.12;
    private static final double kQuickFraction = 0.6;
    private static final double kSlowVoltsPerSec = 0.5;
    private static final double kMaxSlowVolts = 4.0;
    private static final double kSlowTraverseSeconds = 2.0;
    private static final double kLoopSeconds = 0.02;
    private static final double kQuickVolts = 3.0;
    private static final double kCenterSeconds = 1.5;
    private static final double kPauseSeconds = 0.5;
    private static final double kSlowTimeoutSeconds = 10.0;
    private static final double kQuickTimeoutSeconds = 0.15;
    private static final double kOvertravelFraction = 0.02;
    private static final double kEdgeSampleFraction = 0.03;

    private static final double kStartBandwidth = 6.0;
    private static final double kLoadVolts = 2.0;
    private static final double kLoadSeconds = 0.4;
    private static final double kReleaseSeconds = 0.8;
    private static final double kLoadReference = 0.1;
    private static final double kGoodDip = 0.03;
    private static final double kProfileFraction = 0.8;
    private static final double kDamping = 0.8;
    private static final double kStepUp = 1.5;
    private static final double kGoodSettle = 0.02;
    private static final int kMaxRounds = 7;
    private static final double kTailFraction = 0.4;
    private static final double kNoiseFraction = 0.005;
    private static final int kOscillationCrossings = 3;
    private static final double kStallSpeedFraction = 0.01;
    private static final double kStallSeconds = 0.5;
    private static final double kVerifySeconds = 1.5;
    private static final double kMaxOvershoot = 0.10;
    private static final double kBackoff = 0.65;

    private static final Map<String, String> results = new LinkedHashMap<>();
    private static String[] resultLines = new String[0];
    private static final Map<String, Result> lastResults = new HashMap<>();

    public record Result(boolean ok, String summary, Map<String, Double> proposed, List<String> applied) {}

    private enum Drive {
        VOLTAGE,
        HOLD,
        SPIN
    }

    private final StateMachine<?> machine;
    private final Motor motor;
    private final MotorConfig config;
    private final String label;
    private final boolean positional;
    private final double[] range;
    private final List<Sample> samples = new ArrayList<>();
    private final Map<Motor, Double> resting = new HashMap<>();
    private final Map<String, Double> originals = new LinkedHashMap<>();
    private final Timer timer = new Timer();
    private Drive drive = Drive.VOLTAGE;
    private double command;
    private double load;
    private double startVolts;
    private double slowVolts;
    private double peakSpeed;
    private double stalledSince = Double.NaN;
    private String failure;
    private Model model;
    private Feedback feedback;
    private Map<String, Double> proposed = new LinkedHashMap<>();
    private double worstOvershoot;
    private double settleError;
    private boolean oscillating;
    private int rounds;
    private boolean done;
    private Feedback lastGood;
    private double startingKp;
    private double goodOvershoot;

    private MotorAutoTune(StateMachine<?> machine, Motor motor) {
        this.machine = machine;
        this.motor = motor;
        config = motor.config();
        label = config == null ? motor.getName() : config.name();
        positional = motor instanceof PosMotor;
        range = config == null ? null : config.tuningRange();
    }

    public static boolean canTune(Motor motor) {
        MotorConfig config = motor.config();
        return config != null && (motor instanceof SpinMotor || (motor instanceof PosMotor && config.tuningRange() != null));
    }

    public static String group(Motor motor) {
        return motor.config() == null ? motor.getName() : motor.config().name();
    }

    public static String[] results() {
        return resultLines;
    }

    public static void record(String group, boolean ok, String summary) {
        results.put(group, String.join("\t", group, ok ? "ok" : "failed", summary));
        resultLines = results.values().toArray(String[]::new);
    }

    public static Optional<Result> lastResult(String label) {
        return Optional.ofNullable(lastResults.get(label));
    }

    public static Command build(StateMachine<?> machine, Motor motor) {
        MotorAutoTune tune = new MotorAutoTune(machine, motor);
        return Commands.either(tune.run(), Commands.runOnce(tune::refuse), tune::allowed).withName("Auto-tune " + tune.label);
    }

    private boolean allowed() {
        return canTune(motor) && DriverStation.isEnabled() && DriverStation.isTest() && Tuning.isActive();
    }

    private void refuse() {
        String reason = !canTune(motor) ? label + " needs a travel range (soft limits or tuneRange) before it can be auto-tuned"
                : !Tuning.isActive() ? "Turn on tuning mode first"
                        : "Enable Test mode with the robot on a cart";
        Cockpit.toast(Level.WARNING, "Can't auto-tune " + label, reason);
    }

    private Command run() {
        return Commands.sequence(
                Commands.runOnce(this::start),
                positional ? positionTests() : speedTests(),
                Commands.runOnce(this::calculate),
                verify(),
                Commands.runOnce(this::report))
                .until(() -> failure != null)
                .finallyDo(this::cleanUp);
    }

    private void start() {
        samples.clear();
        failure = null;
        model = null;
        feedback = null;
        proposed = new LinkedHashMap<>();
        worstOvershoot = 0.0;
        peakSpeed = 0.0;
        stalledSince = Double.NaN;
        resting.clear();
        originals.clear();
        machine.getMotors().forEach(each -> resting.put(each, each.getPosition()));
        motor.setCurrentLimit((int) Math.min(kTestCurrentLimit, config.currentLimit));
        drive(Drive.HOLD, positional ? middle() : 0.0);
        machine.setOverride(this::apply);
        Logger.recordOutput("AutoTune/Active", true);
        Logger.recordOutput("AutoTune/Motor", label);
    }

    private void apply() {
        machine.getMotors().stream().filter(each -> each != motor).forEach(each -> {
            if (each instanceof PosMotor) {
                each.holdPosition(resting.get(each));
            } else {
                each.stop();
            }
        });
        switch (drive) {
            case VOLTAGE -> motor.setVoltage(command);
            case HOLD -> {
                if (positional) {
                    motor.holdPosition(command);
                } else {
                    motor.stop();
                }
            }
            case SPIN -> ((SpinMotor) motor).set(command, load);
        }
    }

    private void drive(Drive mode, double value) {
        drive(mode, value, 0.0);
    }

    private void drive(Drive mode, double value, double loadVolts) {
        drive = mode;
        command = value;
        load = loadVolts;
    }

    private Command step(String name, Runnable each, BooleanSupplier done, double timeout) {
        return Commands.run(() -> {
            each.run();
            record();
        }).beforeStarting(() -> {
            timer.restart();
            Logger.recordOutput("AutoTune/Step", name);
        }).until(done).withTimeout(timeout);
    }

    private Command speedTests() {
        return Commands.sequence(
                step("Ramp", () -> drive(Drive.VOLTAGE, Math.min(kMaxRampVolts, timer.get() * kRampVoltsPerSec)),
                        () -> timer.get() * kRampVoltsPerSec >= kMaxRampVolts, kMaxRampVolts / kRampVoltsPerSec + 1.0),
                step("Coast", () -> drive(Drive.VOLTAGE, 0.0), () -> false, kCoastSeconds),
                step("Step", () -> drive(Drive.VOLTAGE, kStepVolts), () -> false, kStepSeconds),
                step("Stop", () -> drive(Drive.VOLTAGE, 0.0), () -> false, kCoastSeconds));
    }

    private Command positionTests() {
        double span = range[1] - range[0];
        double low = range[0] + kEdgeFraction * span;
        double high = range[1] - kEdgeFraction * span;
        double middle = middle();
        return Commands.sequence(
                step("Center", () -> drive(Drive.HOLD, middle), () -> false, kCenterSeconds),
                Commands.runOnce(() -> startVolts = motor.getAppliedVolts()),
                Commands.runOnce(() -> slowVolts = 0.0),
                step("Slow up", () -> drive(Drive.VOLTAGE, startVolts + slowRamp(span)),
                        () -> motor.getPosition() + stoppingDistance() >= high, kSlowTimeoutSeconds),
                step("Pause", () -> drive(Drive.HOLD, high), () -> false, kPauseSeconds),
                Commands.runOnce(() -> slowVolts = 0.0),
                step("Slow down", () -> drive(Drive.VOLTAGE, startVolts - slowRamp(span)),
                        () -> motor.getPosition() - stoppingDistance() <= low, kSlowTimeoutSeconds),
                step("Pause", () -> drive(Drive.HOLD, low), () -> false, kPauseSeconds),
                step("Quick up", () -> drive(Drive.VOLTAGE, startVolts + kQuickVolts),
                        () -> motor.getPosition() >= low + kQuickFraction * span, kQuickTimeoutSeconds),
                step("Pause", () -> drive(Drive.HOLD, motor.getPosition()), () -> false, kPauseSeconds),
                step("Over", () -> drive(Drive.HOLD, high), () -> false, kCenterSeconds),
                step("Quick down", () -> drive(Drive.VOLTAGE, startVolts - kQuickVolts),
                        () -> motor.getPosition() <= high - kQuickFraction * span, kQuickTimeoutSeconds),
                step("Pause", () -> drive(Drive.HOLD, motor.getPosition()), () -> false, kPauseSeconds),
                step("Settle", () -> drive(Drive.HOLD, middle), () -> false, kCenterSeconds));
    }

    private double slowRamp(double span) {
        if (Math.abs(motor.getVelocity()) < span / kSlowTraverseSeconds) {
            slowVolts = Math.min(kMaxSlowVolts, slowVolts + kSlowVoltsPerSec * kLoopSeconds);
        }
        return slowVolts;
    }

    private double stoppingDistance() {
        double velocity = motor.getVelocity();
        double deceleration = config.maxMotion ? config.gains.maxAccel : 0.0;
        return deceleration > 0.0 ? velocity * velocity / (2.0 * deceleration) : 0.0;
    }

    private double middle() {
        return range == null ? 0.0 : (range[0] + range[1]) / 2.0;
    }

    private void record() {
        double speed = Math.abs(motor.getVelocity());
        peakSpeed = Math.max(peakSpeed, speed);
        if (drive == Drive.VOLTAGE && !nearEdge()) {
            samples.add(new Sample(Timer.getFPGATimestamp(), motor.getAppliedVolts(), motor.getPosition(), motor.getVelocity(),
                    motor.getCurrentAmps()));
        }
        if (positional) {
            double span = range[1] - range[0];
            if (motor.getPosition() < range[0] - kOvertravelFraction * span || motor.getPosition() > range[1] + kOvertravelFraction * span) {
                failure = "It went past its travel range, so the test stopped";
            }
        }
        boolean pushing = drive == Drive.VOLTAGE && Math.abs(command) > 1.0;
        boolean stopped = speed <= kStallSpeedFraction * Math.max(peakSpeed, 1e-9);
        boolean loaded = motor.getCurrentAmps() >= kCurrentFraction * Math.min(kTestCurrentLimit, config.currentLimit);
        if (pushing && stopped && loaded) {
            stalledSince = Double.isNaN(stalledSince) ? Timer.getFPGATimestamp() : stalledSince;
            if (Timer.getFPGATimestamp() - stalledSince > kStallSeconds) {
                failure = "It stalled, so the test stopped. Check nothing is jammed";
            }
        } else {
            stalledSince = Double.NaN;
        }
    }

    private boolean nearEdge() {
        if (!positional) {
            return false;
        }
        double edge = kEdgeSampleFraction * (range[1] - range[0]);
        return motor.getPosition() <= range[0] + edge || motor.getPosition() >= range[1] - edge;
    }

    private void calculate() {
        Gravity gravity = !positional ? Gravity.NONE : config.gains.gravityIsCosine ? Gravity.COSINE : Gravity.CONSTANT;
        double maxAmps = kCurrentFraction * Math.min(kTestCurrentLimit, config.currentLimit);
        Optional<Model> fit = AutoTune.fit(samples, gravity, config.gains.unitsPerRotation, kMinSpeedFraction * peakSpeed, maxAmps);
        if (fit.isEmpty() || fit.get().residual() > kMaxResidualVolts) {
            failure = fit.isEmpty() ? "Not enough clean movement to measure" : String.format(Locale.ROOT,
                    "The measurements were too noisy (off by %.2f V)", fit.get().residual());
            return;
        }
        model = fit.get();
        boolean talon = config.controller == MotorConfig.Controller.TALON_FX;
        Feedback volts = positional
                ? AutoTune.startingPosition(model, kStartBandwidth, kDamping, talon)
                : AutoTune.startingVelocity(model, velocityDelay());
        feedback = talon ? volts : AutoTune.forSpark(volts);
        startingKp = feedback.kP();
        propose();
    }

    private double velocityDelay() {
        return switch (config.controller) {
            case SPARK_MAX -> 0.05;
            case SPARK_FLEX -> 0.03;
            case TALON_FX -> 0.01;
        };
    }

    private void propose() {
        proposed = new LinkedHashMap<>();
        proposed.put("kS", model.kS());
        proposed.put("kV", model.kV());
        proposed.put("kA", model.kA());
        if (positional) {
            proposed.put(config.gains.gravityIsCosine ? "kCos" : "kG", model.kG());
            proposed.put("kD", feedback.kD());
        }
        proposed.put("kP", feedback.kP());
        if (config.maxMotion) {
            double headroom = 12.0 - model.kS() - Math.abs(model.kG());
            if (positional) {
                proposed.put("kCruiseVel", Math.min(config.gains.cruiseVel, kProfileFraction * headroom / model.kV()));
            }
            proposed.put("kMaxAccel", Math.min(config.gains.maxAccel, kProfileFraction * headroom / model.kA()));
        }
        proposed.replaceAll((gain, value) -> AutoTune.round(value));
        proposed.forEach((gain, value) -> {
            originals.computeIfAbsent(gain, config::value);
            config.apply(gain, value);
            Tuning.propose(label + "/" + gain, value);
        });
    }

    private Command verify() {
        Command check = positional ? positionCheck() : speedCheck();
        return Commands.sequence(
                Commands.runOnce(() -> {
                    rounds = 0;
                    done = false;
                    lastGood = null;
                }),
                Commands.sequence(Commands.waitSeconds(0.1), check, Commands.runOnce(this::refine)).repeatedly().until(() -> done))
                .onlyIf(() -> model != null);
    }

    private void refine() {
        boolean bad = worstOvershoot > kMaxOvershoot || oscillating;
        if (bad) {
            feedback = lastGood != null ? lastGood : AutoTune.scaled(feedback, kBackoff);
            done = lastGood != null || rounds + 1 >= kMaxRounds;
        } else {
            lastGood = feedback;
            goodOvershoot = worstOvershoot;
            done = settleError <= (positional ? kGoodSettle : kGoodDip) || rounds + 1 >= kMaxRounds;
            if (!done) {
                feedback = AutoTune.scaled(feedback, kStepUp);
            }
        }
        rounds++;
        propose();
        Logger.recordOutput("AutoTune/" + label + "/Round", rounds);
        Logger.recordOutput("AutoTune/" + label + "/Overshoot", worstOvershoot);
        Logger.recordOutput("AutoTune/" + label + "/SettleError", settleError);
    }

    private Command speedCheck() {
        List<double[]> start = new ArrayList<>();
        List<double[]> loaded = new ArrayList<>();
        List<double[]> release = new ArrayList<>();
        double[] target = new double[1];
        return Commands.sequence(
                Commands.runOnce(() -> {
                    target[0] = kVerifySpeedFraction * 12.0 / model.kV();
                    start.clear();
                    loaded.clear();
                    release.clear();
                }),
                step("Check", () -> {
                    drive(Drive.SPIN, target[0]);
                    start.add(new double[] {motor.getVelocity()});
                }, () -> false, kVerifySeconds),
                step("Load", () -> {
                    drive(Drive.SPIN, target[0], -kLoadVolts);
                    loaded.add(new double[] {motor.getVelocity()});
                }, () -> false, kLoadSeconds),
                step("Release", () -> {
                    drive(Drive.SPIN, target[0]);
                    release.add(new double[] {motor.getVelocity()});
                }, () -> false, kReleaseSeconds),
                Commands.runOnce(() -> {
                    grade(start, 0.0, target[0]);
                    double spinUp = worstOvershoot;
                    grade(release, target[0] * (1.0 - kLoadReference), target[0]);
                    worstOvershoot = Math.max(spinUp, worstOvershoot);
                    double lowest = loaded.stream().mapToDouble(sample -> sample[0]).min().orElse(target[0]);
                    settleError = Math.max(0.0, target[0] - lowest) / target[0];
                }),
                step("Stop", () -> drive(Drive.VOLTAGE, 0.0), () -> false, kCoastSeconds));
    }

    private Command positionCheck() {
        double span = range[1] - range[0];
        double near = range[0] + 0.3 * span;
        double far = range[0] + 0.7 * span;
        List<double[]> out = new ArrayList<>();
        List<double[]> back = new ArrayList<>();
        double[] grades = new double[3];
        return Commands.sequence(
                Commands.runOnce(() -> {
                    out.clear();
                    back.clear();
                }),
                step("Check", () -> drive(Drive.HOLD, near), () -> false, kVerifySeconds),
                step("Check", () -> {
                    drive(Drive.HOLD, far);
                    out.add(new double[] {motor.getPosition()});
                }, () -> false, kVerifySeconds),
                Commands.runOnce(() -> {
                    grade(out, near, far);
                    grades[0] = worstOvershoot;
                    grades[1] = settleError;
                    grades[2] = oscillating ? 1.0 : 0.0;
                }),
                step("Check", () -> {
                    drive(Drive.HOLD, near);
                    back.add(new double[] {motor.getPosition()});
                }, () -> false, kVerifySeconds),
                Commands.runOnce(() -> {
                    grade(back, far, near);
                    worstOvershoot = Math.max(worstOvershoot, grades[0]);
                    settleError = Math.max(settleError, grades[1]);
                    oscillating |= grades[2] > 0.0;
                }));
    }

    private void grade(List<double[]> trace, double from, double to) {
        double move = Math.abs(to - from);
        double direction = Math.signum(to - from);
        double beyond = 0.0;
        int crossings = 0;
        double lastSign = 0.0;
        for (int i = 0; i < trace.size(); i++) {
            double error = trace.get(i)[0] - to;
            beyond = Math.max(beyond, error * direction);
            if (i >= trace.size() * kTailFraction && Math.abs(error) > kNoiseFraction * move) {
                double sign = Math.signum(error);
                if (lastSign != 0.0 && sign != lastSign) {
                    crossings++;
                }
                lastSign = sign;
            }
        }
        double last = trace.isEmpty() ? from : trace.get(trace.size() - 1)[0];
        worstOvershoot = beyond / move;
        settleError = Math.abs(last - to) / move;
        oscillating = crossings >= kOscillationCrossings;
    }

    private void report() {
        List<String> applied = new ArrayList<>();
        List<String> missing = new ArrayList<>();
        proposed.forEach((gain, value) -> {
            String key = label + "/" + gain;
            Logger.recordOutput("AutoTune/" + label + "/" + gain, value);
            if (Tuning.propose(key, value)) {
                applied.add(gain);
            } else {
                missing.add(gain);
            }
        });
        String summary = proposed.entrySet().stream()
                .map(entry -> entry.getKey() + " " + format(entry.getValue()))
                .reduce((a, b) -> a + ", " + b).orElse("");
        String next = missing.isEmpty() ? "On the Tune tab. Press Save to keep them."
                : "Add " + String.join(", ", missing) + " to " + label + "'s MotorConfig to make them tunable; the rest are on the Tune tab.";
        String rounding = rounds <= 1 ? "" : String.format(Locale.ROOT, ", %d rounds up from kP %s", rounds, format(startingKp));
        Result result = new Result(lastGood != null, summary + String.format(Locale.ROOT, " (overshoot %.0f%%%s)",
                (lastGood != null ? goodOvershoot : worstOvershoot) * 100.0, rounding),
                proposed, applied);
        save(result);
        Cockpit.toast(result.ok() ? Level.INFO : Level.WARNING, result.ok() ? label + " auto-tuned" : label + " always overshot",
                result.summary() + ". " + (result.ok() ? next : "These are backed off but not proven. Check it by hand."));
    }

    private void cleanUp() {
        machine.clearOverride();
        originals.forEach((gain, value) -> {
            if (failure != null || !config.tunable(gain)) {
                config.apply(gain, value);
            }
        });
        motor.setCurrentLimit(config.currentLimit);
        Logger.recordOutput("AutoTune/Active", false);
        Logger.recordOutput("AutoTune/Step", "");
        if (failure != null) {
            save(new Result(false, failure, Map.of(), List.of()));
            Cockpit.toast(Level.WARNING, "Couldn't auto-tune " + label, failure);
        }
    }

    private void save(Result result) {
        lastResults.put(label, result);
        results.put(label, String.join("\t", label, result.ok() ? "ok" : "failed", result.summary()));
        resultLines = results.values().toArray(String[]::new);
        Logger.recordOutput("AutoTune/" + label + "/Summary", result.summary());
    }

    private static String format(double value) {
        return String.format(Locale.ROOT, "%.4g", value);
    }
}
