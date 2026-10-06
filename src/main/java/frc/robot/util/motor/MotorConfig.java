package frc.robot.util.motor;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

import frc.robot.util.tuning.Source;

public class MotorConfig {
    public enum Controller {
        SPARK_MAX,
        SPARK_FLEX,
        TALON_FX
    }

    public record Follower(int canId, boolean inverted) {}

    public static final int kDefaultCurrentLimit = 40;

    final String name;
    final int canId;
    final Controller controller;
    final List<Follower> followers = new ArrayList<>();
    final Gains gains = new Gains();
    final Map<String, Source> sources = new LinkedHashMap<>();
    final Map<String, DoubleConsumer> editors = new LinkedHashMap<>();
    Gains slot1;

    boolean inverted;
    boolean brake = true;
    int currentLimit = kDefaultCurrentLimit;
    double positionFactor = 1.0;
    double velocityFactor = 1.0 / 60.0;
    boolean maxMotion;
    double minOutput = -1.0;
    double maxOutput = 1.0;
    double reverseSoftLimit = Double.NaN;
    double forwardSoftLimit = Double.NaN;
    double startingPosition = Double.NaN;
    double simVelocityLagSeconds = 0.1;
    double selfTestPosition = Double.NaN;
    double tuneMin = Double.NaN;
    int uvwPeriodMs;
    int uvwDepth;
    int quadraturePeriodMs;
    int quadratureDepth;
    double tuneMax = Double.NaN;

    public MotorConfig(String name, int canId, Controller controller) {
        this.name = name;
        this.canId = canId;
        this.controller = controller;
    }

    public String name() {
        return name;
    }

    public MotorConfig follower(int canId, boolean inverted) {
        followers.add(new Follower(canId, inverted));
        return this;
    }

    public MotorConfig inverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    public MotorConfig coast() {
        brake = false;
        return this;
    }

    public static int safeCurrentLimit(double amps) {
        return amps >= 1.0 ? (int) Math.round(amps) : kDefaultCurrentLimit;
    }

    public MotorConfig currentLimit(int amps) {
        currentLimit = safeCurrentLimit(amps);
        track("currentLimit", "Current Limit");
        return this;
    }

    public MotorConfig conversion(double positionFactor, double velocityFactor) {
        this.positionFactor = positionFactor;
        this.velocityFactor = velocityFactor;
        return this;
    }

    public MotorConfig pid(double kP, double kI, double kD) {
        gains.kP = kP;
        gains.kI = kI;
        gains.kD = kD;
        track("pid", "kP", "kI", "kD");
        return this;
    }

    public MotorConfig feedforward(double kS, double kV, double kA) {
        gains.kS = kS;
        gains.kV = kV;
        gains.kA = kA;
        track("feedforward", "kS", "kV", "kA");
        return this;
    }

    public MotorConfig gravity(double kG) {
        gains.kG = kG;
        gains.gravityIsCosine = false;
        track("gravity", "kG");
        return this;
    }

    public MotorConfig cosineGravity(double kCos, double unitsPerRotation) {
        gains.kG = kCos;
        gains.gravityIsCosine = true;
        gains.unitsPerRotation = unitsPerRotation;
        track("cosineGravity", "kCos");
        return this;
    }

    public MotorConfig maxMotion(double maxAccel, double cruiseVel, double allowedError) {
        maxMotion = true;
        gains.maxAccel = maxAccel;
        gains.cruiseVel = cruiseVel;
        gains.allowedError = allowedError;
        track("maxMotion", "kMaxAccel", "kCruiseVel", "kDeviationErr");
        return this;
    }

    public MotorConfig slot1(Consumer<Gains> edit) {
        slot1 = gains.copy();
        edit.accept(slot1);
        return this;
    }

    public MotorConfig outputRange(double min, double max) {
        minOutput = min;
        maxOutput = max;
        return this;
    }

    public MotorConfig softLimits(double reverse, double forward) {
        reverseSoftLimit = reverse;
        forwardSoftLimit = forward;
        return this;
    }

    public MotorConfig tuneRange(double min, double max) {
        tuneMin = Math.min(min, max);
        tuneMax = Math.max(min, max);
        return this;
    }

    public double[] tuningRange() {
        if (!Double.isNaN(tuneMin) && !Double.isNaN(tuneMax)) {
            return new double[] {tuneMin, tuneMax};
        }
        if (!Double.isNaN(reverseSoftLimit) && !Double.isNaN(forwardSoftLimit)) {
            return new double[] {reverseSoftLimit, forwardSoftLimit};
        }
        return null;
    }

    public MotorConfig selfTestPosition(double position) {
        selfTestPosition = position;
        return this;
    }

    public double selfTestPosition() {
        if (!Double.isNaN(selfTestPosition)) {
            return selfTestPosition;
        }
        if (!Double.isNaN(reverseSoftLimit) && !Double.isNaN(forwardSoftLimit)) {
            return (reverseSoftLimit + forwardSoftLimit) / 2.0;
        }
        return Double.NaN;
    }

    public MotorConfig startingPosition(double position) {
        startingPosition = position;
        return this;
    }

    public MotorConfig uvwFilter(int periodMs, int depth) {
        uvwPeriodMs = periodMs;
        uvwDepth = depth;
        return this;
    }

    public MotorConfig quadratureFilter(int periodMs, int depth) {
        quadraturePeriodMs = periodMs;
        quadratureDepth = depth;
        return this;
    }

    public MotorConfig simVelocityLag(double seconds) {
        simVelocityLagSeconds = seconds;
        return this;
    }

    boolean tunable(String gain) {
        return sources.containsKey(gain);
    }

    boolean apply(String gain, double value) {
        DoubleConsumer editor = editors.get(gain);
        if (editor == null) {
            return false;
        }
        editor.accept(value);
        return true;
    }

    double value(String gain) {
        return switch (gain) {
            case "kP" -> gains.kP;
            case "kI" -> gains.kI;
            case "kD" -> gains.kD;
            case "kS" -> gains.kS;
            case "kV" -> gains.kV;
            case "kA" -> gains.kA;
            case "kG", "kCos" -> gains.kG;
            case "kMaxAccel" -> gains.maxAccel;
            case "kCruiseVel" -> gains.cruiseVel;
            case "kDeviationErr" -> gains.allowedError;
            case "Current Limit" -> currentLimit;
            default -> throw new IllegalArgumentException(gain);
        };
    }

    private void track(String call, String... names) {
        Source source = Source.caller(MotorConfig.class, call, call, 0);
        for (int i = 0; i < names.length; i++) {
            sources.put(names[i], source.withArgument(i));
        }
    }
}
