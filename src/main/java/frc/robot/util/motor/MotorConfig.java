package frc.robot.util.motor;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class MotorConfig {
    public enum Controller {
        SPARK_MAX,
        SPARK_FLEX,
        TALON_FX
    }

    public record Follower(int canId, boolean inverted) {}

    final String name;
    final int canId;
    final Controller controller;
    final List<Follower> followers = new ArrayList<>();
    final Gains gains = new Gains();
    Gains slot1;

    boolean inverted;
    boolean brake = true;
    int currentLimit = 40;
    double positionFactor = 1.0;
    double velocityFactor = 1.0 / 60.0;
    boolean maxMotion;
    double minOutput = -1.0;
    double maxOutput = 1.0;
    double reverseSoftLimit = Double.NaN;
    double forwardSoftLimit = Double.NaN;
    double startingPosition = Double.NaN;
    boolean tunable;
    boolean tuneFeedforward;
    boolean tuneMaxMotion;
    double simVelocityLagSeconds = 0.1;

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

    public MotorConfig currentLimit(int amps) {
        currentLimit = amps;
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
        return this;
    }

    public MotorConfig feedforward(double kS, double kV, double kA) {
        gains.kS = kS;
        gains.kV = kV;
        gains.kA = kA;
        return this;
    }

    public MotorConfig gravity(double kG) {
        gains.kG = kG;
        gains.gravityIsCosine = false;
        return this;
    }

    public MotorConfig cosineGravity(double kCos) {
        gains.kG = kCos;
        gains.gravityIsCosine = true;
        return this;
    }

    public MotorConfig maxMotion(double maxAccel, double cruiseVel, double allowedError) {
        maxMotion = true;
        gains.maxAccel = maxAccel;
        gains.cruiseVel = cruiseVel;
        gains.allowedError = allowedError;
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

    public MotorConfig startingPosition(double position) {
        startingPosition = position;
        return this;
    }

    public MotorConfig tunable(boolean feedforward, boolean maxMotion) {
        tunable = true;
        tuneFeedforward = feedforward;
        tuneMaxMotion = maxMotion;
        return this;
    }

    public MotorConfig simVelocityLag(double seconds) {
        simVelocityLagSeconds = seconds;
        return this;
    }
}
