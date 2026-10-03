package frc.robot.util.motor;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleConsumer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class MotorIOSim implements MotorIO {
    private enum Mode {
        OPEN_LOOP,
        VELOCITY,
        POSITION
    }

    private static final double kLoopSeconds = 0.02;
    private static final int kSubsteps = 20;
    private static final double kNominalVolts = 12.0;
    private static final double kDefaultFrictionVolts = 0.1;
    private static final double kStuckSpeed = 1e-6;

    private final MotorConfig config;
    private Mode mode = Mode.OPEN_LOOP;
    private double setpoint;
    private double position;
    private double velocity;
    private double appliedVolts;
    private double requestedVolts;
    private double feedforwardVolts;
    private int slot;
    private double referencePosition;
    private double referenceVelocity;
    private double lastError;
    private double currentAmps;
    private final double plantKs;
    private final double plantKv;
    private final double plantKa;
    private final double plantKg;
    private final boolean plantCosine;
    private final double plantUnitsPerRotation;
    private final double resistanceOhms;

    public MotorIOSim(MotorConfig config) {
        this.config = config;
        DCMotor motor = switch (config.controller) {
            case SPARK_FLEX -> DCMotor.getNeoVortex(1);
            case SPARK_MAX -> DCMotor.getNEO(1);
            case TALON_FX -> DCMotor.getKrakenX60(1);
        };
        double freeSpeed = Units.radiansPerSecondToRotationsPerMinute(motor.freeSpeedRadPerSec) * Math.abs(config.velocityFactor);
        Gains declared = config.gains;
        plantKv = motor.nominalVoltageVolts / freeSpeed;
        plantKa = plantKv * config.simVelocityLagSeconds;
        plantKs = declared.kS > 0.0 ? declared.kS : kDefaultFrictionVolts;
        plantKg = declared.kG;
        plantCosine = declared.gravityIsCosine;
        plantUnitsPerRotation = declared.unitsPerRotation;
        resistanceOhms = motor.rOhms;
        if (!Double.isNaN(config.startingPosition)) {
            position = config.startingPosition;
        }
        Gains gains = config.gains;
        Map<String, DoubleConsumer> edits = new HashMap<>();
        edits.put("kP", value -> gains.kP = value);
        edits.put("kI", value -> gains.kI = value);
        edits.put("kD", value -> gains.kD = value);
        edits.put("kS", value -> gains.kS = value);
        edits.put("kV", value -> gains.kV = value);
        edits.put("kA", value -> gains.kA = value);
        edits.put("kG", value -> gains.kG = value);
        edits.put("kCos", value -> gains.kG = value);
        edits.put("kMaxAccel", value -> gains.maxAccel = value);
        edits.put("kCruiseVel", value -> gains.cruiseVel = value);
        edits.put("kDeviationErr", value -> gains.allowedError = value);
        edits.put("Current Limit", value -> config.currentLimit = MotorConfig.safeCurrentLimit(value));
        MotorTuning.register(config, edits);
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.connected = true;
        double dt = kLoopSeconds / kSubsteps;
        for (int i = 0; i < kSubsteps; i++) {
            double volts = switch (mode) {
                case OPEN_LOOP -> requestedVolts;
                case VELOCITY -> velocityControl(dt);
                case POSITION -> positionControl(dt);
            };
            appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
            simulate(appliedVolts, dt);
        }
        currentAmps = Math.min(Math.abs(appliedVolts - plantKv * velocity) / resistanceOhms, config.currentLimit);

        inputs.position = position;
        inputs.velocity = velocity;
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = currentAmps;
    }

    private Gains gains() {
        return slot == 1 && config.slot1 != null ? config.slot1 : config.gains;
    }

    private double velocityControl(double dt) {
        Gains gains = gains();
        double previous = referenceVelocity;
        referenceVelocity = config.maxMotion && gains.maxAccel > 0.0
                ? referenceVelocity + MathUtil.clamp(setpoint - referenceVelocity, -gains.maxAccel * dt, gains.maxAccel * dt)
                : setpoint;
        double error = referenceVelocity - velocity;
        double feedback = kNominalVolts * (gains.kP * error + gains.kD * (error - lastError));
        lastError = error;
        double volts = feedback + gains.kS * Math.signum(referenceVelocity) + gains.kV * referenceVelocity
                + gains.kA * (referenceVelocity - previous) / dt + feedforwardVolts;
        return MathUtil.clamp(volts, config.minOutput * kNominalVolts, config.maxOutput * kNominalVolts);
    }

    private double positionControl(double dt) {
        Gains gains = gains();
        double previous = referenceVelocity;
        if (config.maxMotion && gains.cruiseVel > 0.0 && gains.maxAccel > 0.0) {
            TrapezoidProfile.State next = new TrapezoidProfile(new TrapezoidProfile.Constraints(gains.cruiseVel, gains.maxAccel))
                    .calculate(dt, new TrapezoidProfile.State(referencePosition, referenceVelocity), new TrapezoidProfile.State(setpoint, 0.0));
            referencePosition = next.position;
            referenceVelocity = next.velocity;
        } else {
            referencePosition = setpoint;
            referenceVelocity = 0.0;
        }
        double error = referencePosition - position;
        double feedback = kNominalVolts * (gains.kP * error + gains.kD * (error - lastError));
        lastError = error;
        double load = gains.gravityIsCosine ? gains.kG * Math.cos(position / gains.unitsPerRotation * 2.0 * Math.PI) : gains.kG;
        double volts = feedback + gains.kS * Math.signum(referenceVelocity) + gains.kV * referenceVelocity
                + gains.kA * (referenceVelocity - previous) / dt + load + feedforwardVolts;
        return MathUtil.clamp(volts, config.minOutput * kNominalVolts, config.maxOutput * kNominalVolts);
    }

    private void simulate(double volts, double dt) {
        double backEmf = plantKv * velocity;
        double limited = MathUtil.clamp(volts, backEmf - config.currentLimit * resistanceOhms,
                backEmf + config.currentLimit * resistanceOhms);
        double driving = limited - gravity(position);
        if (Math.abs(velocity) < kStuckSpeed && Math.abs(driving) <= plantKs) {
            velocity = 0.0;
            return;
        }
        double friction = plantKs * Math.signum(Math.abs(velocity) < kStuckSpeed ? driving : velocity);
        double acceleration = (driving - friction - plantKv * velocity) / plantKa;
        double next = velocity + acceleration * dt;
        velocity = Math.signum(next) != Math.signum(velocity) && Math.abs(velocity) >= kStuckSpeed ? 0.0 : next;
        position += velocity * dt;
        limit();
    }

    private double gravity(double at) {
        return plantCosine ? plantKg * Math.cos(at / plantUnitsPerRotation * 2.0 * Math.PI) : plantKg;
    }

    private void limit() {
        if (!Double.isNaN(config.reverseSoftLimit) && position < config.reverseSoftLimit) {
            position = config.reverseSoftLimit;
            velocity = Math.max(velocity, 0.0);
        }
        if (!Double.isNaN(config.forwardSoftLimit) && position > config.forwardSoftLimit) {
            position = config.forwardSoftLimit;
            velocity = Math.min(velocity, 0.0);
        }
    }

    @Override
    public void setVoltage(double volts) {
        mode = Mode.OPEN_LOOP;
        requestedVolts = volts;
    }

    @Override
    public void setOutput(double percent) {
        mode = Mode.OPEN_LOOP;
        requestedVolts = percent * kNominalVolts;
    }

    @Override
    public void setVelocity(double velocity, double feedforwardVolts) {
        if (mode != Mode.VELOCITY) {
            referenceVelocity = this.velocity;
            lastError = 0.0;
        }
        mode = Mode.VELOCITY;
        setpoint = velocity;
        this.feedforwardVolts = feedforwardVolts;
        slot = 0;
    }

    @Override
    public void setPosition(double position, double feedforwardVolts, int slot) {
        if (mode != Mode.POSITION || this.slot != slot) {
            referencePosition = this.position;
            referenceVelocity = velocity;
            lastError = 0.0;
        }
        mode = Mode.POSITION;
        setpoint = position;
        this.feedforwardVolts = feedforwardVolts;
        this.slot = slot;
    }

    @Override
    public void stop() {
        mode = Mode.OPEN_LOOP;
        requestedVolts = 0.0;
    }

    @Override
    public void setEncoderPosition(double position) {
        this.position = position;
    }
}
