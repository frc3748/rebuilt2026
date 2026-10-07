package frc.robot.util.motor;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.util.SparkUtil;
import frc.robot.util.motor.MotorConfig.Controller;
import frc.robot.util.motor.MotorConfig.Follower;

public class MotorIOSpark implements MotorIO {
    private static final int kConfigAttempts = 5;
    private static final int kResetWarning = 0x40;
    private static final double kRecoverPeriodSeconds = 1.0;

    private final MotorConfig config;
    private final SparkBase motor;
    private final SparkBase[] followers;
    private final SparkBaseConfig[] followerConfigs;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;
    private final SparkBaseConfig sparkConfig;
    private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final boolean[] followerConfigured;
    private final boolean[] followerReachable;
    private final boolean[] followerResetHandled;
    private final Alert settingsAlert;
    private final Alert rebootAlert;

    private boolean configured;
    private boolean resetHandled = true;
    private double nextRecoverTime;
    private double cosineGravity;
    private double lastPosition;

    public MotorIOSpark(MotorConfig config) {
        this.config = config;
        motor = newSpark(config.controller, config.canId);
        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();
        sparkConfig = newConfig(config.controller);

        sparkConfig
                .inverted(config.inverted)
                .idleMode(config.brake ? IdleMode.kBrake : IdleMode.kCoast)
                .smartCurrentLimit(config.currentLimit)
                .voltageCompensation(12.0);
        sparkConfig.encoder
                .positionConversionFactor(config.positionFactor)
                .velocityConversionFactor(config.velocityFactor);
        if (config.uvwPeriodMs > 0) {
            sparkConfig.encoder.uvwMeasurementPeriod(config.uvwPeriodMs).uvwAverageDepth(config.uvwDepth);
        }
        if (config.quadraturePeriodMs > 0) {
            sparkConfig.encoder.quadratureMeasurementPeriod(config.quadraturePeriodMs).quadratureAverageDepth(config.quadratureDepth);
        }
        sparkConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .outputRange(config.minOutput, config.maxOutput);
        applyGains(config.gains, ClosedLoopSlot.kSlot0);
        if (config.slot1 != null) {
            applyGains(config.slot1, ClosedLoopSlot.kSlot1);
        }
        sparkConfig.closedLoop.feedForward
                .kS(config.gains.kS)
                .kV(config.gains.kV)
                .kA(config.gains.kA);
        if (config.gains.gravityIsCosine) {
            cosineGravity = config.gains.kG;
        } else {
            sparkConfig.closedLoop.feedForward.kG(config.gains.kG);
        }
        sparkConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        if (!Double.isNaN(config.forwardSoftLimit)) {
            sparkConfig.softLimit.forwardSoftLimit(config.forwardSoftLimit).forwardSoftLimitEnabled(true);
        }
        if (!Double.isNaN(config.reverseSoftLimit)) {
            sparkConfig.softLimit.reverseSoftLimit(config.reverseSoftLimit).reverseSoftLimitEnabled(true);
        }
        configured = tryUntilOk(motor, kConfigAttempts,
                () -> motor.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        motor.clearFaults();

        followers = new SparkBase[config.followers.size()];
        followerConfigs = new SparkBaseConfig[followers.length];
        followerConfigured = new boolean[followers.length];
        followerReachable = new boolean[followers.length];
        followerResetHandled = new boolean[followers.length];
        Arrays.fill(followerResetHandled, true);
        for (int i = 0; i < followers.length; i++) {
            Follower follower = config.followers.get(i);
            followers[i] = newSpark(config.controller, follower.canId());
            SparkBaseConfig followerConfig = newConfig(config.controller);
            followerConfig
                    .idleMode(config.brake ? IdleMode.kBrake : IdleMode.kCoast)
                    .smartCurrentLimit(config.currentLimit)
                    .voltageCompensation(12.0)
                    .follow(config.canId, follower.inverted());
            SparkBase spark = followers[i];
            followerConfigured[i] = tryUntilOk(spark, kConfigAttempts,
                    () -> spark.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
            followerConfigs[i] = followerConfig;
            followers[i].clearFaults();
        }

        if (!Double.isNaN(config.startingPosition)) {
            configured &= tryUntilOk(motor, kConfigAttempts, () -> encoder.setPosition(config.startingPosition));
        }
        settingsAlert = new Alert("Devices", config.name() + " motor didn't take its settings", AlertType.kError);
        rebootAlert = new Alert("Devices", config.name() + " motor rebooted, check its power and CAN wires", AlertType.kWarning);
        settingsAlert.set(!allConfigured());

        tune();
    }

    private void tune() {
        Map<String, DoubleConsumer> edits = new HashMap<>();
        edits.put("kP", apply(value -> sparkConfig.closedLoop.p(value)));
        edits.put("kI", apply(value -> sparkConfig.closedLoop.i(value)));
        edits.put("kD", apply(value -> sparkConfig.closedLoop.d(value)));
        edits.put("kS", apply(value -> sparkConfig.closedLoop.feedForward.kS(value)));
        edits.put("kV", apply(value -> sparkConfig.closedLoop.feedForward.kV(value)));
        edits.put("kA", apply(value -> sparkConfig.closedLoop.feedForward.kA(value)));
        edits.put("kG", apply(value -> sparkConfig.closedLoop.feedForward.kG(value)));
        edits.put("kCos", value -> cosineGravity = value);
        edits.put("kMaxAccel", apply(value -> sparkConfig.closedLoop.maxMotion.maxAcceleration(value)));
        edits.put("kCruiseVel", apply(value -> sparkConfig.closedLoop.maxMotion.cruiseVelocity(value)));
        edits.put("kDeviationErr", apply(value -> sparkConfig.closedLoop.maxMotion.allowedProfileError(value)));
        edits.put("Current Limit", value -> setCurrentLimit((int) value));
        MotorTuning.register(config, edits);
    }

    private DoubleConsumer apply(DoubleConsumer edit) {
        return value -> {
            edit.accept(value);
            motor.configureAsync(sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        };
    }

    private void applyGains(Gains gains, ClosedLoopSlot slot) {
        sparkConfig.closedLoop.pid(gains.kP, gains.kI, gains.kD, slot);
        if (config.maxMotion) {
            sparkConfig.closedLoop.maxMotion
                    .maxAcceleration(gains.maxAccel, slot)
                    .cruiseVelocity(gains.cruiseVel, slot)
                    .allowedProfileError(gains.allowedError, slot);
        }
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        SparkUtil.sparkStickyFault = false;
        ifOk(motor, encoder::getPosition, value -> inputs.position = value);
        ifOk(motor, encoder::getVelocity, value -> inputs.velocity = value);
        ifOk(motor,
                new DoubleSupplier[] { motor::getAppliedOutput, motor::getBusVoltage },
                values -> inputs.appliedVolts = values[0] * values[1]);
        ifOk(motor, motor::getOutputCurrent, value -> inputs.currentAmps = value);
        ifOk(motor, motor::getMotorTemperature, value -> inputs.tempCelsius = value);
        boolean reachable = !SparkUtil.sparkStickyFault;
        inputs.connected = connectedDebounce.calculate(reachable);
        inputs.faults = motor.getFaults().rawBits;
        inputs.stickyFaults = motor.getStickyFaults().rawBits;
        inputs.stickyWarnings = motor.getStickyWarnings().rawBits;
        lastPosition = inputs.position;

        if (inputs.followerAppliedVolts.length != followers.length) {
            inputs.followerAppliedVolts = new double[followers.length];
            inputs.followerCurrentAmps = new double[followers.length];
        }
        for (int i = 0; i < followers.length; i++) {
            SparkBase follower = followers[i];
            int index = i;
            SparkUtil.sparkStickyFault = false;
            ifOk(follower,
                    new DoubleSupplier[] { follower::getAppliedOutput, follower::getBusVoltage },
                    values -> inputs.followerAppliedVolts[index] = values[0] * values[1]);
            ifOk(follower, follower::getOutputCurrent, value -> inputs.followerCurrentAmps[index] = value);
            followerReachable[i] = !SparkUtil.sparkStickyFault;
        }

        double now = Timer.getFPGATimestamp();
        if (now >= nextRecoverTime) {
            nextRecoverTime = now + kRecoverPeriodSeconds;
            recover(reachable, inputs.stickyWarnings);
        }
    }

    private void recover(boolean reachable, int stickyWarnings) {
        if (reachable) {
            boolean reset = (stickyWarnings & kResetWarning) != 0;
            if (!configured || (reset && !resetHandled)) {
                motor.configureAsync(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
                if (!Double.isNaN(config.startingPosition)) {
                    encoder.setPosition(config.startingPosition);
                }
                rebootAlert.set(rebootAlert.get() || (configured && reset));
                configured = true;
            }
            if (reset) {
                motor.clearFaults();
            }
            resetHandled = reset;
        }
        for (int i = 0; i < followers.length; i++) {
            SparkBase follower = followers[i];
            if (!followerReachable[i]) {
                continue;
            }
            boolean reset = follower.getStickyWarnings().hasReset;
            if (!followerConfigured[i] || (reset && !followerResetHandled[i])) {
                follower.configureAsync(followerConfigs[i], ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
                rebootAlert.set(rebootAlert.get() || (followerConfigured[i] && reset));
                followerConfigured[i] = true;
            }
            if (reset) {
                follower.clearFaults();
            }
            followerResetHandled[i] = reset;
        }
        settingsAlert.set(!allConfigured());
    }

    private boolean allConfigured() {
        boolean all = configured;
        for (boolean follower : followerConfigured) {
            all &= follower;
        }
        return all;
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setOutput(double percent) {
        motor.set(percent);
    }

    @Override
    public void setVelocity(double velocity, double feedforwardVolts) {
        ControlType type = config.maxMotion ? ControlType.kMAXMotionVelocityControl : ControlType.kVelocity;
        controller.setSetpoint(velocity, type, ClosedLoopSlot.kSlot0, feedforwardVolts, ArbFFUnits.kVoltage);
    }

    @Override
    public void setPosition(double position, double feedforwardVolts, int slot) {
        ControlType type = config.maxMotion ? ControlType.kMAXMotionPositionControl : ControlType.kPosition;
        ClosedLoopSlot closedLoopSlot = slot == 1 ? ClosedLoopSlot.kSlot1 : ClosedLoopSlot.kSlot0;
        double gravity = cosineGravity * Math.cos(lastPosition / config.gains.unitsPerRotation * 2.0 * Math.PI);
        controller.setSetpoint(position, type, closedLoopSlot, feedforwardVolts + gravity, ArbFFUnits.kVoltage);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setEncoderPosition(double position) {
        encoder.setPosition(position);
    }

    @Override
    public void setCurrentLimit(int requested) {
        int amps = MotorConfig.safeCurrentLimit(requested);
        sparkConfig.smartCurrentLimit(amps);
        motor.configureAsync(sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        for (int i = 0; i < followers.length; i++) {
            followerConfigs[i].smartCurrentLimit(amps);
            followers[i].configureAsync(followerConfigs[i], ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    private static SparkBase newSpark(Controller controller, int canId) {
        return controller == Controller.SPARK_FLEX
                ? new SparkFlex(canId, MotorType.kBrushless)
                : new SparkMax(canId, MotorType.kBrushless);
    }

    private static SparkBaseConfig newConfig(Controller controller) {
        return controller == Controller.SPARK_FLEX ? new SparkFlexConfig() : new SparkMaxConfig();
    }
}
