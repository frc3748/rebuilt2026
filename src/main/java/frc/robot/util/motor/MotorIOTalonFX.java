package frc.robot.util.motor;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class MotorIOTalonFX implements MotorIO {
    private final MotorConfig config;
    private final double velocityScale;
    private final TalonFX motor;
    private final TalonFX[] followers;
    private final SlotConfigs slot0;
    private final MotionMagicConfigs motionMagic = new MotionMagicConfigs();

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> currentAmps;
    private final StatusSignal<Temperature> temperature;
    private final BaseStatusSignal[] followerVolts;
    private final BaseStatusSignal[] followerCurrents;
    private final BaseStatusSignal[] signals;
    private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final DutyCycleOut outputRequest = new DutyCycleOut(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final MotionMagicVelocityVoltage profiledVelocityRequest = new MotionMagicVelocityVoltage(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final MotionMagicVoltage profiledPositionRequest = new MotionMagicVoltage(0);
    private final NeutralOut neutralRequest = new NeutralOut();

    public MotorIOTalonFX(MotorConfig config) {
        this.config = config;
        velocityScale = 60.0 * config.velocityFactor / config.positionFactor;
        motor = new TalonFX(config.canId);

        TalonFXConfiguration talonConfig = baseConfig(config);
        talonConfig.MotorOutput.Inverted = config.inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        talonConfig.MotorOutput.PeakForwardDutyCycle = config.maxOutput;
        talonConfig.MotorOutput.PeakReverseDutyCycle = config.minOutput;
        talonConfig.Voltage.PeakForwardVoltage = 12.0 * config.maxOutput;
        talonConfig.Voltage.PeakReverseVoltage = 12.0 * config.minOutput;
        talonConfig.Feedback.SensorToMechanismRatio = 1.0 / config.positionFactor;
        if (!Double.isNaN(config.forwardSoftLimit)) {
            talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = config.forwardSoftLimit;
            talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        }
        if (!Double.isNaN(config.reverseSoftLimit)) {
            talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = config.reverseSoftLimit;
            talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        }
        tryUntilOk(() -> motor.getConfigurator().apply(talonConfig));

        slot0 = slotConfigs(0, config.gains);
        tryUntilOk(() -> motor.getConfigurator().apply(slot0));
        if (config.slot1 != null) {
            SlotConfigs slot1 = slotConfigs(1, config.slot1);
            tryUntilOk(() -> motor.getConfigurator().apply(slot1));
        }
        if (config.maxMotion) {
            motionMagic.MotionMagicCruiseVelocity = config.gains.cruiseVel / velocityScale;
            motionMagic.MotionMagicAcceleration = config.gains.maxAccel / velocityScale;
            tryUntilOk(() -> motor.getConfigurator().apply(motionMagic));
        }

        followers = new TalonFX[config.followers.size()];
        followerVolts = new BaseStatusSignal[followers.length];
        followerCurrents = new BaseStatusSignal[followers.length];
        for (int i = 0; i < followers.length; i++) {
            MotorConfig.Follower follower = config.followers.get(i);
            TalonFX talon = new TalonFX(follower.canId());
            tryUntilOk(() -> talon.getConfigurator().apply(baseConfig(config)));
            talon.setControl(new Follower(config.canId,
                    follower.inverted() ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
            followers[i] = talon;
            followerVolts[i] = talon.getMotorVoltage();
            followerCurrents[i] = talon.getStatorCurrent();
        }

        if (!Double.isNaN(config.startingPosition)) {
            motor.setPosition(config.startingPosition);
        }

        position = motor.getPosition();
        velocity = motor.getVelocity();
        appliedVolts = motor.getMotorVoltage();
        currentAmps = motor.getStatorCurrent();
        temperature = motor.getDeviceTemp();
        signals = new BaseStatusSignal[5 + 2 * followers.length];
        signals[0] = position;
        signals[1] = velocity;
        signals[2] = appliedVolts;
        signals[3] = currentAmps;
        signals[4] = temperature;
        System.arraycopy(followerVolts, 0, signals, 5, followers.length);
        System.arraycopy(followerCurrents, 0, signals, 5 + followers.length, followers.length);
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, signals);
        motor.optimizeBusUtilization();
        for (TalonFX talon : followers) {
            talon.optimizeBusUtilization();
        }

        tune();
    }

    private static TalonFXConfiguration baseConfig(MotorConfig config) {
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        talonConfig.MotorOutput.NeutralMode = config.brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        talonConfig.CurrentLimits.StatorCurrentLimit = config.currentLimit;
        talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        return talonConfig;
    }

    private static SlotConfigs slotConfigs(int slot, Gains gains) {
        SlotConfigs slotConfigs = new SlotConfigs();
        slotConfigs.SlotNumber = slot;
        slotConfigs.kP = gains.kP;
        slotConfigs.kI = gains.kI;
        slotConfigs.kD = gains.kD;
        slotConfigs.kS = gains.kS;
        slotConfigs.kV = gains.kV;
        slotConfigs.kA = gains.kA;
        slotConfigs.kG = gains.kG;
        slotConfigs.GravityType = gains.gravityIsCosine ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static;
        return slotConfigs;
    }

    private void tune() {
        Runnable applySlot = () -> motor.getConfigurator().apply(slot0);
        Runnable applyMotionMagic = () -> motor.getConfigurator().apply(motionMagic);
        Map<String, DoubleConsumer> edits = new HashMap<>();
        edits.put("kP", then(value -> slot0.kP = value, applySlot));
        edits.put("kI", then(value -> slot0.kI = value, applySlot));
        edits.put("kD", then(value -> slot0.kD = value, applySlot));
        edits.put("kS", then(value -> slot0.kS = value, applySlot));
        edits.put("kV", then(value -> slot0.kV = value, applySlot));
        edits.put("kA", then(value -> slot0.kA = value, applySlot));
        edits.put("kG", then(value -> slot0.kG = value, applySlot));
        edits.put("kCos", then(value -> slot0.kG = value, applySlot));
        edits.put("kMaxAccel", then(value -> motionMagic.MotionMagicAcceleration = value / velocityScale, applyMotionMagic));
        edits.put("kCruiseVel", then(value -> motionMagic.MotionMagicCruiseVelocity = value / velocityScale, applyMotionMagic));
        edits.put("Current Limit", value -> setCurrentLimit((int) value));
        MotorTuning.register(config, edits);
    }

    private static DoubleConsumer then(DoubleConsumer edit, Runnable apply) {
        return value -> {
            edit.accept(value);
            apply.run();
        };
    }

    private static void tryUntilOk(Supplier<StatusCode> command) {
        for (int attempt = 0; attempt < 5; attempt++) {
            if (command.get().isOK()) {
                return;
            }
        }
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.connected = connectedDebounce.calculate(BaseStatusSignal.refreshAll(signals).isOK());
        inputs.faults = motor.getFaultField().refresh().getValue();
        inputs.stickyFaults = motor.getStickyFaultField().refresh().getValue();
        inputs.position = position.getValueAsDouble();
        inputs.velocity = velocity.getValueAsDouble() * velocityScale;
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = currentAmps.getValueAsDouble();
        inputs.tempCelsius = temperature.getValueAsDouble();

        if (inputs.followerAppliedVolts.length != followers.length) {
            inputs.followerAppliedVolts = new double[followers.length];
            inputs.followerCurrentAmps = new double[followers.length];
        }
        for (int i = 0; i < followers.length; i++) {
            inputs.followerAppliedVolts[i] = followerVolts[i].getValueAsDouble();
            inputs.followerCurrentAmps[i] = followerCurrents[i].getValueAsDouble();
        }
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void setOutput(double percent) {
        motor.setControl(outputRequest.withOutput(percent));
    }

    @Override
    public void setVelocity(double velocity, double feedforwardVolts) {
        double mechanismVelocity = velocity / velocityScale;
        if (config.maxMotion) {
            motor.setControl(profiledVelocityRequest.withVelocity(mechanismVelocity).withFeedForward(feedforwardVolts));
        } else {
            motor.setControl(velocityRequest.withVelocity(mechanismVelocity).withFeedForward(feedforwardVolts));
        }
    }

    @Override
    public void setPosition(double position, double feedforwardVolts, int slot) {
        if (config.maxMotion) {
            motor.setControl(profiledPositionRequest.withPosition(position).withFeedForward(feedforwardVolts).withSlot(slot));
        } else {
            motor.setControl(positionRequest.withPosition(position).withFeedForward(feedforwardVolts).withSlot(slot));
        }
    }

    @Override
    public void stop() {
        motor.setControl(neutralRequest);
    }

    @Override
    public void setEncoderPosition(double position) {
        motor.setPosition(position);
    }

    @Override
    public void setCurrentLimit(int amps) {
        CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
        limits.StatorCurrentLimit = amps;
        limits.StatorCurrentLimitEnable = true;
        motor.getConfigurator().apply(limits);
    }
}
