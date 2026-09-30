package frc.robot.util.motor;

import static frc.robot.util.SparkUtil.ifOk;

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

import frc.robot.util.SparkUtil;
import frc.robot.util.motor.MotorConfig.Controller;
import frc.robot.util.motor.MotorConfig.Follower;

public class MotorIOSpark implements MotorIO {
    private final MotorConfig config;
    private final SparkBase motor;
    private final SparkBase[] followers;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;
    private final SparkBaseConfig sparkConfig;

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
            sparkConfig.closedLoop.feedForward.kCos(config.gains.kG);
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
        motor.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor.clearFaults();

        followers = new SparkBase[config.followers.size()];
        for (int i = 0; i < followers.length; i++) {
            Follower follower = config.followers.get(i);
            followers[i] = newSpark(config.controller, follower.canId());
            SparkBaseConfig followerConfig = newConfig(config.controller);
            followerConfig
                    .idleMode(config.brake ? IdleMode.kBrake : IdleMode.kCoast)
                    .smartCurrentLimit(config.currentLimit)
                    .voltageCompensation(12.0)
                    .follow(config.canId, follower.inverted());
            followers[i].configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            followers[i].clearFaults();
        }

        if (!Double.isNaN(config.startingPosition)) {
            encoder.setPosition(config.startingPosition);
        }

        if (config.tunable) {
            SparkUtil.tune(config.name, motor, sparkConfig, config.gains, config.tuneFeedforward, config.tuneMaxMotion);
        }
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
        ifOk(motor, encoder::getPosition, value -> inputs.position = value);
        ifOk(motor, encoder::getVelocity, value -> inputs.velocity = value);
        ifOk(motor,
                new DoubleSupplier[] { motor::getAppliedOutput, motor::getBusVoltage },
                values -> inputs.appliedVolts = values[0] * values[1]);
        ifOk(motor, motor::getOutputCurrent, value -> inputs.currentAmps = value);

        if (inputs.followerAppliedVolts.length != followers.length) {
            inputs.followerAppliedVolts = new double[followers.length];
            inputs.followerCurrentAmps = new double[followers.length];
        }
        for (int i = 0; i < followers.length; i++) {
            SparkBase follower = followers[i];
            int index = i;
            ifOk(follower,
                    new DoubleSupplier[] { follower::getAppliedOutput, follower::getBusVoltage },
                    values -> inputs.followerAppliedVolts[index] = values[0] * values[1]);
            ifOk(follower, follower::getOutputCurrent, value -> inputs.followerCurrentAmps[index] = value);
        }
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
        controller.setSetpoint(position, type, closedLoopSlot, feedforwardVolts, ArbFFUnits.kVoltage);
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
    public void setCurrentLimit(int amps) {
        sparkConfig.smartCurrentLimit(amps);
        motor.configure(sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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
