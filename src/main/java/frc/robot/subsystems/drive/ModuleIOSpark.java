package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.AbsoluteEncoder;
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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.DriveConfig.ModuleConstants;
import frc.robot.subsystems.drive.DriveConfig.TurnSensor;
import frc.robot.util.SparkUtil;
import frc.robot.util.motor.Gains;
import frc.robot.util.motor.MotorConfig.Controller;

public class ModuleIOSpark implements ModuleIO {
    private static final double kTurnMin = 0;
    private static final double kTurnMax = 2 * Math.PI;

    private final DriveConfig config;
    private final Rotation2d zeroRotation;
    private final SparkBase driveSpark;
    private final SparkMax turnSpark;
    private final RelativeEncoder driveEncoder;
    private final DoubleSupplier turnPosition;
    private final DoubleSupplier turnVelocity;
    private final CANcoder canCoder;
    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController turnController;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    private final Debouncer driveConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    public ModuleIOSpark(DriveConfig config, int index) {
        this.config = config;
        ModuleConstants module = config.module(index);
        boolean useCanCoder = config.turnSensor == TurnSensor.CANCODER;
        if (config.driveController == Controller.TALON_FX) {
            throw new IllegalArgumentException("ModuleIOSpark drives Spark motors; Kraken swerve needs its own ModuleIO");
        }

        driveSpark = config.driveController == Controller.SPARK_FLEX
                ? new SparkFlex(module.driveCanId(), MotorType.kBrushless)
                : new SparkMax(module.driveCanId(), MotorType.kBrushless);
        turnSpark = new SparkMax(module.turnCanId(), MotorType.kBrushless);
        driveEncoder = driveSpark.getEncoder();
        driveController = driveSpark.getClosedLoopController();
        turnController = turnSpark.getClosedLoopController();

        SparkBaseConfig driveConfig = config.driveController == Controller.SPARK_FLEX ? new SparkFlexConfig() : new SparkMaxConfig();
        driveConfig
                .inverted(module.driveInverted())
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(config.driveCurrentLimit)
                .voltageCompensation(12.0);
        driveConfig.encoder
                .positionConversionFactor(config.driveEncoderPositionFactor())
                .velocityConversionFactor(config.driveEncoderPositionFactor() / 60.0)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        driveConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(config.driveKp, config.driveKi, config.driveKd)
                .iMaxAccum(config.driveIntegrationCap);
        driveConfig.closedLoop.feedForward.kV(config.driveSparkKv);
        driveConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / config.odometryFrequency))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        driveSpark.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveSpark.clearFaults();
        tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

        SparkMaxConfig turnConfig = new SparkMaxConfig();
        turnConfig
                .inverted(config.turnInverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(config.turnCurrentLimit)
                .voltageCompensation(12.0);
        turnConfig.closedLoop
                .feedbackSensor(useCanCoder ? FeedbackSensor.kPrimaryEncoder : FeedbackSensor.kAbsoluteEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(kTurnMin, kTurnMax)
                .pid(config.turnKp, config.turnKi, config.turnKd);
        turnConfig.closedLoop.feedForward.kV(config.turnKv);
        turnConfig.signals
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        if (useCanCoder) {
            CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
            canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
            canCoderConfig.MagnetSensor.withMagnetOffset(module.zeroRotation().getRotations());
            canCoder = new CANcoder(module.canCoderId());
            canCoder.getConfigurator().apply(canCoderConfig);
            zeroRotation = Rotation2d.kZero;

            turnConfig.encoder
                    .positionConversionFactor(config.turnEncoderPositionFactor())
                    .velocityConversionFactor(config.turnEncoderPositionFactor() / 60.0);
            turnConfig.signals
                    .primaryEncoderPositionAlwaysOn(true)
                    .primaryEncoderPositionPeriodMs((int) (1000.0 / config.odometryFrequency))
                    .primaryEncoderVelocityAlwaysOn(true)
                    .primaryEncoderVelocityPeriodMs(20);
            turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            RelativeEncoder relative = turnSpark.getEncoder();
            tryUntilOk(turnSpark, 5, () -> relative.setPosition(canCoder.getAbsolutePosition().getValue().in(Radians)));
            turnPosition = relative::getPosition;
            turnVelocity = relative::getVelocity;
        } else {
            canCoder = null;
            zeroRotation = module.zeroRotation();

            turnConfig.absoluteEncoder
                    .inverted(config.turnEncoderInverted)
                    .positionConversionFactor(2 * Math.PI)
                    .velocityConversionFactor(2 * Math.PI / 60.0);
            turnConfig.signals
                    .absoluteEncoderPositionAlwaysOn(true)
                    .absoluteEncoderPositionPeriodMs((int) (1000.0 / config.odometryFrequency))
                    .absoluteEncoderVelocityAlwaysOn(true)
                    .absoluteEncoderVelocityPeriodMs(20);
            turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            AbsoluteEncoder absolute = turnSpark.getAbsoluteEncoder();
            turnPosition = absolute::getPosition;
            turnVelocity = absolute::getVelocity;
        }
        turnSpark.clearFaults();

        SparkUtil.tune("Drive PID", driveSpark, driveConfig,
                Gains.of(config.driveKp, config.driveKi, config.driveKd).withFeedforward(config.driveKs, config.driveKv, 0),
                true, false);
        SparkUtil.tune("Turn PID", turnSpark, turnConfig,
                Gains.of(config.turnKp, config.turnKi, config.turnKd).withFeedforward(0, config.turnKv, 0),
                true, false);

        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
        turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(turnSpark, turnPosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        sparkStickyFault = false;
        ifOk(driveSpark, driveEncoder::getPosition, value -> inputs.drivePositionRad = value);
        ifOk(driveSpark, driveEncoder::getVelocity, value -> inputs.driveVelocityRadPerSec = value);
        ifOk(driveSpark,
                new DoubleSupplier[] { driveSpark::getAppliedOutput, driveSpark::getBusVoltage },
                values -> inputs.driveAppliedVolts = values[0] * values[1]);
        ifOk(driveSpark, driveSpark::getOutputCurrent, value -> inputs.driveCurrentAmps = value);
        ifOk(driveSpark, driveSpark::getMotorTemperature, value -> inputs.driveTempCelsius = value);
        inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

        sparkStickyFault = false;
        ifOk(turnSpark, turnPosition, value -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
        ifOk(turnSpark, turnVelocity, value -> inputs.turnVelocityRadPerSec = value);
        ifOk(turnSpark,
                new DoubleSupplier[] { turnSpark::getAppliedOutput, turnSpark::getBusVoltage },
                values -> inputs.turnAppliedVolts = values[0] * values[1]);
        ifOk(turnSpark, turnSpark::getOutputCurrent, value -> inputs.turnCurrentAmps = value);
        ifOk(turnSpark, turnSpark::getMotorTemperature, value -> inputs.turnTempCelsius = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        if (canCoder != null) {
            inputs.canPosition = new Rotation2d(canCoder.getAbsolutePosition().getValue().in(Radians));
        }
        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.odometryTurnPositions = turnPositionQueue.stream()
                .map(value -> new Rotation2d(value).minus(zeroRotation))
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveSpark.setVoltage(output);
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnSpark.setVoltage(output);
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        double feedforward = config.driveKs * Math.signum(velocityRadPerSec) + config.driveKv * velocityRadPerSec;
        driveController.setSetpoint(velocityRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward,
                ArbFFUnits.kVoltage);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        double setpoint = MathUtil.inputModulus(rotation.plus(zeroRotation).getRadians(), kTurnMin, kTurnMax);
        turnController.setSetpoint(setpoint, ControlType.kPosition);
    }
}
