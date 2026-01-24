package frc.robot.subsystems.intake;


import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.util.SparkUtil;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

@SuppressWarnings("unused")
public class IntakeIOSpark implements IntakeIO {

    // Hardware objects
    private final SparkFlex rollers;
    private final SparkMax extension;

    private final RelativeEncoder rollerEncoder;
    private final RelativeEncoder extensionEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController rollerController;
    private final SparkClosedLoopController extensionController;

    public IntakeIOSpark() {

        rollers = new SparkFlex(IntakeConstants.kRollersCanID, MotorType.kBrushless);
        extension = new SparkMax(IntakeConstants.kExtensionCanID, MotorType.kBrushless);

        rollerEncoder = rollers.getEncoder();
        extensionEncoder = extension.getEncoder();

        rollerController = rollers.getClosedLoopController();
        extensionController = extension.getClosedLoopController();

        // Configure roller motor
        SparkFlexConfig rollerConfig = new SparkFlexConfig();
        rollerConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(10)
                .voltageCompensation(12.0);
        rollerConfig.encoder
                .positionConversionFactor(IntakeConstants.kRollerPositionConversionFactor)
                .velocityConversionFactor(IntakeConstants.kRollerVelocityConversionFactor)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        rollerConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(IntakeConstants.kRollerP, IntakeConstants.kRollerI, IntakeConstants.kRollerD)
                .iMaxAccum(0.01); // change if needed
        rollerConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        rollers.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollers.clearFaults();
        // Configure extension motor
        SparkMaxConfig extensionConfig = new SparkMaxConfig();
        extensionConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(10)
                .voltageCompensation(12.0);
        extensionConfig.encoder
                .positionConversionFactor(IntakeConstants.kExtensionPositionConversionFactor)
                .velocityConversionFactor(IntakeConstants.kExtensionVelocityConversionFactor);
        extensionConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(true)
                .pid(IntakeConstants.kExtensionP, IntakeConstants.kExtensionI, IntakeConstants.kExtensionD)
                .maxMotion
                .maxAcceleration(IntakeConstants.kExtensionMaxAccel)
                .cruiseVelocity(IntakeConstants.kExtensionCruiseVel)
                .allowedProfileError(IntakeConstants.kExtensionDeviationErr);
        extensionConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        // turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters,
        // PersistMode.kPersistParameters);
        extension.configure(extensionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        extension.clearFaults();

        SparkUtil.tunePID(
                "Intake Roller",
                rollers,
                rollerConfig,
                new double[] { IntakeConstants.kRollerP, IntakeConstants.kRollerI, IntakeConstants.kRollerD },
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters,
                false,
                false);

        SparkUtil.tunePID(
            "Intake Extension",
            extension,
            extensionConfig,
            new double[] {IntakeConstants.kExtensionP, IntakeConstants.kExtensionI, IntakeConstants.kExtensionD, 0,0,0,0, IntakeConstants.kExtensionMaxAccel, IntakeConstants.kExtensionCruiseVel, IntakeConstants.kExtensionDeviationErr},
            ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters,
                false,
                true
        );
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

    }

    @Override
    public void setRollerVoltage(double volts) {
        rollers.setVoltage(volts);
    }

    @Override
    public void setRollerSpeed(double speed) {
        rollerController.setSetpoint(speed, ControlType.kVelocity);
    }

    @Override
    public void stopRollers() {
        rollers.stopMotor();
    }

    @Override
    public void setExtensionPosition(double position) {
        extensionController.setSetpoint(position, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void stopExtension() {
        rollers.stopMotor();
    }
}
