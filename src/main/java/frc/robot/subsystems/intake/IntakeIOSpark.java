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
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class IntakeIOSpark implements IntakeIO{
 
    // Hardware objects
    private final SparkFlex rollers;
    private final SparkMax extention;

    private final RelativeEncoder rollerEncoder;
    private final RelativeEncoder extentionEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController rollerController;
    private final SparkClosedLoopController extentionController;

    public IntakeIOSpark(){

    rollers = new SparkFlex(0, MotorType.kBrushless);
    extention = new SparkMax(0, MotorType.kBrushless);

    rollerEncoder = rollers.getEncoder();
    extentionEncoder = extention.getEncoder();

    rollerController = rollers.getClosedLoopController();
    extentionController = extention.getClosedLoopController();

    // Configure roller motor
    SparkFlexConfig rollerConfig = new SparkFlexConfig();
    rollerConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(10)
        .voltageCompensation(12.0);
    rollerConfig
        .encoder
        .positionConversionFactor(0)
        .velocityConversionFactor(0)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    rollerConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0,0,0)
        .iMaxAccum(0);
    rollerConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    rollers.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rollers.clearFaults();


    // Configure extention motor
    SparkMaxConfig extentionConfig = new SparkMaxConfig();
    extentionConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(10)
        .voltageCompensation(12.0);
    extentionConfig
        .encoder
        .positionConversionFactor(0)
        .velocityConversionFactor(0);
    extentionConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .pid(0, 0, 0)
        .maxMotion
        .maxAcceleration(0)
        .cruiseVelocity(0)
        .allowedProfileError(0);
    extentionConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    // turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    extention.configure(extentionConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    extention.clearFaults();

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
    public void setExtentionPosition(double position) {
        extentionController.setSetpoint(position, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void stopExtention() {
        rollers.stopMotor();
    }
  }



