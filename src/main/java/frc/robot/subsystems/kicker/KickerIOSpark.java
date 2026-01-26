package frc.robot.subsystems.kicker;


import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.util.SparkUtil;

import com.revrobotics.spark.SparkLowLevel.MotorType;


public class KickerIOSpark implements KickerIO{
 
    // Hardware objects
    private final SparkMax kicker;

    private final RelativeEncoder kickerEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController kickerController;

    public KickerIOSpark(){

    kicker = new SparkMax(KickerConstants.kKickerCanID, MotorType.kBrushless);

    kickerEncoder = kicker.getEncoder();

    kickerController = kicker.getClosedLoopController();

    // Configure extention motor
    SparkMaxConfig kickerConfig = new SparkMaxConfig();
    kickerConfig
        .inverted(KickerConstants.kKickerinverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(KickerConstants.kKickerCurrentLimit)
        .voltageCompensation(12.0);
    kickerConfig
        .encoder
        .positionConversionFactor(KickerConstants.kKickerPositionConversionFactor)
        .velocityConversionFactor(KickerConstants.kKickerVelocityConversionFactor);
    kickerConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .pid(KickerConstants.kKickerP, KickerConstants.kKickerI, KickerConstants.kKickerD)
        .maxMotion
        .maxAcceleration(KickerConstants.kKickerMaxAccel)
        .cruiseVelocity(KickerConstants.kKickerCruiseVel)
        .allowedProfileError(KickerConstants.kKickerDeviationErr);
    kickerConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    kicker.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kicker.clearFaults();

    SparkUtil.tunePID(
        "Kicker",
        kicker,
        kickerConfig,
        new double[] {KickerConstants.kKickerP, KickerConstants.kKickerI, KickerConstants.kKickerD, 0,0,0,0, KickerConstants.kKickerMaxAccel, KickerConstants.kKickerCruiseVel, KickerConstants.kKickerDeviationErr},
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters,
        false,
        true);

    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        
    }

    @Override
    public void setKickerVoltage(double volts) {
        kicker.setVoltage(volts);
    }

    @Override
    public void setKickerSpeed(double speed) {
        kickerController.setSetpoint(speed, ControlType.kMAXMotionVelocityControl);
    }

    @Override
    public void stopKicker() {
        kicker.stopMotor();
    }
  }



