package frc.robot.subsystems.kicker;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class KickerIOSpark implements KickerIO{
 
    // Hardware objects
    private final SparkMax kicker;

    private final RelativeEncoder kickerEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController kickerController;

    public ClimbIOSpark(){

    kicker = new SparkMax(0, MotorType.kBrushless);

    kickerEncoder = kicker.getEncoder();

    climbController = kicker.getClosedLoopController();

    // Configure extention motor
    SparkMaxConfig kickerConfig = new SparkMaxConfig();
    kickerConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(10)
        .voltageCompensation(12.0);
    kickerConfig
        .encoder
        .positionConversionFactor(0)
        .velocityConversionFactor(0);
    kickerConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .pid(0, 0, 0)
        .maxMotion
        .maxAcceleration(0)
        .cruiseVelocity(0)
        .allowedProfileError(0);
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

    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        
    }

    @Override
    public void setKickerVoltage(double volts) {
        climb.setVoltage(volts);
    }

    @Override
    public void setKickerSpeed(double position) {
        climbController.setSetpoint(position, ControlType.kVelocity);
    }

    @Override
    public void stopKicker() {
        climb.stopMotor();
    }
  }



