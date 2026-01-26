package frc.robot.subsystems.shooter.flywheel;


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


public class FlywheelIOSpark implements FlywheelIO{
 
    // Hardware objects
    private final SparkMax flywheel;

    private final RelativeEncoder flywheelEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController flywheelController;

    public FlywheelIOSpark(){

    flywheel = new SparkMax(FlywheelConstants.kFlywheelCanID, MotorType.kBrushless);

    flywheelEncoder = flywheel.getEncoder();

    flywheelController = flywheel.getClosedLoopController();

    // Configure extention motor
    SparkMaxConfig flywheelConfig = new SparkMaxConfig();
    flywheelConfig
        .inverted(FlywheelConstants.kFlywheelinverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(FlywheelConstants.kFlywheelCurrentLimit)
        .voltageCompensation(12.0);
    flywheelConfig
        .encoder
        .positionConversionFactor(FlywheelConstants.kFlywheelPositionConversionFactor)
        .velocityConversionFactor(FlywheelConstants.kFlywheelVelocityConversionFactor);
    flywheelConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .pid(FlywheelConstants.kFlywheelP, FlywheelConstants.kFlywheelI, FlywheelConstants.kFlywheelD)
        .maxMotion
        .maxAcceleration(FlywheelConstants.kFlywheelMaxAccel)
        .cruiseVelocity(FlywheelConstants.kFlywheelCruiseVel)
        .allowedProfileError(FlywheelConstants.kFlywheelDeviationErr);
    flywheelConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    flywheel.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flywheel.clearFaults();

    SparkUtil.tunePID(
        "Flywheel",
        flywheel,
        flywheelConfig,
        new double[] {FlywheelConstants.kFlywheelP, FlywheelConstants.kFlywheelI, FlywheelConstants.kFlywheelD, 0,0,0,0, FlywheelConstants.kFlywheelMaxAccel, FlywheelConstants.kFlywheelCruiseVel, FlywheelConstants.kFlywheelDeviationErr},
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters,
        false,
        true);

    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        
    }

    @Override
    public void setFlywheelVoltage(double volts) {
        flywheel.setVoltage(volts);
    }

    @Override
    public void setFlywheelSpeed(double speed) {
        flywheelController.setSetpoint(speed, ControlType.kMAXMotionVelocityControl);
    }

    @Override
    public void stopFlywheel() {
        flywheel.stopMotor();
    }
  }



