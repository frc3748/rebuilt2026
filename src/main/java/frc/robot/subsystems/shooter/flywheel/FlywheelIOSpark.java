package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.ctre.phoenix6.hardware.CANcoder;
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


public class FlywheelIOSpark implements FlywheelIO{
 
    // Hardware objects
    private final SparkMax flywheel;

    private final RelativeEncoder flywheelEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController flywheelController;

    public FlywheelIOSpark(){

    flywheel = new SparkMax(0, MotorType.kBrushless);

    flywheelEncoder = flywheel.getEncoder();

    flywheelController = flywheel.getClosedLoopController();

    // Configure flywheel motor
    // NEED TO CONFIGURE
    SparkMaxConfig flywheelConfig = new SparkMaxConfig();
    flywheelConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(10)
        .voltageCompensation(12.0);
    flywheelConfig
        .encoder
        .positionConversionFactor(0)
        .velocityConversionFactor(0);
    flywheelConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .pid(0, 0, 0)
        .maxMotion
        .maxAcceleration(0)
        .cruiseVelocity(0)
        .allowedProfileError(0);
    flywheelConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    // turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flywheel.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flywheel.clearFaults();


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
        flywheelController.setSetpoint(speed, ControlType.kVelocity);
    }

    @Override
    public void stopFlywheel() {
        flywheel.stopMotor();
    }


    
}
