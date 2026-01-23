package frc.robot.subsystems.shooter.hood;

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

import frc.robot.subsystems.climb.ClimbIO.ClimbIOInputs;
import frc.robot.subsystems.drive.ModuleIO;

public class HoodIOSpark implements HoodIO{
 
    // Hardware objects
    private final SparkMax hood;

    private final RelativeEncoder hoodEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController hoodController;

    public HoodIOSpark(){

    hood = new SparkMax(0, MotorType.kBrushless);

    hoodEncoder = hood.getEncoder();

    hoodController = hood.getClosedLoopController();

    // Configure hood motor
    // NEED TO CONFIGURE
    SparkMaxConfig hoodConfig = new SparkMaxConfig();
    hoodConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(10)
        .voltageCompensation(12.0);
    hoodConfig
        .encoder
        .positionConversionFactor(0)
        .velocityConversionFactor(0);
    hoodConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .pid(0, 0, 0)
        .maxMotion
        .maxAcceleration(0)
        .cruiseVelocity(0)
        .allowedProfileError(0);
    hoodConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    // turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hood.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hood.clearFaults();


    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        
    }

    @Override
    public void setHoodVoltage(double volts) {
        hood.setVoltage(volts);
    }

    @Override
    public void setHoodPosition(double position) {
        hoodController.setSetpoint(position, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void stopHood() {
        hood.stopMotor();
    }


    
    
}
