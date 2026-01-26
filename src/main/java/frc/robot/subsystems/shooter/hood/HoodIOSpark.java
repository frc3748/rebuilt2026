package frc.robot.subsystems.shooter.hood;


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


import frc.robot.util.SparkUtil;

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
        .inverted(HoodConstants.kHoodinverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(HoodConstants.kHoodCurrentLimit)
        .voltageCompensation(12.0);
    hoodConfig
        .encoder
        .positionConversionFactor(HoodConstants.kHoodPositionConversionFactor)
        .velocityConversionFactor(HoodConstants.kHoodVelocityConversionFactor);
    hoodConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(HoodConstants.kHoodP, HoodConstants.kHoodI, HoodConstants.kHoodD)
        .maxMotion
        .maxAcceleration(HoodConstants.kHoodMaxAccel)
        .cruiseVelocity(HoodConstants.kHoodCruiseVel)
        .allowedProfileError(HoodConstants.kHoodDeviationErr);
    hoodConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    hood.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hood.clearFaults();

    SparkUtil.tunePID(
            "Hood",
            hood,
            hoodConfig,
            new double[] {HoodConstants.kHoodP, HoodConstants.kHoodI, HoodConstants.kHoodD, 0,0,0,0, HoodConstants.kHoodMaxAccel, HoodConstants.kHoodCruiseVel, HoodConstants.kHoodCruiseVel},
            ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters,
                false,
                true
        );

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
