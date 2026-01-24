package frc.robot.subsystems.shooter.turret;

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


public class TurretIOSpark implements TurretIO{
 
    // Hardware objects
    private final SparkMax turret;

    private final RelativeEncoder turretEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController turretController;

    public TurretIOSpark(){

    turret = new SparkMax(0, MotorType.kBrushless);

    turretEncoder = turret.getEncoder();

    turretController = turret.getClosedLoopController();

    // Configure turret motor
    // NEED TO CONFIGURE
    SparkMaxConfig turretConfig = new SparkMaxConfig();
    turretConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(0)
        .voltageCompensation(12.0);
    turretConfig
        .encoder
        .positionConversionFactor(0)
        .velocityConversionFactor(0);
    turretConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, 0) // we might need this
        .pid(0, 0, 0)
        .maxMotion
        .maxAcceleration(0)
        .cruiseVelocity(0)
        .allowedProfileError(0);
    turretConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    // turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turret.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turret.clearFaults();


    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        
    }

    @Override
    public void setTurretVoltage(double volts) {
        turret.setVoltage(volts);
    }

    @Override
    public void setTurretPosition(double position) {
        turretController.setSetpoint(position, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void stopTurret() {
        turret.stopMotor();
    }


    
    
}
