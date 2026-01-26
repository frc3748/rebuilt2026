package frc.robot.subsystems.shooter.turret;

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

import frc.robot.util.SparkUtil;

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
        .inverted(TurretConstants.kTurretinverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(TurretConstants.kTurretCurrentLimit)
        .voltageCompensation(12.0);
    turretConfig
        .encoder
        .positionConversionFactor(TurretConstants.kTurretPositionConversionFactor)
        .velocityConversionFactor(TurretConstants.kTurretVelocityConversionFactor);
    turretConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingInputRange(0,2 * Math.PI)
        .pid(TurretConstants.kTurretP, TurretConstants.kTurretI, TurretConstants.kTurretD)
        .maxMotion
        .maxAcceleration(TurretConstants.kTurretMaxAccel)
        .cruiseVelocity(TurretConstants.kTurretCruiseVel)
        .allowedProfileError(TurretConstants.kTurretDeviationErr);
    turretConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    turret.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turret.clearFaults();

    SparkUtil.tunePID(
            "Turret",
            turret,
            turretConfig,
            new double[] {TurretConstants.kTurretP, TurretConstants.kTurretI, TurretConstants.kTurretD, 0,0,0,0, TurretConstants.kTurretMaxAccel, TurretConstants.kTurretCruiseVel, TurretConstants.kTurretCruiseVel},
            ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters,
                false,
                true
        );




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
