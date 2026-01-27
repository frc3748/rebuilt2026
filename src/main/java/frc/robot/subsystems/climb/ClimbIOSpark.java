package frc.robot.subsystems.climb;

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


public class ClimbIOSpark implements ClimbIO{
 
    // Hardware objects
    private final SparkMax climb;

    private final RelativeEncoder climbEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController climbController;

    public ClimbIOSpark(){

    climb = new SparkMax(0, MotorType.kBrushless);

    climbEncoder = climb.getEncoder();

    climbController = climb.getClosedLoopController();

    // Configure extention motor
    SparkMaxConfig climbConfig = new SparkMaxConfig();
    climbConfig
        .inverted(climbInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(10)
        .voltageCompensation(12.0);
    climbConfig
        .encoder
        .positionConversionFactor(climbEncoderPositionFactor)
        .velocityConversionFactor(climbEncoderVelocityFactor);
    climbConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .pid(climbKp, climbKi, climbKd)
        .maxMotion
        .maxAcceleration(0)
        .cruiseVelocity(0)
        .allowedProfileError(0);
    climbConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    climb.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climb.clearFaults();

    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        // Update climb inputs - Lily:I don't know how this works tbh
        sparkStickyFault = false;
        ifOk(climb, climbEncoder::getPosition, (value) -> inputs.climbPositionRad = value);
        ifOk(climb, climbEncoder::getVelocity, (value) -> inputs.climbVelocityRadPerSec = value);
        ifOk(
            climb,
            new DoubleSupplier[] {climb::getAppliedOutput, climb::getBusVoltage},
            (values) -> inputs.climbAppliedVolts = values[0] * values[1]);
        ifOk(climb, climb::getOutputCurrent, (value) -> inputs.climbCurrentAmps = value);
        inputs.climbConnected = climbConnectedDebounce.calculate(!sparkStickyFault);
    }

    @Override
    public void setClimbVoltage(double volts) {
        climb.setVoltage(volts);
    }

    @Override
    public void setClimbPosition(double position) {
        climbController.setSetpoint(position, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void stopClimb() {
        climb.stopMotor();
    }

    SparkUtil.tunePID(
        "Climb PID " + module, 
        climb, 
        climbConfig, 
        new double [] {climbKp, climbKi, climbKd, 0, climbKf, 0, 0},
        ResetMode.kResetSafeParameters, 
        PersistMode.kPersistParameters,
        true,
        false
        );
  }



