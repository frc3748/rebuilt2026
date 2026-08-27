package frc.robot.subsystems.shooter.dumper;

import static frc.robot.util.SparkUtil.ifOk;
import java.util.function.DoubleSupplier;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.SparkUtil;

public class DumperIOSpark implements DumperIO {
    private final SparkMax dumper;
    private final RelativeEncoder dumperEncoder;
    private final SparkClosedLoopController dumperController;
    private double latencyCompensatedMS = DumperConstants.latencyCompensationMS;
    private double desiredPos = 0.0;
    private boolean dumperDisabled = false;

    public DumperIOSpark() {
        dumper = new SparkMax(DumperConstants.kDumperCanId, MotorType.kBrushless);
        dumperEncoder = dumper.getEncoder();
        dumperController = dumper.getClosedLoopController();

        SparkMaxConfig dumperConfig = new SparkMaxConfig();
        dumperConfig.inverted(DumperConstants.kDumperInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(DumperConstants.kDumperCurrentLimit)
            .voltageCompensation(12.0);
        dumperConfig.encoder
            .positionConversionFactor(DumperConstants.kDumperPositionConversionFactor)
            .velocityConversionFactor(DumperConstants.kDumperVelocityConversionFactor)
            .quadratureAverageDepth(10).quadratureMeasurementPeriod(2);
        dumperConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .positionWrappingInputRange(DumperConstants.kBackwardSoftLimit, DumperConstants.kForwardSoftLimit)
            .positionWrappingEnabled(false)
            .pid(DumperConstants.kDumperP, DumperConstants.kDumperI, DumperConstants.kDumperD)
                .maxMotion.maxAcceleration(DumperConstants.kDumperMaxAccel)
                .cruiseVelocity(DumperConstants.kDumperCruiseVel)
                .allowedProfileError(DumperConstants.kDumperDeviationErr);
        dumperConfig.signals
            .primaryEncoderPositionPeriodMs(10)
            .primaryEncoderPosition(true)
            .primaryEncoderVelocity(true)
            .appliedOutput(true)
            .busVoltage(true)
            .outputCurrent(true);
        dumperConfig.softLimit
            .forwardSoftLimit(DumperConstants.kForwardSoftLimit).forwardSoftLimitEnabled(true)
            .reverseSoftLimit(DumperConstants.kBackwardSoftLimit).reverseSoftLimitEnabled(true);
        dumper.configure(dumperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        dumper.clearFaults();
        SparkUtil.tunePID("Dumper", dumper, dumperConfig,
            new double[] { DumperConstants.kDumperP, DumperConstants.kDumperI, DumperConstants.kDumperD, DumperConstants.kDumperS, DumperConstants.kDumperV, DumperConstants.kDumperA, DumperConstants.kDumperG, DumperConstants.kDumperMaxAccel, DumperConstants.kDumperCruiseVel, DumperConstants.kDumperDeviationErr },
            new InstantCommand(() -> dumperController.setTunedGains(
                SparkUtil.getTunePID("Dumper", new double[] { DumperConstants.kDumperP, DumperConstants.kDumperI, DumperConstants.kDumperD, DumperConstants.kDumperS, DumperConstants.kDumperV, DumperConstants.kDumperA, DumperConstants.kDumperG, DumperConstants.kDumperMaxAccel, DumperConstants.kDumperCruiseVel, DumperConstants.kDumperDeviationErr })
            ))
        );
        DogLog.tunable("Dumper/Latency", () -> dumper.configAccessor.closedLoop.maxMotion.latencyCompDurationMs());
        SmartDashboard.putData("Dumper/Disable", new InstantCommand(() -> dumperDisabled = true));
        SmartDashboard.putData("Dumper/Enable", new InstantCommand(() -> dumperDisabled = false));
    }

    @Override
    public void updateInputs(DumperIOInputs inputs) {
        ifOk(dumper, dumperEncoder::getVelocity, (v) -> inputs.dumperVelRadPerSec = v);
        ifOk(dumper, dumperEncoder::getPosition, (v) -> inputs.dumperPos = v);
        ifOk(dumper, new DoubleSupplier[] { dumper::getAppliedOutput, dumper::getBusVoltage },
                (vals) -> inputs.appliedVolts = vals[0] * vals[1]);
        ifOk(dumper, dumper::getOutputCurrent, (v) -> inputs.currentAmps = v);
        inputs.desiredPos = this.desiredPos;
        inputs.isDumping = Math.abs(this.desiredPos - inputs.dumperPos) > 0.05;
    }

    @Override
    public void setDumperVoltage(double volts) {
        dumper.setVoltage(volts);
    }

    @Override
    public void setDumperPosition(double position, double ff) {
        if (dumperDisabled) return;
        position = MathUtil.clamp(position, DumperConstants.kBackwardSoftLimit, DumperConstants.kForwardSoftLimit);
        this.desiredPos = position;
        dumperController.setSetpoint(position, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kVoltage);
    }

    @Override
    public void setDumperPosition(double position) {
        if (dumperDisabled) return;
        position = MathUtil.clamp(position, DumperConstants.kBackwardSoftLimit, DumperConstants.kForwardSoftLimit);
        this.desiredPos = position;
        dumperController.setSetpoint(position, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public double getDumperPosition() {
        return dumperController.getMAXMotionSetpointPosition();
    }

    @Override
    public double getDumperVelocity() {
        return dumperController.getMAXMotionSetpointVelocity();
    }

    @Override
    public void stopDumper() {
        dumper.stopMotor();
    }
}
