package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Arrays;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.TunableNumber;

public class ModuleIOSim implements ModuleIO {
  private final DriveConfig config;
  private final SwerveModuleSimulation module;
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController turnMotor;
  private final PIDController driveController;
  private final PIDController turnController;
  private final double driveKs;
  private final double driveKv;
  private final double driveKa;
  private final double steerKv;
  private final TunableNumber steerFeedforward;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private double driveFFVolts = 0.0;
  private double turnFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim(DriveConfig config, SwerveModuleSimulation module) {
    this.config = config;
    this.module = module;
    driveMotor = module.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(config.driveCurrentLimit));
    turnMotor = module.useGenericControllerForSteer().withCurrentLimit(Amps.of(config.turnCurrentLimit));
    driveController = new PIDController(config.driveSimP, 0, config.driveSimD);
    turnController = new PIDController(config.turnSimP, 0, config.turnSimD);
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    TunableNumber.field("Drive Sim/kP", config, "driveSimP").onChange(driveController::setP);
    TunableNumber.field("Drive Sim/kD", config, "driveSimD").onChange(driveController::setD);
    TunableNumber.field("Turn Sim/kP", config, "turnSimP").onChange(turnController::setP);
    TunableNumber.field("Turn Sim/kD", config, "turnSimD").onChange(turnController::setD);
    steerFeedforward = TunableNumber.field("Turn PID/Steer FF", config, "steerFeedforward");
    steerKv = config.steerKv();
    DCMotor motor = config.driveGearbox;
    driveKs = DriveSimulation.kDriveFrictionVolts;
    driveKv = config.driveReduction / (config.wheelRadiusMeters * motor.KvRadPerSecPerVolt);
    driveKa = config.robotMassKg / 4.0 * config.wheelRadiusMeters * motor.rOhms / (config.driveReduction * motor.KtNMPerAmp);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    double wheelVelocity = module.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    if (driveClosedLoop) {
      driveAppliedVolts = driveFFVolts + driveController.calculate(wheelVelocity);
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnFFVolts + turnController.calculate(module.getSteerAbsoluteFacing().getRadians());
    } else {
      turnController.reset();
    }
    driveAppliedVolts = MathUtil.clamp(driveAppliedVolts, -12.0, 12.0);
    turnAppliedVolts = MathUtil.clamp(turnAppliedVolts, -12.0, 12.0);
    driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
    turnMotor.requestVoltage(Volts.of(turnAppliedVolts));

    inputs.driveConnected = true;
    inputs.drivePositionRad = module.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocityRadPerSec = wheelVelocity;
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(module.getDriveMotorStatorCurrent().in(Amps));

    inputs.turnConnected = true;
    inputs.encoderConnected = true;
    inputs.turnPosition = module.getSteerAbsoluteFacing();
    inputs.turnVelocityRadPerSec = module.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(module.getSteerMotorStatorCurrent().in(Amps));

    inputs.odometryTimestamps = DriveSimulation.odometryTimestamps();
    inputs.odometryDrivePositionsRad = Arrays.stream(module.getCachedDriveWheelFinalPositions()).mapToDouble(angle -> angle.in(Radians)).toArray();
    inputs.odometryTurnPositions = module.getCachedSteerAbsolutePositions();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityMetersPerSec, double accelerationMetersPerSecSq) {
    driveClosedLoop = true;
    driveFFVolts = driveKs * Math.signum(velocityMetersPerSec) + driveKv * velocityMetersPerSec + driveKa * accelerationMetersPerSecSq;
    driveController.setSetpoint(velocityMetersPerSec / config.wheelRadiusMeters);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation, double velocityRadPerSec) {
    turnClosedLoop = true;
    turnFFVolts = steerFeedforward.get() * steerKv * velocityRadPerSec;
    turnController.setSetpoint(rotation.getRadians());
  }

  @Override
  public void liftDriveCurrentLimit(double amps) {
    driveMotor.withCurrentLimit(Amps.of(amps));
  }

  @Override
  public void restoreDriveCurrentLimit() {
    driveMotor.withCurrentLimit(Amps.of(config.driveCurrentLimit));
  }
}
