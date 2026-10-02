package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.TunableNumber;

public class ModuleIOSim implements ModuleIO {
  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private final DriveConfig config;
  private final PIDController driveController;
  private final PIDController turnController;
  private final TunableNumber driveKs;
  private final TunableNumber driveKv;
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim(DriveConfig config) {
    this.config = config;
    driveController = new PIDController(config.driveSimP, 0, config.driveSimD);
    turnController = new PIDController(config.turnSimP, 0, config.turnSimD);
    driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(config.driveGearbox, 0.001, config.driveReduction),
        config.driveGearbox);
    turnSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(config.turnGearbox, 0.001, config.turnReduction),
        config.turnGearbox);

    turnController.enableContinuousInput(-Math.PI, Math.PI);
    TunableNumber.field("Drive Sim/kP", config, "driveSimP").onChange(driveController::setP);
    TunableNumber.field("Drive Sim/kD", config, "driveSimD").onChange(driveController::setD);
    TunableNumber.field("Turn Sim/kP", config, "turnSimP").onChange(turnController::setP);
    TunableNumber.field("Turn Sim/kD", config, "turnSimD").onChange(turnController::setD);
    driveKs = TunableNumber.field("Drive Sim/kS", config, "driveSimKs");
    driveKv = TunableNumber.field("Drive Sim/kV", config, "driveSimKv");
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
    } else {
      turnController.reset();
    }

    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
    turnSim.update(0.02);

    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    inputs.turnConnected = true;
    inputs.encoderConnected = true;
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
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
  public void setDriveVelocity(double velocityMetersPerSec) {
    driveClosedLoop = true;
    driveFFVolts = driveKs.get() * Math.signum(velocityMetersPerSec) + driveKv.get() * velocityMetersPerSec;
    driveController.setSetpoint(velocityMetersPerSec / config.wheelRadiusMeters);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}