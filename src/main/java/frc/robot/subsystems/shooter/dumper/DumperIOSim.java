package frc.robot.subsystems.shooter.dumper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.shooter.dumper.DumperIO.DumperIOInputs;
import frc.robot.subsystems.shooter.dumper.DumperConstants;

public class DumperIOSim implements DumperIO {

  private final DCMotorSim motorSim;

  private boolean closedLoop = false;
  private final PIDController controller =
      new PIDController(DumperConstants.kDumperSimP, 0, DumperConstants.kDumperSimD);

  private double appliedVolts = 0.0;
  private double ffVolts = 0.0;
  private double desiredPos = 0.0;

  public DumperIOSim() {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNeo550(1),
                0.001,
                1.0),
            DCMotor.getNeo550(1));
  }

  @Override
  public void updateInputs(DumperIOInputs inputs) {

    double position = motorSim.getAngularPositionRad();

    // Soft limits (match Spark)
    if (position > DumperConstants.kForwardSoftLimit) {
      motorSim.setState(DumperConstants.kForwardSoftLimit, 0);
      position = DumperConstants.kForwardSoftLimit;
    } else if (position < DumperConstants.kBackwardSoftLimit) {
      motorSim.setState(DumperConstants.kBackwardSoftLimit, 0);
      position = DumperConstants.kBackwardSoftLimit;
    }

    // Closed loop control
    if (closedLoop) {
      double pidOutput = controller.calculate(position);
      appliedVolts = pidOutput + ffVolts;
    }

    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(0.02);

    // Update inputs (match Spark structure)


    inputs.dumperPos = position;                                   
    inputs.dumperVelRadPerSec = motorSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(motorSim.getCurrentDrawAmps());
    inputs.desiredPos = desiredPos;
    inputs.isDumping = Math.abs(this.desiredPos - inputs.dumperPos) > 0.05;
  }

  @Override
  public void setDumperVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
  }

  @Override
  public void setDumperPosition(double position, double ff) {
    closedLoop = true;

    // Clamp to soft limits like real controller
    position =
        MathUtil.clamp(
            position,
            DumperConstants.kBackwardSoftLimit,
            DumperConstants.kForwardSoftLimit);

    controller.setSetpoint(position);
    ffVolts = ff;
    desiredPos = position;
  }

  @Override
  public double getDumperPosition() {
    return controller.getSetpoint();
  }

  @Override
  public double getDumperVelocity() {
    return motorSim.getAngularVelocityRadPerSec();
  }


  @Override
  public void stopDumper() {
    closedLoop = false;
    appliedVolts = 0.0;
  }
}