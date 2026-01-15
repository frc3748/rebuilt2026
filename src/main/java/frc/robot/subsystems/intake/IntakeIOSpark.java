package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

public class IntakeIOSpark implements IntakeIO {
  private final SparkFlex topMotor;
  private final SparkFlex bottomMotor;

  public IntakeIOSpark(int topID, int bottomID) {
    topMotor = new SparkFlex(topID, MotorType.kBrushless);
    bottomMotor = new SparkFlex(bottomID, MotorType.kBrushless);

    SparkFlexConfig topConfig = new SparkFlexConfig();
    topConfig.smartCurrentLimit(5);

    SparkFlexConfig bottomConfig = new SparkFlexConfig();
    bottomConfig.smartCurrentLimit(5);

    bottomConfig.follow(topMotor, true);

    topMotor.configure(
        topConfig,
        SparkFlex.ResetMode.kResetSafeParameters,
        SparkFlex.PersistMode.kPersistParameters);

    bottomMotor.configure(
        bottomConfig,
        SparkFlex.ResetMode.kResetSafeParameters,
        SparkFlex.PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.velocity = topMotor.getEncoder().getVelocity();
    inputs.voltage = topMotor.getAppliedOutput() * topMotor.getBusVoltage();
  }

  @Override
  public void setVoltage(double volts) {
    topMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }
}
