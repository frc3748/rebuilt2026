package frc.robot.subsystems.intake;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io, int topID, int bottomID) {
    this.io = io;

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
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void run(double speed) {
    io.setVoltage(speed * 12.0);
  }

  public void stop() {
    io.stop();
  }
}
