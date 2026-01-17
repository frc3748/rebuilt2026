package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.util.state.StateMachine;
import org.littletonrobotics.junction.Logger;

public class Intake extends StateMachine<Intake.State> implements IntakeIO {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    super("Intake", State.UNDETERMINED, State.class);
    this.io = io;

    registerStateTransitions();
    registerStateCommands();
    enable();
  }

  @Override
  public void update() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void run(double speed) {
    io.setVoltage(speed * 12.0);
  }

  public void stop() {
    io.stop();
  }

  private void registerStateTransitions() {
    addOmniTransitions(State.IDLE, State.INTAKE, State.EXPEL, State.VOLTAGE_CALC, State.PROX_TRIPPED);
  }

  private void registerStateCommands() {
    registerStateCommand(State.IDLE, Commands.runOnce(() -> stop(), this));
    
    registerStateCommand(State.INTAKE, Commands.run(() -> run(1.0), this));
    registerStateCommand(State.EXPEL, Commands.run(() -> run(-1.0), this));
    // Setup Commands for Pathfinding as needed
  }

  public enum State {
    UNDETERMINED,
    IDLE,
    INTAKE,
    EXPEL,
    VOLTAGE_CALC,

    // flags
    PROX_TRIPPED
  }

  @Override
  protected void determineSelf() {
    throw new UnsupportedOperationException("Unimplemented method 'determineSelf'");
  }
}
