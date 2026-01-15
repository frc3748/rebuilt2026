package frc.robot.util.state.transitions;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandTransition<E extends Enum<E>> extends TransitionBase<E> {
  private final Command command;

  public CommandTransition(E startState, E endState, Command command) {
    super(startState, endState);
    this.command = command;
  }

  @Override
  public String toString() {
    return "Start state: "
        + this.startState.name()
        + ", End state: "
        + this.endState.name()
        + ", Command: "
        + this.command.toString();
  }

  @Override
  public boolean isFinished() {
    return command.isFinished();
  }

  @Override
  public boolean hasStarted() {
    return command.isScheduled() || command.isFinished();
  }

  public Command getCommand() {
    return command;
  }

  @Override
  public void execute() {
    command.schedule();
  }

  @Override
  public void cancel() {
    command.cancel();
  }
}