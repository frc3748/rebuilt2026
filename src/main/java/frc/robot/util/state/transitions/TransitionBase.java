package frc.robot.util.state.transitions;

public abstract class TransitionBase<E extends Enum<E>> {
  protected final E startState;
  protected final E endState;

  public TransitionBase(E startState, E endState) {
    this.startState = startState;
    this.endState = endState;
  }

  public boolean isValid(TransitionBase<E> other) {
    if (other.startState == this.startState && other.endState == this.endState) return false;

    return true;
  }

  public abstract String toString();

  public abstract void execute();

  public abstract void cancel();

  public abstract boolean isFinished();

  public abstract boolean hasStarted();

  public E getStartState() {
    return startState;
  }

  public E getEndState() {
    return endState;
  }
}