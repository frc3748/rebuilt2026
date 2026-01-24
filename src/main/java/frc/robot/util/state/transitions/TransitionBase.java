package frc.robot.util.state.transitions;

public abstract class TransitionBase<E extends Enum<E>> {
  protected final E startState;
  protected final E endState;

  public TransitionBase(E startState, E endState) {
    this.startState = startState;
    this.endState = endState;
  }

  /**
   * Tell if this transition already has the to and from states of `other`
   *
   * @param other the transition to compare
   * @return whether the transition is a duplicate or not
   */
  public boolean isValid(TransitionBase<E> other) {
    if (other.startState == this.startState && other.endState == this.endState) return false;

    return true;
  }

  /**
   * A string representation of the transition to be easily printed (generally, print the start,
   * end, and interruption states, as well as whatever you're running)
   */
  public abstract String toString();

  /**
   * Run (or start) the transition, then return a BooleanSupplier indicating whether it is finished
   *
   * @return whether the transition is finished
   */
  public abstract void execute();

  /** Cancel the transition (if it's currently running) */
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