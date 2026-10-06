package frc.robot.util.state;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.util.motor.Motor;
import frc.robot.util.state.graph.DirectionalEnumGraph;
import frc.robot.util.state.transitions.CommandTransition;
import frc.robot.util.state.transitions.TransitionBase;

import java.util.*;
import org.littletonrobotics.junction.Logger;

public abstract class StateMachine<E extends Enum<E>> extends SubsystemBase {
  private final DirectionalEnumGraph<E, TransitionBase<E>> transitionGraph;
  private TransitionBase<E> currentTransition;
  private TransitionBase<E> queuedTransition;
  private final HashMap<E, Command> stateCommands;
  private final Timer transitionTimer;
  private final Set<E> currentFlags;
  private static final double STUCK_SECONDS = 2.0;
  private final double transitionTimeOut = 2;
  private final E undeterminedState;
  private E currentState;

  private boolean enabled;

  private final Class<E> enumType;
  private final List<StateMachine<?>> subsystems;

  private final Alert stuckAlert;
  private final Timer stuckTimer = new Timer();

  private final List<Hardware> hardware = new ArrayList<>();
  private Runnable override;
  private int rejectedRequests;
  private String[] logKeys;
  private E loggedDesired;
  private E loggedState;
  private Set<E> loggedFlags;
  private int loggedSwitches = -1;

  public StateMachine(String name, E undeterminedState, Class<E> enumType) {
    this.enumType = enumType;

    this.undeterminedState = undeterminedState;
    currentState = undeterminedState;
    currentTransition = null;
    transitionTimer = new Timer();
    currentFlags = new HashSet<>();
    stateCommands = new HashMap<>();
    subsystems = new ArrayList<>();
    stuckAlert = new Alert(name + " is stuck", AlertType.kWarning);

    setName(name);

    transitionGraph = new DirectionalEnumGraph<>(enumType);
    enabled = false;
  }

  public final List<StateMachine<?>> getChildSubsystems() {
    return subsystems;
  }

  protected final void addChildSubsystem(StateMachine<?> machine) {
    subsystems.add(machine);
  }

  public final E getState() {
    return currentState;
  }

  protected final void addHardware(Hardware... devices) {
    hardware.addAll(Arrays.asList(devices));
  }

  public final List<Motor> getMotors() {
    return hardware.stream().filter(Motor.class::isInstance).map(Motor.class::cast).toList();
  }

  public final void setOverride(Runnable action) {
    override = action;
  }

  public final void setOverride(E state) {
    override = () -> applyState(state);
  }

  public final void clearOverride() {
    override = null;
  }

  public final boolean isOverridden() {
    return override != null;
  }

  public final void enable() {
    determineState();
    enabled = true;

    onEnable();
  }

  public final boolean isEnabled() {
    return enabled;
  }

  protected void onEnable() {}

  protected void onTeleopStart() {}

  protected void onAutonomousStart() {}

  protected void onTestStart() {}

  public final void disable() {
    enabled = false;
    if (currentTransition != null) currentTransition.cancel();
    currentTransition = null;
    queuedTransition = null;
    setState(undeterminedState);
    if (getCurrentCommand() != null) getCurrentCommand().cancel();

    onDisable();
  }

  protected void onDisable() {}

  public final boolean isDetermined() {
    return currentState != undeterminedState;
  }

  public final void registerStateCommand(E state, Command command) {
    stateCommands.put(state, command);
  }

  protected final void registerStateCommand(E state, Runnable toRun) {
    registerStateCommand(state, new InstantCommand(toRun));
  }

  protected final void addTransition(E start, E end, Command command) {
    transitionGraph.addEdge(new CommandTransition<>(start, end, command));
  }

  protected final void addTransition(E start, E end) {
    transitionGraph.addEdge(new CommandTransition<>(start, end, new InstantCommand()));
  }

  protected final void addTransition(E start, E end, Runnable toRun) {
    transitionGraph.addEdge(new CommandTransition<>(start, end, new InstantCommand(toRun)));
  }

  protected final void removeTransition(E start, E end) {
    transitionGraph.removeEdge(start, end);
  }

  protected final void removeAllTransitionsFromState(E start) {
    for (E s : enumType.getEnumConstants()) {
      transitionGraph.removeEdge(start, s);
    }
  }

  public final void addOmniTransition(E state, Command run) {
    for (E s : enumType.getEnumConstants()) {
      if (s != state) {
        addTransition(s, state, run);
      }
    }
  }

  public final void addOmniTransition(E state, Runnable run) {
    addOmniTransition(state, new InstantCommand(run));
  }

  public final void addOmniTransition(E state) {
    addOmniTransition(state, () -> {});
  }

  protected final void allowAllTransitions() {
    for (E state : enumType.getEnumConstants()) {
      if (state != undeterminedState) {
        addOmniTransition(state);
      }
    }
  }

  @SafeVarargs
  public final void addOmniTransitions(E... states) {
    for (E state : states) {
      addOmniTransition(state);
    }
  }

  public final void addCommutativeTransition(E start, E end, Command run) {
    transitionGraph.addEdge(new CommandTransition<>(start, end, run));
    transitionGraph.addEdge(new CommandTransition<>(end, start, run));
  }

  public final void addCommutativeTransition(E start, E end, Runnable toRun) {
    transitionGraph.addEdge(new CommandTransition<>(start, end, new InstantCommand(toRun)));
    transitionGraph.addEdge(new CommandTransition<>(end, start, new InstantCommand(toRun)));
  }

  public final void addCommutativeTransition(E start, E end) {
    transitionGraph.addEdge(new CommandTransition<>(start, end, new InstantCommand()));
    transitionGraph.addEdge(new CommandTransition<>(end, start, new InstantCommand()));
  }

  public final void addCommutativeTransition(E start, E end, Command run1, Command run2) {
    transitionGraph.addEdge(new CommandTransition<>(start, end, run1));
    transitionGraph.addEdge(new CommandTransition<>(end, start, run2));
  }

  public final void addCommutativeTransition(E start, E end, Runnable run1, Runnable run2) {
    transitionGraph.addEdge(new CommandTransition<>(start, end, new InstantCommand(run1)));
    transitionGraph.addEdge(new CommandTransition<>(end, start, new InstantCommand(run2)));
  }

  public final boolean isTransitioning() {
    return currentTransition != null;
  }

  public final TransitionBase<E> getCurrentTransition() {
    return currentTransition;
  }

  public final void requestTransition(E state) {
    TransitionBase<E> transition = transitionGraph.getEdge(currentState, state);
    if (!isTransitioning() && transition != null && state != currentState) {
      currentTransition = transition;
      cancelStateCommand();
      transition.execute();
      transitionTimer.start();
    } else if (state != currentState) {
      queuedTransition = transition;
    }
    if (transition == null && state != currentState) {
      rejectedRequests++;
      Logger.recordOutput(getName() + "/rejectedRequests", rejectedRequests);
      Logger.recordOutput(getName() + "/lastRejected", currentState.name() + " -> " + state.name());
    }
  }

  private void cancelStateCommand() {
    if (stateCommands.containsKey(getState())) {
      Command prevCommand = stateCommands.get(getState());
      if (prevCommand.isScheduled()) prevCommand.cancel();
    }
  }

  public final void requestTransition(E state, Command command) {
    stateCommands.put(state, command);
    requestTransition(state);
  }

  public final Command transitionCommand(E state) {
    return new FunctionalCommand(
        () -> requestTransition(state), () -> {}, (interrupted) -> {}, () -> getState() == state);
  }

  public final Command transitionCommand(E state, Command command) {
    return new FunctionalCommand(
        () -> requestTransition(state, command),
        () -> {},
        (interrupted) -> {},
        () -> getState() == state);
  }

  public final Command transitionCommand(E state, Command command, boolean waitForState) {
    if (waitForState) {
      return transitionCommand(state, command);
    } else {
      return new InstantCommand(() -> requestTransition(state, command));
    }
  }

  public final Command transitionCommand(E state, boolean waitForState) {
    if (waitForState) {
      return transitionCommand(state);
    } else {
      return new InstantCommand(() -> requestTransition(state));
    }
  }

  public final Command waitForState(E state) {
    return new WaitUntilCommand(() -> getState() == state);
  }

  public final Command waitForFlag(E flag) {
    return new WaitUntilCommand(() -> isFlag(flag));
  }

  public final Set<E> getCurrentFlags() {
    return currentFlags;
  }

  public final String[] getCurrentFlagsAsArray() {
    int n = getCurrentFlags().size();
    String arr[] = new String[n];

    int i = 0;
    for (E flag : getCurrentFlags()) {
      arr[i] = flag.toString();
      i++;
    }

    return arr;
  }

  public final boolean isFlag(E state) {
    return getCurrentFlags().contains(state);
  }

  public final void setFlag(E flag) {
    currentFlags.add(flag);
  }

  public final Command setFlagCommand(E flag) {
    return new InstantCommand(() -> setFlag(flag));
  }

  public final void clearFlag(E flag) {
    currentFlags.remove(flag);
  }

  public final Command clearFlagCommand(E flag) {
    return new InstantCommand(() -> clearFlag(flag));
  }

  public final void clearFlags() {
    currentFlags.clear();
  }

  @Override
  public final void periodic() {
    long start = System.nanoTime();

    if (enabled) {
      updateTransitioning();
    }
    updateStuck();

    hardware.forEach(Hardware::read);
    recordLogs();
    update();

    if (override != null) {
      override.run();
    } else {
      applyState(currentState);
    }
    applyConstraints();

    hardware.forEach(Hardware::write);
    Logger.recordOutput("LoopTimes/" + getName(), (System.nanoTime() - start) / 1e6);
  }

  private void updateStuck() {
    boolean waiting = isTransitioning() && DriverStation.isEnabled() && !isOverridden();
    if (!waiting) {
      stuckTimer.stop();
      stuckTimer.reset();
    } else {
      stuckTimer.start();
    }
    boolean stuck = stuckTimer.hasElapsed(STUCK_SECONDS);
    if (stuck) {
      stuckAlert.setText(getName() + " is stuck going to " + getCurrentTransition().getEndState().name());
    }
    stuckAlert.set(stuck);
  }

  private void recordLogs() {
    if (logKeys == null) {
      String name = getName();
      logKeys = new String[] {name + "/desired", name + "/state", name + "/transitioning", name + "/flags",
          name + "/enabled", name + "/overridden"};
    }
    E desired = isTransitioning() ? getCurrentTransition().getEndState() : getState();
    if (desired != loggedDesired) {
      loggedDesired = desired;
      Logger.recordOutput(logKeys[0], desired.name());
    }
    if (getState() != loggedState) {
      loggedState = getState();
      Logger.recordOutput(logKeys[1], loggedState.toString());
    }
    if (!currentFlags.equals(loggedFlags)) {
      loggedFlags = new HashSet<>(currentFlags);
      Logger.recordOutput(logKeys[3], getCurrentFlagsAsArray());
    }
    int switches = (isTransitioning() ? 1 : 0) | (enabled ? 2 : 0) | (override != null ? 4 : 0);
    if (switches != loggedSwitches) {
      loggedSwitches = switches;
      Logger.recordOutput(logKeys[2], isTransitioning());
      Logger.recordOutput(logKeys[4], enabled);
      Logger.recordOutput(logKeys[5], override != null);
    }

    logAdditionalOutputs();
  }

  protected final void setState(E state) {
    cancelStateCommand();

    currentState = state;

    clearFlags();
    if (stateCommands.containsKey(state)) {
      stateCommands.get(state).schedule();
    }
  }

  private void updateTransitioning() {
    if (isTransitioning() && currentTransition.isFinished()) {
      setState(currentTransition.getEndState());
      currentTransition = null;
      transitionTimer.stop();
      transitionTimer.reset();
    }

    if (queuedTransition != null
        && (!isTransitioning() || transitionTimer.hasElapsed(transitionTimeOut))) {
      forceChangeTransition();
    }
  }

  private void forceChangeTransition() {
    if (currentTransition != null) currentTransition.cancel();
    currentTransition = queuedTransition;
    currentTransition.execute();
    queuedTransition = null;
    transitionTimer.reset();
    clearFlags();
  }

  public final String toString() {
    return "Stated Subsystem Machine - "
        + getName()
        + "; In State: "
        + getState().name()
        + "; In Transition: "
        + isTransitioning();
  }

  public final void determineState() {
    if (!isDetermined()) determineSelf();
  }

  protected void update() {}

  protected void applyState(E state) {}

  protected void applyConstraints() {}

  protected abstract void determineSelf();

  protected void logAdditionalOutputs() {}

  public Map<String, Sendable> additionalSendables() {
    return new HashMap<>();
  }
}