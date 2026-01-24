package frc.robot.util.state;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.*;

public class SubsystemManager {
  private final List<StateMachine<?>> subsystems = new ArrayList<>();

  SubsystemManager() {}

  /**
   * Add a subsystem and its children to be tracked by the SubsystemManager instance. It will
   * automatically enable and disable it.
   *
   * @param subsystem subsystem to add to the manager
   */
  public void registerSubsystem(StateMachine<?> subsystem) {
    registerSubsystem(subsystem, "", true);
  }

  /**
   * Add a subsystem and its children to be tracked by the SubsystemManager instance. It will
   * automatically enable and disable it.
   *
   * @param subsystem subsystem to add to the manager
   * @param sendToNT whether to send the subsystem's information on network tables
   */
  public void registerSubsystem(StateMachine<?> subsystem, boolean sendToNT) {
    registerSubsystem(subsystem, "", sendToNT);
  }

  private void registerSubsystem(StateMachine<?> subsystem, String subtable, boolean sendToNT) {
    if (!subsystems.contains(subsystem)) {
      subsystems.add(subsystem);
      if (sendToNT) sendOnNt(subsystem, subtable);
    }

    for (StateMachine<?> machine : subsystem.getChildSubsystems()) {
      registerSubsystem(machine, subtable + "/" + subsystem.getName(), sendToNT);
    }
  }

  private void sendOnNt(StateMachine<?> subsystem, String subtable) {
    SmartDashboard.putData(subsystem.getName(), subsystem);

    for (Map.Entry<String, Sendable> entry : subsystem.additionalSendables().entrySet()) {
      if (subtable != "") {
        SmartDashboard.putData(
            "/" + subtable + "/" + subsystem.getName() + "/" + entry.getKey(), entry.getValue());
      } else {
        SmartDashboard.putData("/" + subsystem.getName() + "/" + entry.getKey(), entry.getValue());
      }
    }
  }

  /** Call in teleopInit(), will notify all subsystems that the teleoperated period has started */
  public void notifyTeleopStart() {
    prepSubsystems();

    for (StateMachine<?> sm : subsystems) {
      sm.onTeleopStart();
    }
  }

  /** Call in testInit(), will notify all subsystems that the teleoperated period has started */
  public void notifyTestStart() {
    prepSubsystems();

    for (StateMachine<?> sm : subsystems) {
      sm.onTestStart();
    }
  }

  /** Call in autonomousInit(), will notify all subsystems that the autonomous period has started */
  public void notifyAutonomousStart() {
    prepSubsystems();

    for (StateMachine<?> sm : subsystems) {
      sm.onAutonomousStart();
    }
  }

  /**
   * Register a number of subsystems at once
   *
   * @param subsystems array of stated subsystems
   */
  public void registerSubsystems(StateMachine<?>... subsystems) {
    for (StateMachine<?> s : subsystems) {
      registerSubsystem(s);
    }
  }

  /**
   * Compose a parallel command group that will determine the state of every subsystem that has not
   * yet determined itself
   *
   * @return the command group that will determine every subsystem
   */
  public void determineAllSubsystems() {
    for (StateMachine<?> sm : subsystems) {
      sm.determineState();
    }
  }

  /** Tell all subsystems that they should be enabled */
  public void enableAllSubsystems() {
    for (StateMachine<?> s : subsystems) {
      s.enable();
    }
  }

  /** Tell all subsystems that they should be disabled */
  public void disableAllSubsystems() {
    for (StateMachine<?> s : subsystems) {
      s.disable();
    }
  }

  /** Enable all subsystems and determine all subsystems */
  public void prepSubsystems() {
    enableAllSubsystems();
    determineAllSubsystems();
  }
}