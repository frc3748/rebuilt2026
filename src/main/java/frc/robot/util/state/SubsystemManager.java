package frc.robot.util.state;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.*;

public class SubsystemManager {
  private final List<StateMachine<?>> subsystems = new ArrayList<>();

  SubsystemManager() {}

  public void registerSubsystem(StateMachine<?> subsystem) {
    registerSubsystem(subsystem, "", true);
  }

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

  public void notifyTeleopStart() {
    prepSubsystems();

    for (StateMachine<?> sm : subsystems) {
      sm.onTeleopStart();
    }
  }

  public void notifyTestStart() {
    prepSubsystems();

    for (StateMachine<?> sm : subsystems) {
      sm.onTestStart();
    }
  }

  public void notifyAutonomousStart() {
    prepSubsystems();

    for (StateMachine<?> sm : subsystems) {
      sm.onAutonomousStart();
    }
  }

  public void registerSubsystems(StateMachine<?>... subsystems) {
    for (StateMachine<?> s : subsystems) {
      registerSubsystem(s);
    }
  }

  public void determineAllSubsystems() {
    for (StateMachine<?> sm : subsystems) {
      sm.determineState();
    }
  }

  public void enableAllSubsystems() {
    for (StateMachine<?> s : subsystems) {
      s.enable();
    }
  }

  public void disableAllSubsystems() {
    for (StateMachine<?> s : subsystems) {
      s.disable();
    }
  }

  public void prepSubsystems() {
    enableAllSubsystems();
    determineAllSubsystems();
  }
}