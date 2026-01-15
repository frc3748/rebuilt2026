package frc.robot.util.state;

public class SubsystemManagerFactory {

  private static SubsystemManager instance;

  /**
   * The Subsystem Manager is a singleton to make accessing it from anywhere easier
   *
   * @return the current instance of SubsystemManager
   */
  public static SubsystemManager getInstance() {
    if (instance == null) instance = new SubsystemManager();
    return instance;
  }

  public static void setInstance(SubsystemManager instanceToSet) {
    instance = instanceToSet;
  }
}