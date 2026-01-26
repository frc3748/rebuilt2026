package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;

/**
 * Modified robot state used in simulation. Includes simulated note states.
 */
public class SimulatedRobotState {
  RobotState state;
  TimeInterpolatableBuffer<Pose2d> fieldToRobotSimulatedTruth = TimeInterpolatableBuffer
      .createBuffer(RobotState.LOOKBACK_TIME);

  public SimulatedRobotState(RobotState state) {
    this.state = state;
    SmartDashboard.putString("Accepted Midline Notes", "ABCDE");
  }

  synchronized public void addFieldToRobot(Pose2d pose) {
    fieldToRobotSimulatedTruth.addSample(RobotTime.getTimestampSeconds(), pose);
  }


  synchronized public Pose2d getLatestFieldToRobot() {
    var entry = fieldToRobotSimulatedTruth.getInternalBuffer().lastEntry();
    if (entry == null) {
      return null;
    }
    return entry.getValue();
  }
}