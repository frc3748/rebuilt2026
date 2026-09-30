package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;

public class SimulatedRobotState {
    private static final double kLookbackSeconds = 1.0;

    private final TimeInterpolatableBuffer<Pose2d> fieldToRobotTruth = TimeInterpolatableBuffer.createBuffer(kLookbackSeconds);

    public synchronized void addFieldToRobot(Pose2d pose) {
        fieldToRobotTruth.addSample(RobotTime.getTimestampSeconds(), pose);
    }

    public synchronized Pose2d getLatestFieldToRobot() {
        var entry = fieldToRobotTruth.getInternalBuffer().lastEntry();
        return entry == null ? null : entry.getValue();
    }
}
