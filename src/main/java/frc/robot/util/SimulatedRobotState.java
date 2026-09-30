package frc.robot.util;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;

public class SimulatedRobotState {
    private static final double kLookbackSeconds = 1.0;

    private final TimeInterpolatableBuffer<Pose2d> fieldToRobotTruth = TimeInterpolatableBuffer.createBuffer(kLookbackSeconds);
    private Supplier<List<Translation3d>> gamePieces = List::of;

    public synchronized void addFieldToRobot(Pose2d pose) {
        fieldToRobotTruth.addSample(RobotTime.getTimestampSeconds(), pose);
    }

    public synchronized Pose2d getLatestFieldToRobot() {
        var entry = fieldToRobotTruth.getInternalBuffer().lastEntry();
        return entry == null ? null : entry.getValue();
    }

    public synchronized void setGamePieces(Supplier<List<Translation3d>> gamePieces) {
        this.gamePieces = gamePieces;
    }

    public synchronized List<Translation3d> getGamePieces() {
        return gamePieces.get();
    }
}
