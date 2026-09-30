package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public record DetectedObject(double timestamp, int classId, Translation2d fieldPosition, double confidence) {
    public double distanceTo(Translation2d point) {
        return fieldPosition.getDistance(point);
    }

    public Pose2d poseFrom(Translation2d origin) {
        return new Pose2d(fieldPosition, fieldPosition.minus(origin).getAngle());
    }
}
