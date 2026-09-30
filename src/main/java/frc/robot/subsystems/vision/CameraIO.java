package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public interface CameraIO {
    enum PoseSource {
        MEGATAG_1(Double.POSITIVE_INFINITY, 1.0, true),
        MEGATAG_2(0.5, Double.POSITIVE_INFINITY, false),
        MULTI_TAG(1.0, 1.0, false),
        SINGLE_TAG(1.0, 1.0, true),
        TRIG_SOLVE(0.5, Double.POSITIVE_INFINITY, false);

        public final double linearStdDevFactor;
        public final double angularStdDevFactor;
        public final boolean rejectAmbiguous;

        PoseSource(double linearStdDevFactor, double angularStdDevFactor, boolean rejectAmbiguous) {
            this.linearStdDevFactor = linearStdDevFactor;
            this.angularStdDevFactor = angularStdDevFactor;
            this.rejectAmbiguous = rejectAmbiguous;
        }
    }

    record PoseObservation(
            double timestamp,
            Pose3d robotPose,
            double ambiguity,
            int tagCount,
            double averageTagDistance,
            PoseSource source) {}

    record ObjectObservation(
            double timestamp,
            int classId,
            double yawDegrees,
            double pitchDegrees,
            double area,
            double confidence) {}

    @AutoLog
    class CameraInputs {
        public boolean connected = false;
        public int pipeline = 0;
        public PoseObservation[] poseObservations = new PoseObservation[0];
        public ObjectObservation[] objectObservations = new ObjectObservation[0];
        public int[] tagIds = new int[0];
    }

    default void updateInputs(CameraInputs inputs) {}

    default void setRobotOrientation(Rotation2d heading, double yawRateDegreesPerSecond) {}

    default void setRobotToCamera(Transform3d robotToCamera) {}

    default void setPipeline(int index) {}
}
