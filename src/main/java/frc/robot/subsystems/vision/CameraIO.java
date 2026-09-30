package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface CameraIO {
    @AutoLog
    class CameraInputs {
        public boolean connected = false;
        public boolean seesTarget = false;
        public FiducialObservation[] fiducialObservations = new FiducialObservation[0];

        public MegatagPoseEstimate megatagPoseEstimate = new MegatagPoseEstimate(null, 0, 0, 0, 0, null);
        public int megatagCount = 0;
        public double megatagAvgDist = 0.0;

        public MegatagPoseEstimate megatag2PoseEstimate = new MegatagPoseEstimate(null, 0, 0, 0, 0, null);
        public int megatag2Count = 0;
        public double megatag2AvgDist = 0.0;

        public Pose3d fieldToRobot3d = new Pose3d();
    }

    default void updateInputs(CameraInputs inputs) {}

    default void setRobotOrientation(Rotation2d fieldToRobot, double yawRateDegreesPerSecond) {}
}
