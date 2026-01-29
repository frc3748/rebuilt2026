package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for vision system hardware abstraction. */
public interface VisionIO {
    /** * Input data from a single camera.
     * Both the inner and outer classes need @AutoLog.
     */

    @AutoLog
    public static class CameraInputs {

        public boolean seesTarget = false;

        public FiducialObservation[] fiducialObservations =
            new FiducialObservation[0];
        public MegatagPoseEstimate megatagPoseEstimate =
            new MegatagPoseEstimate(null, 0, 0, 0, 0, null);
        public MegatagPoseEstimate megatag2PoseEstimate =
            new MegatagPoseEstimate(null, 0, 0, 0, 0, null);

        public int megatag2Count = 0;
        public int megatagCount = 0;
        public Pose3d pose3d = new Pose3d();
        public double[] standardDeviations = new double[12];
    }

    /** Updates the set of loggable inputs. */
    public default void readInputs(
        CameraInputsAutoLogged turretInput,
        CameraInputsAutoLogged chassisInput
    ) {}
}