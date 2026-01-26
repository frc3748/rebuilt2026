package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotState;
import frc.robot.util.SimulatedRobotState;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import java.util.ArrayList;
import java.util.List;

public class VisionIOSimPhoton extends VisionIOHardwareLimelight {
        private final PhotonCamera turretCamera = new PhotonCamera("turretCamera");
        private final PhotonCamera elevatorCamera = new PhotonCamera("elevatorCamera");

        private PhotonCameraSim turretCameraSim;
        private PhotonCameraSim elevatorCameraSim;
        private final VisionSystemSim visionSim;

        private final RobotState state;
        private final SimulatedRobotState simRobotState;

        public VisionIOSimPhoton(RobotState state, SimulatedRobotState simRobotState) {
                super(state);
                this.state = state;
                this.simRobotState = simRobotState;

                visionSim = new VisionSystemSim("main");
                visionSim.addAprilTags(VisionConstants.kAprilTagLayout);

                var prop = new SimCameraProperties();
                prop.setCalibration(1280, 800, Rotation2d.fromDegrees(97.7));
                prop.setFPS(20);
                prop.setAvgLatencyMs(20);

                turretCameraSim = new PhotonCameraSim(turretCamera, prop);
                elevatorCameraSim = new PhotonCameraSim(elevatorCamera, prop);

                visionSim.addCamera(turretCameraSim, new Transform3d());

                // Static offset for Chassis/Elevator camera
                Transform3d robotToElevatorCamera = new Transform3d(
                                new Translation3d(VisionConstants.kTurretToCameraBX, VisionConstants.kTurretToCameraBY,
                                                VisionConstants.kCameraBHeightOffGroundMeters),
                                new Rotation3d(0.0, -Units.degreesToRadians(VisionConstants.kCameraBPitchDegrees),
                                                0.0));
                visionSim.addCamera(elevatorCameraSim, robotToElevatorCamera);
        }

        @Override
        public void readInputs(CameraInputsAutoLogged turret, CameraInputsAutoLogged chassis) {
                // 1. Update Sim World Position
                Pose2d realPose = simRobotState.getLatestFieldToRobot();
                if (realPose != null) {
                        visionSim.update(realPose);
                }

                // 2. Update Turret Dynamic Transform in Sim
                var turretRotationEntry = state.getLatestRobotToTurret();

                // If this returns an Optional, use .isPresent().
                // If it returns a raw object (like Timestamped<Rotation2d>), use a null check.
                if (turretRotationEntry != null) {
                        // Extract the rotation - adjust .getValue() if your wrapper uses a different
                        // getter
                        Rotation2d turretRot = turretRotationEntry.getValue();

                        Transform3d robotToTurret = new Transform3d(
                                        new Translation3d(),
                                        new Rotation3d(0, 0, turretRot.getRadians()));

                        Transform3d turretToLens = new Transform3d(
                                        new Translation3d(
                                                        VisionConstants.kTurretToCameraX,
                                                        VisionConstants.kTurretToCameraY,
                                                        VisionConstants.kCameraHeightOffGroundMeters),
                                        new Rotation3d(0, 0, 0));

                        // Combine transforms: Robot -> Turret Center -> Camera Lens
                        visionSim.adjustCamera(turretCameraSim, robotToTurret.plus(turretToLens));
                }

                // 3. Fake the NetworkTable data for the parent class to read
                NetworkTable turretTable = NetworkTableInstance.getDefault()
                                .getTable(VisionConstants.kLimelightTableName);
                NetworkTable elevatorTable = NetworkTableInstance.getDefault()
                                .getTable(VisionConstants.kLimelightBTableName);

                // Use modern PhotonVision API to avoid warnings
                for (var res : turretCamera.getAllUnreadResults()) {
                        writeToTable(res, turretTable);
                }
                for (var res : elevatorCamera.getAllUnreadResults()) {
                        writeToTable(res, elevatorTable);
                }

                // 4. Call parent to process the "fake" table data into the actual 'inputs'
                // object
                super.readInputs(turret, chassis);
        }

        private void writeToTable(PhotonPipelineResult result, NetworkTable table) {
                table.getEntry("tv").setDouble(result.hasTargets() ? 1.0 : 0.0);
                if (!result.hasTargets())
                        return;

                var multiTag = result.getMultiTagResult();
                if (multiTag.isPresent() && multiTag.get().estimatedPose != null) {
                        Transform3d pose = multiTag.get().estimatedPose.best;

                        // Limelight botpose array format: [x, y, z, roll, pitch, yaw, latency,
                        // tagCount, tagSpan, avgDist, avgArea]
                        List<Double> data = new ArrayList<>(List.of(
                                        pose.getX(), pose.getY(), pose.getZ(),
                                        0.0, 0.0, pose.getRotation().getZ(),
                                        0.0,
                                        (double) multiTag.get().fiducialIDsUsed.size(),
                                        0.0, 0.0,
                                        result.getBestTarget().getArea()));

                        // Append Raw Fiducials (6 entries per tag)
                        for (var target : result.getTargets()) {
                                data.add((double) target.getFiducialId());
                                data.add(target.getYaw()); // tx
                                data.add(target.getPitch()); // ty
                                data.add(target.getArea()); // ta
                                data.add(0.0); // ts
                                data.add(0.0); // tr
                        }

                        double[] dataArray = data.stream().mapToDouble(Double::doubleValue).toArray();
                        table.getEntry("botpose_wpiblue").setDoubleArray(dataArray);

                        // Fake the MT2 standard deviations (Limelight sends these)
                        table.getEntry("stddevs").setDoubleArray(
                                        new double[] { 0.1, 0.1, 0.0, 0.0, 0.0, 0.5, 0.1, 0.1, 0.0, 0.0, 0.0, 0.5 });
                }
        }
}