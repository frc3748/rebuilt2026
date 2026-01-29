package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotState;
import frc.robot.util.SimulatedRobotState;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOSimPhoton extends VisionIOHardwareLimelight {

    private static final int kResWidth = 1280;
    private static final int kResHeight = 800;
    private static final double kDiagFovDeg = 97.7;

    private static final double[] SIM_STDDEVS = new double[] {
        0.1,
        0.1,
        0.0,
        0.0,
        0.0,
        0.5,
        0.1,
        0.1,
        0.0,
        0.0,
        0.0,
        0.5,
    };

    private final PhotonCamera turretCamera = new PhotonCamera("turretCamera");
    private final PhotonCamera elevatorCamera = new PhotonCamera(
        "elevatorCamera"
    );

    private final PhotonCameraSim turretCameraSim;
    private final PhotonCameraSim elevatorCameraSim;
    private final VisionSystemSim visionSim;

    private final RobotState state;
    private final SimulatedRobotState simRobotState;

    private final double horizFovRad;
    private final double vertFovRad;

    public VisionIOSimPhoton(
        RobotState state,
        SimulatedRobotState simRobotState
    ) {
        super(state);
        this.state = state;
        this.simRobotState = simRobotState;

        horizFovRad = computeHorizontalFovRad(
            kResWidth,
            kResHeight,
            kDiagFovDeg
        );
        vertFovRad = computeVerticalFovRad(kResWidth, kResHeight, kDiagFovDeg);

        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(VisionConstants.kAprilTagLayout);

        var prop = new SimCameraProperties();
        prop.setCalibration(
            kResWidth,
            kResHeight,
            Rotation2d.fromDegrees(kDiagFovDeg)
        );
        prop.setFPS(20);
        prop.setAvgLatencyMs(20);

        turretCameraSim = new PhotonCameraSim(turretCamera, prop);
        elevatorCameraSim = new PhotonCameraSim(elevatorCamera, prop);

        visionSim.addCamera(turretCameraSim, new Transform3d());

        Transform3d robotToElevatorCamera = new Transform3d(
            new Translation3d(
                VisionConstants.kTurretToCameraBX,
                VisionConstants.kTurretToCameraBY,
                VisionConstants.kCameraBHeightOffGroundMeters
            ),
            new Rotation3d(
                0.0,
                -Units.degreesToRadians(VisionConstants.kCameraBPitchDegrees),
                0.0
            )
        );
        visionSim.addCamera(elevatorCameraSim, robotToElevatorCamera);
    }

    @Override
    public void readInputs(
        CameraInputsAutoLogged turret,
        CameraInputsAutoLogged chassis
    ) {
        Pose2d simPose = getSimPose();
        if (simPose != null) {
            visionSim.update(simPose);
        }

        updateTurretTransform();

        NetworkTable turretTable = NetworkTableInstance.getDefault().getTable(
            VisionConstants.kLimelightTableName
        );
        NetworkTable elevatorTable = NetworkTableInstance.getDefault().getTable(
            VisionConstants.kLimelightBTableName
        );

        writeToTable(turretCamera.getLatestResult(), turretTable);
        writeToTable(elevatorCamera.getLatestResult(), elevatorTable);

        super.readInputs(turret, chassis);
    }

    private Pose2d getSimPose() {
        Pose2d simPose = null;
        if (simRobotState != null) {
            simPose = simRobotState.getLatestFieldToRobot();
        }
        if (simPose == null) {
            Map.Entry<Double, Pose2d> latest = state.getLatestFieldToRobot();
            if (latest != null) {
                simPose = latest.getValue();
            }
        }
        return simPose;
    }

    private void updateTurretTransform() {
        var turretRotationEntry = state.getLatestRobotToTurret();
        if (turretRotationEntry == null) {
            return;
        }

        Rotation2d turretRot = turretRotationEntry.getValue();
        Transform3d robotToTurret = new Transform3d(
            new Translation3d(),
            new Rotation3d(0.0, 0.0, turretRot.getRadians())
        );

        Transform3d turretToLens = new Transform3d(
            new Translation3d(
                VisionConstants.kTurretToCameraX,
                VisionConstants.kTurretToCameraY,
                VisionConstants.kCameraHeightOffGroundMeters
            ),
            new Rotation3d(
                0.0,
                -Units.degreesToRadians(VisionConstants.kCameraPitchDegrees),
                0.0
            )
        );

        visionSim.adjustCamera(
            turretCameraSim,
            robotToTurret.plus(turretToLens)
        );
    }

    private void writeToTable(PhotonPipelineResult result, NetworkTable table) {
        var multiTag = result.getMultiTagResult();
        if (
            !result.hasTargets() ||
            multiTag.isEmpty() ||
            multiTag.get().estimatedPose == null
        ) {
            table.getEntry("tv").setDouble(0.0);
            table.getEntry("botpose_wpiblue").setDoubleArray(new double[0]);
            table.getEntry("botpose_orb_wpiblue").setDoubleArray(new double[0]);
            table.getEntry("rawfiducials").setDoubleArray(new double[0]);
            return;
        }

        Transform3d pose = multiTag.get().estimatedPose.best;
        List<PhotonTrackedTarget> targets = result.getTargets();
        int tagCount = targets.size();

        double avgArea = 0.0;
        double avgDist = 0.0;
        for (PhotonTrackedTarget target : targets) {
            avgArea += target.getArea();
            avgDist += target
                .getBestCameraToTarget()
                .getTranslation()
                .getNorm();
        }
        if (tagCount > 0) {
            avgArea /= tagCount;
            avgDist /= tagCount;
        }

        List<Double> botPose = new ArrayList<>(11 + tagCount * 7);
        botPose.add(pose.getX());
        botPose.add(pose.getY());
        botPose.add(pose.getZ());
        botPose.add(0.0);
        botPose.add(0.0);
        botPose.add(Units.radiansToDegrees(pose.getRotation().getZ()));
        botPose.add(0.0);
        botPose.add((double) tagCount);
        botPose.add(0.0);
        botPose.add(avgDist);
        botPose.add(avgArea);

        List<Double> rawFiducials = new ArrayList<>(tagCount * 7);
        for (PhotonTrackedTarget target : targets) {
            double txnc = toNormalized(target.getYaw(), horizFovRad);
            double tync = toNormalized(target.getPitch(), vertFovRad);
            double area = target.getArea();
            double distToCamera = target
                .getBestCameraToTarget()
                .getTranslation()
                .getNorm();
            double distToRobot = distToCamera;
            double ambiguity = target.getPoseAmbiguity();

            botPose.add((double) target.getFiducialId());
            botPose.add(txnc);
            botPose.add(tync);
            botPose.add(area);
            botPose.add(distToCamera);
            botPose.add(distToRobot);
            botPose.add(ambiguity);

            rawFiducials.add((double) target.getFiducialId());
            rawFiducials.add(txnc);
            rawFiducials.add(tync);
            rawFiducials.add(area);
            rawFiducials.add(distToCamera);
            rawFiducials.add(distToRobot);
            rawFiducials.add(ambiguity);
        }

        table.getEntry("tv").setDouble(1.0);
        table
            .getEntry("botpose_wpiblue")
            .setDoubleArray(toDoubleArray(botPose));
        table
            .getEntry("botpose_orb_wpiblue")
            .setDoubleArray(toDoubleArray(botPose));
        table
            .getEntry("rawfiducials")
            .setDoubleArray(toDoubleArray(rawFiducials));
        table.getEntry("stddevs").setDoubleArray(SIM_STDDEVS);
    }

    private static double toNormalized(double angleDeg, double fovRad) {
        if (fovRad <= 0.0) {
            return 0.0;
        }
        double angleRad = Units.degreesToRadians(angleDeg);
        double normalized = Math.tan(angleRad) / Math.tan(fovRad / 2.0);
        return clamp(normalized, -1.0, 1.0);
    }

    private static double computeVerticalFovRad(
        int width,
        int height,
        double diagFovDeg
    ) {
        double diagFovRad = Units.degreesToRadians(diagFovDeg);
        double aspect = (double) width / (double) height;
        double tanDiag = Math.tan(diagFovRad / 2.0);
        double tanVert = tanDiag / Math.sqrt(1.0 + aspect * aspect);
        return 2.0 * Math.atan(tanVert);
    }

    private static double computeHorizontalFovRad(
        int width,
        int height,
        double diagFovDeg
    ) {
        double vertFovRad = computeVerticalFovRad(width, height, diagFovDeg);
        double aspect = (double) width / (double) height;
        double tanHoriz = Math.tan(vertFovRad / 2.0) * aspect;
        return 2.0 * Math.atan(tanHoriz);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double[] toDoubleArray(List<Double> values) {
        double[] data = new double[values.size()];
        for (int i = 0; i < values.size(); i++) {
            data[i] = values.get(i);
        }
        return data;
    }
}
