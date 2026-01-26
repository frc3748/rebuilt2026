package frc.robot.subsystems.vision;

import java.util.Optional;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.util.MathHelpers;
import frc.robot.util.state.StateMachine;

public class VisionSubsystem extends StateMachine<VisionSubsystem.State> {
    private final VisionIO io;
    private final RobotState state;

    private final CameraInputsAutoLogged turretCamera = new CameraInputsAutoLogged();
    private final CameraInputsAutoLogged chassisCamera = new CameraInputsAutoLogged();

    private double lastTurretTimestamp = 0.0;
    private double lastChassisTimestamp = 0.0;

    public enum State {
        UNDETERMINED,
        VISION_SCANNING,
        BROKEN
    }

    public VisionSubsystem(VisionIO io, RobotState state) {
        super("Vision", State.UNDETERMINED, State.class);
        this.io = io;
        this.state = state;
        enable();
    }

    @Override
    public void update() {
        io.readInputs(turretCamera, chassisCamera);

        if (getState() == State.BROKEN)
            return;

        // Process both cameras using the robust methods below
        var turretEst = processCamera(turretCamera, "Turret", true);
        var chassisEst = processCamera(chassisCamera, "Chassis", false);

        // Weighted Fusion
        Optional<VisionFieldPoseEstimate> fused = fuseEstimates(turretEst, chassisEst);

        if (fused.isPresent()) {
            if (getState() != State.VISION_SCANNING)
                setState(State.VISION_SCANNING);
            state.updateMegatagEstimate(fused.get());
        } else if (getState() == State.VISION_SCANNING) {
            setState(State.UNDETERMINED);
        }
    }

    private Optional<VisionFieldPoseEstimate> processCamera(
            VisionIO.CameraInputs cam, String name, boolean isTurret) {

        if (!cam.seesTarget)
            return Optional.empty();

        // 1. Pick the best estimate (Prefer Megatag 2)
        boolean useMT2 = cam.megatag2PoseEstimate != null && cam.megatag2Count > 0;
        var estimate = useMT2 ? cam.megatag2PoseEstimate : cam.megatagPoseEstimate;
        if (estimate == null)
            return Optional.empty();

        double timestamp = estimate.timestampSeconds();
        String logPath = "Vision/" + name + "/";

        // 2. Timing Check
        if (timestamp == (isTurret ? lastTurretTimestamp : lastChassisTimestamp))
            return Optional.empty();

        // 3. Motion Validation (Method fully implemented below)
        if (!isMotionValid(timestamp, isTurret, logPath))
            return Optional.empty();

        // 4. Transform Logic (Method fully implemented below)
        Optional<Transform2d> robotToCamera = getRobotToCamera(timestamp, isTurret);
        if (robotToCamera.isEmpty())
            return Optional.empty();

        // If Limelight offsets are 0, 'fieldToRobot()' is effectively 'fieldToCamera'
        Pose2d fieldToRobot = estimate.fieldToRobot().plus(robotToCamera.get().inverse());

        // 5. Standard Deviation Calculation
        Matrix<N3, N1> stdDevs = calculateStdDevs(cam, estimate, useMT2);

        if (isTurret)
            lastTurretTimestamp = timestamp;
        else
            lastChassisTimestamp = timestamp;

        return Optional.of(new VisionFieldPoseEstimate(
                fieldToRobot, timestamp, stdDevs, estimate.fiducialIds().length));
    }

    /**
     * Calculates the dynamic transform from the Robot Center to the Camera Lens
     * at a specific point in time using the RobotState history buffers.
     */
    private Optional<Transform2d> getRobotToCamera(double timestamp, boolean isTurret) {
        if (isTurret) {
            // Retrieve the historical rotation of the turret from the RobotState buffer
            Optional<Rotation2d> turretRotation = state.getRobotToTurret(timestamp);

            if (turretRotation.isPresent()) {
                // Combine: Robot -> Turret (Historical) -> Camera (Static offset on turret)
                Transform2d robotToTurret = MathHelpers.transform2dFromRotation(turretRotation.get());
                return Optional.of(robotToTurret.plus(state.getTurretToCamera(true)));
            }
            return Optional.empty();
        }
        // Chassis camera uses a static offset defined in RobotState
        return Optional.of(state.getTurretToCamera(false));
    }

    /**
     * Validates if the robot was stable enough during the frame capture
     * to trust the vision data.
     */
    private boolean isMotionValid(double timestamp, boolean isTurret, String logPath) {
        final double kMaxAngularVel = Units.degreesToRadians(120.0);
        final double kWindow = 0.1; // 100ms window

        // Check Chassis movement
        var chassisVel = state.getMaxAbsDriveYawAngularVelocityInRange(timestamp - kWindow, timestamp);
        double totalAngularVel = Math.abs(chassisVel.orElse(0.0));

        // If it's the turret camera, we must account for the turret's own velocity
        if (isTurret) {
            var turretVel = state.getMaxAbsTurretYawAngularVelocityInRange(timestamp - kWindow, timestamp);
            totalAngularVel = Math.abs(chassisVel.orElse(0.0) + turretVel.orElse(0.0));
        }

        boolean isStable = totalAngularVel < kMaxAngularVel;
        Logger.recordOutput(logPath + "TotalAngularVel", totalAngularVel);
        Logger.recordOutput(logPath + "IsStable", isStable);

        return isStable;
    }

    private Matrix<N3, N1> calculateStdDevs(VisionIO.CameraInputs cam, MegatagPoseEstimate est,
            boolean isMT2) {
        int offset = isMT2 ? 6 : 0;
        // Use the LL4 provided standard deviations if available
        return VecBuilder.fill(
                cam.standardDeviations[offset],
                cam.standardDeviations[offset + 1],
                cam.standardDeviations[offset + 5]);
    }

    private Optional<VisionFieldPoseEstimate> fuseEstimates(
            Optional<VisionFieldPoseEstimate> turret, Optional<VisionFieldPoseEstimate> chassis) {
        if (turret.isEmpty())
            return chassis;
        if (chassis.isEmpty())
            return turret;

        VisionFieldPoseEstimate a = turret.get();
        VisionFieldPoseEstimate b = chassis.get();

        if (b.getTimestampSeconds() < a.getTimestampSeconds()) {
            VisionFieldPoseEstimate tmp = a;
            a = b;
            b = tmp;
        }

        // Preview both estimates to the same timestamp
        Transform2d a_T_b = state.getFieldToRobot(b.getTimestampSeconds())
                .get()
                .minus(state.getFieldToRobot(a.getTimestampSeconds()).get());

        Pose2d poseA = a.getVisionRobotPoseMeters().transformBy(a_T_b);
        Pose2d poseB = b.getVisionRobotPoseMeters();

        // Inverse‑variance weighting
        var varianceA = a.getVisionMeasurementStdDevs().elementTimes(a.getVisionMeasurementStdDevs());
        var varianceB = b.getVisionMeasurementStdDevs().elementTimes(b.getVisionMeasurementStdDevs());

        Rotation2d fusedHeading = poseB.getRotation();
        if (varianceA.get(2, 0) < VisionConstants.kLargeVariance
                && varianceB.get(2, 0) < VisionConstants.kLargeVariance) {
            fusedHeading = new Rotation2d(
                    poseA.getRotation().getCos() / varianceA.get(2, 0)
                            + poseB.getRotation().getCos() / varianceB.get(2, 0),
                    poseA.getRotation().getSin() / varianceA.get(2, 0)
                            + poseB.getRotation().getSin() / varianceB.get(2, 0));
        }

        double weightAx = 1.0 / varianceA.get(0, 0);
        double weightAy = 1.0 / varianceA.get(1, 0);
        double weightBx = 1.0 / varianceB.get(0, 0);
        double weightBy = 1.0 / varianceB.get(1, 0);

        Pose2d fusedPose = new Pose2d(
                new Translation2d(
                        (poseA.getTranslation().getX() * weightAx
                                + poseB.getTranslation().getX() * weightBx)
                                / (weightAx + weightBx),
                        (poseA.getTranslation().getY() * weightAy
                                + poseB.getTranslation().getY() * weightBy)
                                / (weightAy + weightBy)),
                fusedHeading);

        Matrix<N3, N1> fusedStdDev = VecBuilder.fill(
                Math.sqrt(1.0 / (weightAx + weightBx)),
                Math.sqrt(1.0 / (weightAy + weightBy)),
                Math.sqrt(1.0 / (1.0 / varianceA.get(2, 0) + 1.0 / varianceB.get(2, 0))));

        int numTags = a.getNumTags() + b.getNumTags();
        double time = b.getTimestampSeconds();

        return Optional.of(new VisionFieldPoseEstimate(fusedPose, time, fusedStdDev, numTags));
    }

    @Override
    public void determineSelf() {
        setState(State.VISION_SCANNING);
    }
}