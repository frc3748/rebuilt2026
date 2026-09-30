package frc.robot;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.CameraConfig;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIO.ObjectObservation;
import frc.robot.subsystems.vision.CameraIO.PoseObservation;
import frc.robot.subsystems.vision.CameraIO.PoseSource;
import frc.robot.subsystems.vision.CameraInputsAutoLogged;
import frc.robot.subsystems.vision.DetectedObject;
import frc.robot.subsystems.vision.VisionMeasurement;

class RobotStateSmokeTest {
    private static RobotState state;

    @BeforeAll
    static void setup() {
        assertTrue(HAL.initialize(500, 0));
        state = new RobotState();
        loop(25);
    }

    private static void loop(int times) {
        for (int i = 0; i < times; i++) {
            CommandScheduler.getInstance().run();
            state.updateLogger();
            state.updateSimulation();
        }
    }

    @Test
    void subsystemsFollowRequestedStates() {
        state.getShooter().requestTransition(Shooter.State.SHOOTING);
        state.getIntake().requestTransition(Intake.State.INTAKE);
        loop(25);
        assertEquals(Shooter.State.SHOOTING, state.getShooter().getState());
        assertEquals(Intake.State.INTAKE, state.getIntake().getState());
    }

    @Test
    void climbZeroingIsAStateThatEndsInStow() {
        state.getClimb().requestTransition(Climb.State.ZEROING);
        loop(25);
        assertEquals(Climb.State.STOW, state.getClimb().getState());
    }

    @Test
    void overrideWinsOverRequestedStateUntilCleared() {
        Intake intake = state.getIntake();
        intake.setOverride(Intake.State.STOW);
        intake.requestTransition(Intake.State.OUTAKE);
        loop(25);
        assertTrue(intake.isOverridden());
        assertEquals(Intake.State.OUTAKE, intake.getState());
        intake.clearOverride();
        assertFalse(intake.isOverridden());
    }

    @Test
    void cameraInputsRoundTripThroughTheLog() {
        CameraInputsAutoLogged inputs = new CameraInputsAutoLogged();
        inputs.connected = true;
        inputs.tagIds = new int[] { 4, 7 };
        inputs.poseObservations = new PoseObservation[] {
                new PoseObservation(1.5, new Pose3d(3, 2, 0, new Rotation3d()), 0.1, 2, 2.5, PoseSource.MEGATAG_2)
        };
        inputs.objectObservations = new ObjectObservation[] { new ObjectObservation(1.5, 0, 3.0, -10.0, 1.2, 0.9) };

        LogTable table = new LogTable(0);
        inputs.toLog(table);
        CameraInputsAutoLogged restored = new CameraInputsAutoLogged();
        restored.fromLog(table);

        assertTrue(restored.connected);
        assertArrayEquals(new int[] { 4, 7 }, restored.tagIds);
        assertEquals(inputs.poseObservations[0], restored.poseObservations[0]);
        assertEquals(inputs.objectObservations[0], restored.objectObservations[0]);
    }

    @Test
    void cameraTurnsObservationsIntoWeightedMeasurementsAndObjects() {
        double now = Timer.getFPGATimestamp();
        Pose2d robot = state.getLatestFieldToRobot().getValue();
        Pose3d seen = new Pose3d(new Pose2d(robot.getX() + 3, robot.getY() + 2, Rotation2d.kZero));

        CameraIO fake = new CameraIO() {
            @Override
            public void updateInputs(CameraInputs inputs) {
                inputs.poseObservations = new PoseObservation[] {
                        new PoseObservation(now, seen, 0.0, 2, 2.0, PoseSource.MEGATAG_1),
                        new PoseObservation(now, seen, 0.0, 2, 2.0, PoseSource.MEGATAG_2),
                        new PoseObservation(now, seen, 0.9, 1, 2.0, PoseSource.SINGLE_TAG)
                };
                inputs.objectObservations = new ObjectObservation[] { new ObjectObservation(now, 1, 0.0, 0.0, 1.0, 1.0) };
            }
        };
        CameraConfig config = new CameraConfig("Test", "test", CameraConfig.Type.PHOTON)
                .robotToCamera(new Transform3d(new Translation3d(0, 0, 1), new Rotation3d(0, Math.toRadians(45), 0)));
        Camera camera = new Camera(config, fake);
        camera.update(state, false);

        List<VisionMeasurement> measurements = camera.getMeasurements();
        assertEquals(2, measurements.size());
        assertTrue(Double.isInfinite(measurements.get(0).stdDevs().get(0, 0)));
        assertTrue(Double.isFinite(measurements.get(0).stdDevs().get(2, 0)));
        assertTrue(Double.isFinite(measurements.get(1).stdDevs().get(0, 0)));
        assertTrue(Double.isInfinite(measurements.get(1).stdDevs().get(2, 0)));

        DetectedObject object = camera.getObjects().get(0);
        Pose2d robotNow = state.getLatestFieldToRobot().getValue();
        assertEquals(1.0, object.fieldPosition().getDistance(robotNow.getTranslation()), 1e-3);
    }
}
