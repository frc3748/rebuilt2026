package frc.robot;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.commands.SelfTest;
import frc.robot.commands.autos.AutoRoutine;
import frc.robot.commands.autos.Autos;
import frc.robot.game.DashboardManager;
import frc.robot.game.GameState;
import frc.robot.robots.comp.CompRobot;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.CameraConfig;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIO.PoseObservation;
import frc.robot.subsystems.vision.CameraIO.PoseSource;
import frc.robot.subsystems.vision.HeadingSample;
import frc.robot.util.cockpit.Cockpit;
import frc.robot.util.cockpit.Cockpit.Check;

class CockpitTest {
    private static RobotState state;

    @BeforeAll
    static void setup() {
        assertTrue(HAL.initialize(500, 0));
        state = new RobotState(new CompRobot());
    }

    @Test
    void onlyFailingChecksBlockReady() {
        Cockpit.clear();
        Cockpit.check("warn", "Warns", () -> Check.warn("heads up"));
        Cockpit.update();
        assertTrue(Cockpit.isReady());

        Cockpit.check("fail", "Fails", () -> Check.fail("fix me"));
        Cockpit.update();
        assertFalse(Cockpit.isReady());
        Cockpit.clear();
    }

    @Test
    void selfTestResultSurvivesARestart() throws Exception {
        java.nio.file.Path file = java.nio.file.Path.of("build", "selftest.txt");
        java.nio.file.Files.createDirectories(file.getParent());
        java.nio.file.Files.writeString(file, String.join("\t", "false", Long.toString(System.currentTimeMillis() - 3_600_000L),
                BuildInfo.GIT_SHA, "Hood: didn't reach its test position|Intake: disconnected"));
        SelfTest.Saved saved = SelfTest.saved().orElseThrow();
        assertFalse(saved.passed());
        assertTrue(saved.sameCode());
        assertEquals(1.0, saved.hoursAgo(), 0.01);
        assertEquals(List.of("Hood: didn't reach its test position", "Intake: disconnected"), saved.failures());
        java.nio.file.Files.delete(file);
        assertTrue(SelfTest.saved().isEmpty());
    }

    @Test
    void everyAutoHasAModeForTheDashboard() {
        for (AutoRoutine auto : Autos.all(state)) {
            assertFalse(auto.mode().isBlank(), auto.name() + " needs .mode(...) in Autos.all");
        }
    }

    @Test
    void startAdviceIsFromTheDriversView() {
        assertEquals("Move 12 cm forward", DashboardManager.startAdvice(new Translation2d(0.12, 0), 0, false));
        assertEquals("Move 12 cm back", DashboardManager.startAdvice(new Translation2d(0.12, 0), 0, true));
        assertEquals("Move 8 cm left, turn 4° clockwise", DashboardManager.startAdvice(new Translation2d(0, 0.08), -4, false));
        assertEquals("Turn 3° counterclockwise", DashboardManager.startAdvice(new Translation2d(0.01, 0), 3, true));
        assertEquals("", DashboardManager.startAdvice(new Translation2d(0.01, 0.01), 0.5, false));
    }

    @Test
    void hubKnowsWhetherTheNextShiftFlipsIt() {
        GameState game = new GameState();
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setGameSpecificMessage("R");
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(true);

        assertArrayEquals(new boolean[] { true, false }, hub(game, 58));
        assertArrayEquals(new boolean[] { false, true }, hub(game, 33));
        assertArrayEquals(new boolean[] { true, true }, hub(game, 20));

        DriverStationSim.setGameSpecificMessage("");
        assertArrayEquals(new boolean[] { true, true }, hub(game, 58));
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();
    }

    private static boolean[] hub(GameState game, double matchTime) {
        DriverStationSim.setMatchTime(matchTime);
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();
        game.update();
        return new boolean[] { game.isHubActive(), game.isHubActiveNext() };
    }

    @Test
    void onlyStrictMegaTag1FramesGiveHeadings() {
        double now = Timer.getFPGATimestamp();
        Pose2d robot = state.getLatestFieldToRobot().getValue();
        Pose3d seen = new Pose3d(new Pose2d(robot.getX() + 3, robot.getY() + 2, Rotation2d.fromDegrees(30)));
        PoseObservation[][] frames = {
                { new PoseObservation(now, seen, 0.0, 2, 2.0, PoseSource.MEGATAG_1) },
                { new PoseObservation(now + 0.02, seen, 0.0, 1, 2.0, PoseSource.MEGATAG_1) },
                { new PoseObservation(now + 0.04, seen, 0.0, 2, 6.0, PoseSource.MEGATAG_1) },
                { new PoseObservation(now + 0.06, seen, 0.5, 2, 2.0, PoseSource.MEGATAG_1) },
                { new PoseObservation(now + 0.08, seen, 0.0, 3, 2.0, PoseSource.MEGATAG_2) },
        };
        int[] frame = { 0 };
        CameraIO fake = new CameraIO() {
            @Override
            public void updateInputs(CameraInputs inputs) {
                inputs.poseObservations = frames[frame[0]];
            }
        };
        CameraConfig config = new CameraConfig("Strict", "strict", CameraConfig.Type.LIMELIGHT_3)
                .robotToCamera(new Transform3d(new Translation3d(0, 0, 1), new Rotation3d(0, Math.toRadians(45), 0)));
        Camera camera = new Camera(config, fake);

        camera.update(state, false);
        List<HeadingSample> headings = camera.getHeadings();
        assertEquals(1, headings.size());
        assertEquals(30.0, headings.get(0).heading().getDegrees(), 1e-6);

        for (frame[0] = 1; frame[0] < frames.length; frame[0]++) {
            camera.update(state, false);
            assertTrue(camera.getHeadings().isEmpty(), "frame " + frame[0] + " shouldn't be trusted for heading");
        }
    }
}
