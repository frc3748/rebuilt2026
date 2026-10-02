package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotState;
import frc.robot.commands.autos.AutoRoutine;
import frc.robot.game.FieldObstacles;
import frc.robot.robots.comp.CompRobot;
import frc.robot.subsystems.drive.DriveConfig;

class BlairAutoTest {
    private static final double kMaxOffPathMeters = 0.15;
    private static final double kWallMargin = 0.03;
    private static final int kSettleLoops = 3;
    private static final int kMaxLoops = 1250;
    private static RobotState state;

    @BeforeAll
    static void boot() {
        assertTrue(HAL.initialize(500, 0));
        SimHooks.pauseTiming();
        SimulatedArena.overrideInstance(new Arena2026Rebuilt());
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();
        state = new RobotState(new CompRobot());
        step(25);
    }

    private static void step(int loops) {
        for (int i = 0; i < loops; i++) {
            SimHooks.stepTiming(0.02);
            CommandScheduler.getInstance().run();
            state.updateSimulation();
        }
    }

    private static double distanceToLine(Translation2d point, List<Translation2d> line) {
        double best = Double.MAX_VALUE;
        for (int i = 0; i + 1 < line.size(); i++) {
            Translation2d a = line.get(i);
            Translation2d ab = line.get(i + 1).minus(a);
            double lengthSquared = ab.getX() * ab.getX() + ab.getY() * ab.getY();
            double t = lengthSquared == 0.0 ? 0.0
                    : Math.max(0.0, Math.min(1.0, (point.minus(a).getX() * ab.getX() + point.minus(a).getY() * ab.getY()) / lengthSquared));
            best = Math.min(best, point.getDistance(a.plus(ab.times(t))));
        }
        return best;
    }

    private static void run(String autoName) {
        AutoRoutine auto = state.getDefinition().autos(state).stream().filter(routine -> routine.name().equals(autoName)).findFirst()
                .orElseThrow();
        DriveConfig config = state.getDrive().getConfig();
        Map<String, List<Translation2d>> reachable = new HashMap<>();
        for (PathPlannerPath path : auto.previewPaths()) {
            List<Translation2d> line = new ArrayList<>();
            for (PathPoint point : path.getAllPathPoints()) {
                line.add(FieldObstacles.insideWalls(point.position, Rotation2d.kZero, config.bumperLength(), config.bumperWidth(), kWallMargin));
            }
            reachable.put(path.name, line);
        }
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        step(20);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        Command command = auto.build();
        CommandScheduler.getInstance().schedule(command);
        List<String> finished = new ArrayList<>();
        String active = "";
        int loopsOnPath = 0;
        double worst = 0.0;
        for (int loop = 0; command.isScheduled() && loop < kMaxLoops; loop++) {
            step(1);
            String now = PathPlannerAuto.currentPathName == null ? "" : PathPlannerAuto.currentPathName;
            if (!active.isEmpty() && !active.equals(now)) {
                finished.add(active);
            }
            loopsOnPath = now.equals(active) ? loopsOnPath + 1 : 0;
            active = now;
            if (reachable.containsKey(active) && loopsOnPath > kSettleLoops) {
                worst = Math.max(worst, distanceToLine(state.getDrive().getPose().getTranslation(), reachable.get(active)));
            }
        }
        CommandScheduler.getInstance().cancel(command);
        System.out.printf(Locale.ROOT, "%s: worst %.1f cm off the path, finished %s%n", autoName, worst * 100.0, finished);
        assertEquals(new ArrayList<>(reachable.keySet()).stream().sorted().toList(), finished.stream().sorted().toList(),
                autoName + " didn't finish every path");
        assertTrue(worst < kMaxOffPathMeters, String.format(Locale.ROOT, "%s strayed %.1f cm off its path", autoName, worst * 100.0));
    }

    @Test
    void depotSideBlairStaysOnItsPaths() {
        run("Depot Side Blair (GAME)");
    }

    @Test
    void hpSideBlairStaysOnItsPaths() {
        run("HP Side Blair (GAME)");
    }
}
