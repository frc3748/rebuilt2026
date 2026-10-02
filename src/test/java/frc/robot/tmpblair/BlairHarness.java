package frc.robot.tmpblair;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.junit.jupiter.api.Test;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotState;
import frc.robot.commands.autos.AutoRoutine;
import frc.robot.robots.comp.CompRobot;

abstract class BlairHarness {
    private static final String OUT = "/private/tmp/claude-501/-Users-samir-Documents-GitHub-rebuilt2026/327b0393-1aff-457b-bbc9-5c3cbb2b674f/scratchpad/";
    private RobotState state;

    private void step(int loops) {
        for (int i = 0; i < loops; i++) {
            SimHooks.stepTiming(0.02);
            CommandScheduler.getInstance().run();
            state.updateSimulation();
        }
    }

    private static double distanceToPolyline(Translation2d p, List<Translation2d> line) {
        double best = Double.MAX_VALUE;
        for (int i = 0; i + 1 < line.size(); i++) {
            Translation2d a = line.get(i);
            Translation2d b = line.get(i + 1);
            Translation2d ab = b.minus(a);
            double len2 = ab.getX() * ab.getX() + ab.getY() * ab.getY();
            double t = len2 == 0 ? 0 : Math.max(0, Math.min(1, ((p.getX() - a.getX()) * ab.getX() + (p.getY() - a.getY()) * ab.getY()) / len2));
            best = Math.min(best, p.getDistance(a.plus(ab.times(t))));
        }
        return best;
    }

    protected void setup(RobotState state) {}

    protected abstract String label();

    private void scenario(String autoName, double offsetY, boolean vision, String tag) throws IOException {
        state.getVision().setEstimating(vision);
        AutoRoutine auto = state.getDefinition().autos(state).stream().filter(a -> a.name().equals(autoName)).findFirst().orElseThrow();
        List<List<Translation2d>> lines = new ArrayList<>();
        for (PathPlannerPath path : auto.previewPaths()) {
            List<Translation2d> line = new ArrayList<>();
            for (PathPoint point : path.getAllPathPoints()) {
                line.add(point.position);
            }
            lines.add(line);
        }
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        step(20);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        Command command = auto.build();
        CommandScheduler.getInstance().schedule(command);
        step(1);
        var sim = state.getDrive().getSimulation().orElseThrow();
        Pose2d truth0 = sim.getSimulatedDriveTrainPose();
        sim.setSimulationWorldPose(new Pose2d(truth0.getX(), truth0.getY() + offsetY, truth0.getRotation()));
        int collisions0 = state.getDrive().getCollision().events();
        int lastCollisions = collisions0;
        List<String> hits = new ArrayList<>();
        double worstCross = 0;
        double worstLead = 0;
        int stuck = 0;
        int loops = 0;
        try (FileWriter csv = new FileWriter(OUT + "blair_" + tag + ".csv")) {
            csv.write("t,tx,ty,ox,oy,gx,gy,lead,cross,speed\n");
            while (command.isScheduled() && loops < 1250) {
                step(1);
                loops++;
                Pose2d truth = sim.getSimulatedDriveTrainPose();
                Pose2d odo = state.getDrive().getPose();
                Pose2d goal = state.getDrive().getPathTarget().orElse(odo);
                ChassisSpeeds f = sim.getDriveTrainSimulatedChassisSpeedsFieldRelative();
                double speed = Math.hypot(f.vxMetersPerSecond, f.vyMetersPerSecond);
                double cross = Double.MAX_VALUE;
                for (List<Translation2d> line : lines) {
                    cross = Math.min(cross, distanceToPolyline(truth.getTranslation(), line));
                }
                double lead = state.getDrive().getPathTarget().isPresent() && goal.getX() < 10 ? goal.getTranslation().getDistance(odo.getTranslation()) : 0;
                int collisions = state.getDrive().getCollision().events();
                if (collisions != lastCollisions) {
                    hits.add(String.format(Locale.ROOT, "%.2fs at (%.2f, %.2f)", loops * 0.02, truth.getX(), truth.getY()));
                    lastCollisions = collisions;
                }
                worstCross = Math.max(worstCross, cross);
                worstLead = Math.max(worstLead, lead);
                csv.write(String.format(Locale.ROOT, "%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f%n", loops * 0.02, truth.getX(), truth.getY(),
                        odo.getX(), odo.getY(), goal.getX(), goal.getY(), lead, cross, speed));
            }
        }
        CommandScheduler.getInstance().cancel(command);
        System.out.printf(Locale.ROOT, "%s | %s offset %.2f m vision %s: worst off the drawn path %.2f m, worst target lead %.2f m, collisions %s, ran %.1f s%n",
                label(), autoName, offsetY, vision, worstCross, worstLead, hits, loops * 0.02);
    }

    @Test
    void blair() throws IOException {
        assertTrue(HAL.initialize(500, 0));
        SimHooks.pauseTiming();
        SimulatedArena.overrideInstance(new Arena2026Rebuilt());
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();
        state = new RobotState(new CompRobot());
        setup(state);
        step(25);
        scenario("Depot Side Blair (GAME)", 0.0, true, label() + "_depot_0");
        scenario("Depot Side Blair (GAME)", 0.12, false, label() + "_depot_drift_wall");
        scenario("Depot Side Blair (GAME)", -0.12, false, label() + "_depot_drift_in");
        scenario("HP Side Blair (GAME)", 0.0, true, label() + "_hp_0");
        scenario("HP Side Blair (GAME)", -0.12, false, label() + "_hp_drift_wall");
    }
}
