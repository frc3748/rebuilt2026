package frc.robot.diagnostics;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotState;
import frc.robot.commands.autos.AutoRoutine;
import frc.robot.commands.autos.DiagnosticAuto;
import frc.robot.robots.RobotDefinition;

abstract class DiagnosticAutosTest {
    private static final int kMaxLoops = 3000;
    private static RobotState state;

    protected static void boot(RobotDefinition definition) {
        assertTrue(HAL.initialize(500, 0));
        SimHooks.pauseTiming();
        state = new RobotState(definition);
        step(25);
    }

    private static void step(int loops) {
        for (int i = 0; i < loops; i++) {
            SimHooks.stepTiming(0.02);
            CommandScheduler.getInstance().run();
            state.updateSimulation();
        }
    }

    @Test
    void everyDiagnosticAutoEndsWhereItShould() {
        List<DiagnosticAuto> diagnostics = new ArrayList<>();
        for (AutoRoutine auto : state.getDefinition().autos(state)) {
            if (auto instanceof DiagnosticAuto diagnostic) {
                diagnostics.add(diagnostic);
            }
        }
        assertFalse(diagnostics.isEmpty());

        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        List<String> failures = new ArrayList<>();
        for (DiagnosticAuto diagnostic : diagnostics) {
            Command command = diagnostic.build();
            CommandScheduler.getInstance().schedule(command);
            int loops = 0;
            while (command.isScheduled() && loops < kMaxLoops) {
                step(1);
                loops++;
            }
            DiagnosticAuto.Result result = diagnostic.lastResult();
            if (result != null) {
                System.out.printf(Locale.ROOT, "%s %s: %.1f cm, %.1f°, tracked within %.1f cm%n", state.getDefinition().name(),
                        diagnostic.name(), result.errorMeters() * 100, result.errorDegrees(), result.trackingMeters() * 100);
            }
            if (command.isScheduled() || result == null) {
                CommandScheduler.getInstance().cancel(command);
                failures.add(diagnostic.name() + " didn't finish");
            } else if (!result.passed()) {
                failures.add(String.format(Locale.ROOT, "%s ended %.1f cm and %.1f° off (tracked within %.1f cm)",
                        diagnostic.name(), result.errorMeters() * 100, result.errorDegrees(), result.trackingMeters() * 100));
            }
        }
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        assertTrue(failures.isEmpty(), String.join("\n", failures));
    }
}
