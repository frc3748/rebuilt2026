package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.evergreen.ArenaEvergreen;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotState;
import frc.robot.SimNoise;
import frc.robot.robots.comp.CompRobot;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.motor.Motor;

class SelfTestTest {
    private static final int kMaxLoops = 3000;
    private static final double kNearTestPosition = 0.6;
    private static RobotState state;

    private static void step(int loops) {
        for (int i = 0; i < loops; i++) {
            SimHooks.stepTiming(0.02);
            CommandScheduler.getInstance().run();
            state.updateSimulation();
        }
    }

    @Test
    void passesWhenAMechanismStartsNextToItsTestPosition() {
        assertTrue(HAL.initialize(500, 0));
        SimHooks.pauseTiming();
        SimNoise.seed();
        SimulatedArena.overrideInstance(new ArenaEvergreen(false));
        state = new RobotState(new CompRobot());
        step(25);
        DriverStationSim.setTest(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        DriverStation.refreshData();

        Intake intake = state.getSuperstructure().getIntake().orElseThrow();
        Motor extension = intake.getMotors().stream().filter(motor -> motor.getName().endsWith("Intake Extension")).findFirst().orElseThrow();
        double nearby = extension.getSelfTestPosition() + kNearTestPosition;
        intake.setOverride(() -> extension.holdPosition(nearby));
        step(75);
        intake.clearOverride();

        Command selfTest = SelfTest.build(state);
        CommandScheduler.getInstance().schedule(selfTest);
        for (int loops = 0; selfTest.isScheduled() && loops < kMaxLoops; loops++) {
            step(1);
        }
        assertTrue(SelfTest.hasPassed(), String.join(", ", SelfTest.failures()));
    }
}
