package frc.robot.util.motor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.evergreen.ArenaEvergreen;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotState;
import frc.robot.SimNoise;
import frc.robot.commands.autos.MeasureAuto;
import frc.robot.commands.autos.MeasureSteering;
import frc.robot.robots.comp.CompRobot;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.cockpit.Cockpit;
import frc.robot.util.state.StateMachine;
import frc.robot.util.tuning.Tuning;

class MotorAutoTuneTest {
    private static final int kMaxLoops = 3000;
    private static RobotState state;

    @BeforeAll
    static void boot() {
        assertTrue(HAL.initialize(500, 0));
        SimHooks.pauseTiming();
        SimNoise.seed();
        SimulatedArena.overrideInstance(new ArenaEvergreen(false));
        state = new RobotState(new CompRobot());
        step(25);
        Tuning.setEnabled(true);
        DriverStationSim.setTest(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        step(10);
    }

    private static void step(int loops) {
        for (int i = 0; i < loops; i++) {
            SimHooks.stepTiming(0.02);
            CommandScheduler.getInstance().run();
            state.updateSimulation();
        }
    }

    private static void collect(StateMachine<?> machine, List<StateMachine<?>> machines) {
        if (!machine.getMotors().isEmpty()) {
            machines.add(machine);
        }
        machine.getChildSubsystems().forEach(child -> collect(child, machines));
    }

    @Test
    void everyMotorHasAnAutoTuneButton() {
        for (String group : List.of("Flywheel", "Hood", "Intake Extension", "Intake Roller", "Hopper", "Kicker", "Drive PID", "Drive Sim",
                "Turn PID", "Turn Sim")) {
            assertTrue(Cockpit.hasButton("autotune:" + group), "No Auto-tune button for " + group);
        }
    }

    private static MotorAutoTune.Result tune(String name) {
        List<StateMachine<?>> machines = new ArrayList<>();
        collect(state, machines);
        for (StateMachine<?> machine : machines) {
            for (Motor motor : machine.getMotors()) {
                if (MotorAutoTune.group(motor).equals(name)) {
                    run(MotorAutoTune.build(machine, motor));
                    MotorAutoTune.Result result = MotorAutoTune.lastResult(name).orElseThrow();
                    System.out.printf(Locale.ROOT, "%s: %s %s%n", name, result.ok() ? "ok" : "failed", result.summary());
                    return result;
                }
            }
        }
        throw new AssertionError("No motor named " + name);
    }

    private static void run(Command command) {
        CommandScheduler.getInstance().schedule(command);
        for (int loops = 0; command.isScheduled() && loops < kMaxLoops; loops++) {
            step(1);
        }
        assertTrue(!command.isScheduled(), command.getName() + " didn't finish");
    }

    private static double gain(MotorAutoTune.Result result, String gain) {
        return result.proposed().get(gain);
    }

    @Test
    void flywheelComesBackWithItsFeedforward() {
        MotorAutoTune.Result result = tune("Flywheel");
        assertTrue(result.ok(), result.summary());
        double metersPerRotation = 2.0 * Math.PI * edu.wpi.first.math.util.Units.inchesToMeters(2.0);
        double freeSpeed = edu.wpi.first.math.util.Units.radiansPerSecondToRotationsPerMinute(edu.wpi.first.math.system.plant.DCMotor.getNeoVortex(1).freeSpeedRadPerSec)
                * metersPerRotation / 60.0;
        assertEquals(12.0 / freeSpeed, gain(result, "kV"), 12.0 / freeSpeed * 0.05);
        assertTrue(gain(result, "kP") > 0.0);
        assertTrue(result.applied().contains("kP"));
    }

    @Test
    void hoodFindsItsGravity() {
        MotorAutoTune.Result result = tune("Hood");
        assertTrue(result.ok(), result.summary());
        assertEquals(0.401, gain(result, "kG"), 0.06);
        assertTrue(gain(result, "kP") > 0.0 && gain(result, "kD") >= 0.0);
    }

    @Test
    void intakeExtensionStaysInsideItsSetpointsAndFindsCosineGravity() {
        MotorAutoTune.Result result = tune("Intake Extension");
        assertTrue(result.ok(), result.summary());
        assertEquals(new IntakeConstants().extension.gains.kG, gain(result, "kCos"), 0.06);
    }

    @Test
    void rollersTuneToo() {
        for (String name : List.of("Intake Roller", "Hopper", "Kicker")) {
            MotorAutoTune.Result result = tune(name);
            assertTrue(result.ok(), name + ": " + result.summary());
            assertTrue(gain(result, "kP") > 0.0, name);
        }
    }

    @Test
    void steeringTunesTheTurnMotors() {
        MeasureSteering steering = new MeasureSteering(state);
        run(steering.build());
        MeasureAuto.Result result = steering.lastResult();
        System.out.println("Steering: " + result.outcome() + " " + result.summary());
        assertTrue(result.outcome() != MeasureAuto.Outcome.FAILED, result.summary());
        assertTrue(result.proposed().get("Turn Sim/kP") > 0.0);
    }
}
