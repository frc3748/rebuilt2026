package frc.robot.measure;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Locale;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.evergreen.ArenaEvergreen;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.SimNoise;
import frc.robot.commands.autos.MeasureAuto;
import frc.robot.commands.autos.MeasureAuto.Outcome;
import frc.robot.commands.autos.MeasureFeedforward;
import frc.robot.commands.autos.MeasureSlipCurrent;
import frc.robot.commands.autos.MeasureWheelRadius;
import frc.robot.robots.RobotDefinition;
import frc.robot.subsystems.drive.DriveConfig;

abstract class MeasureAutosTest {
    private static final double kGravity = 9.80665;
    private static final int kMaxLoops = 3000;
    private static final double kBumperMargin = 0.25;
    private static final double kStepVolts = 2.0;
    private static final int kStepLoops = 40;
    private static final int kEarlyLoop = 5;
    private static final double kBlockedSeconds = 10.0;
    private static RobotState state;
    private int collisions;

    protected static void boot(RobotDefinition definition) {
        assertTrue(HAL.initialize(500, 0));
        SimHooks.pauseTiming();
        SimNoise.seed();
        SimulatedArena.overrideInstance(arenaWithWalls());
        state = new RobotState(definition);
        step(25);
    }

    private static SimulatedArena arenaWithWalls() {
        return new ArenaEvergreen(false);
    }

    private static void step(int loops) {
        for (int i = 0; i < loops; i++) {
            SimHooks.stepTiming(0.02);
            CommandScheduler.getInstance().run();
            state.updateSimulation();
        }
    }

    @BeforeEach
    void enable() {
        collisions = state.getDrive().getCollision().events();
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    @AfterEach
    void disable() {
        assertEquals(collisions, state.getDrive().getCollision().events(), "Counted a collision that wasn't one");
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        step(10);
    }

    private void place(Pose2d pose) {
        state.getDrive().setPose(pose);
        step(10);
        state.getDrive().setPose(pose);
        step(10);
        collisions = state.getDrive().getCollision().events();
    }

    private MeasureAuto.Result run(MeasureAuto auto, Pose2d start) {
        place(start);
        Command command = auto.build();
        CommandScheduler.getInstance().schedule(command);
        for (int loops = 0; command.isScheduled() && loops < kMaxLoops; loops++) {
            step(1);
        }
        assertTrue(!command.isScheduled(), auto.name() + " didn't finish");
        MeasureAuto.Result result = auto.lastResult();
        assertNotNull(result, auto.name() + " didn't report");
        System.out.printf(Locale.ROOT, "%s %s: %s %s %s%n", state.getDefinition().name(), auto.name(), result.outcome(), result.summary(),
                result.measured());
        return result;
    }

    private static DriveConfig config() {
        return state.getDrive().getConfig();
    }

    private static Pose2d open() {
        return new Pose2d(4.0, 4.0, Rotation2d.kZero);
    }

    @Test
    void wheelRadiusMatchesTheWheels() {
        MeasureAuto.Result result = run(new MeasureWheelRadius(state), open());
        assertTrue(result.outcome() != Outcome.FAILED);
        assertEquals(config().wheelRadiusMeters, result.measured().get("RadiusMeters"), config().wheelRadiusMeters * 0.02);
    }

    @Test
    void feedforwardPredictsAVoltageStep() {
        MeasureAuto.Result result = run(new MeasureFeedforward(state), open());
        assertTrue(result.outcome() != Outcome.FAILED, result.summary());
        double kS = result.measured().get("kS");
        double kV = result.measured().get("kV");
        double kA = result.measured().get("kA");

        place(open());
        step(40);
        double predicted = 0.0;
        double early = 0.0;
        double earlyPredicted = 0.0;
        for (int loop = 1; loop <= kStepLoops; loop++) {
            state.getDrive().runCharacterization(kStepVolts);
            step(1);
            if (loop == kEarlyLoop) {
                early = state.getDrive().getWheelSpeedMetersPerSec();
                earlyPredicted = predicted;
            }
            predicted += (kStepVolts - kS - kV * predicted) / kA * 0.02;
        }
        double actual = state.getDrive().getWheelSpeedMetersPerSec();
        state.getDrive().stop();
        System.out.printf(Locale.ROOT, "%s %.0f V step: %.2f m/s after %.1f s (predicted %.2f), %.2f m/s after %.1f s (predicted %.2f)%n",
                state.getDefinition().name(), kStepVolts, early, kEarlyLoop * 0.02, earlyPredicted, actual, kStepLoops * 0.02, predicted);
        assertEquals(predicted, actual, predicted * 0.05);
        assertEquals(earlyPredicted, early, earlyPredicted * 0.1);
    }

    @Test
    void feedforwardStopsWhenItRunsIntoSomething() {
        double half = (config().wheelBase + kBumperMargin) / 2.0;
        double start = Timer.getFPGATimestamp();
        MeasureAuto.Result result = run(new MeasureFeedforward(state), new Pose2d(half + 0.8, 4.0, Rotation2d.k180deg));
        collisions = state.getDrive().getCollision().events();
        assertEquals(Outcome.FAILED, result.outcome());
        assertTrue(result.summary().contains("ran into something"), result.summary());
        assertTrue(Timer.getFPGATimestamp() - start < kBlockedSeconds, "Kept pushing into the wall");
    }

    @Test
    void slipCurrentMatchesTheGrip() {
        double half = (config().wheelBase + kBumperMargin) / 2.0;
        MeasureAuto.Result result = run(new MeasureSlipCurrent(state), new Pose2d(half + 0.01, 4.0, Rotation2d.k180deg));
        assertTrue(result.outcome() != Outcome.FAILED, String.valueOf(result.measured()));
        DCMotor motor = config().driveGearbox;
        double expected = config().wheelCOF * config().robotMassKg * kGravity / 4.0 * config().wheelRadiusMeters
                / (config().driveReduction * motor.KtNMPerAmp);
        if (result.measured().get("Slipped") == 1.0) {
            assertEquals(expected, result.measured().get("SlipAmps"), expected * 0.25);
        } else {
            assertTrue(result.measured().get("PeakAmps") < expected * 1.25, result.summary());
        }
    }

    @Test
    void hittingTheWallIsACollision() {
        double half = (config().wheelBase + kBumperMargin) / 2.0;
        place(new Pose2d(half + 1.0, 4.0, Rotation2d.k180deg));
        Command ram = Commands.run(() -> state.getDrive().runVelocity(new ChassisSpeeds(3.0, 0.0, 0.0)), state.getDrive())
                .withTimeout(1.5);
        CommandScheduler.getInstance().schedule(ram);
        boolean trusted = false;
        while (ram.isScheduled()) {
            step(1);
            trusted |= state.getDrive().getCollision().isUpset();
        }
        state.getDrive().stop();
        assertEquals(collisions + 1, state.getDrive().getCollision().events());
        assertTrue(trusted);
        collisions++;
    }

    @Test
    void slipCurrentNoticesThereIsNoWall() {
        MeasureAuto.Result result = run(new MeasureSlipCurrent(state), open());
        assertEquals(Outcome.FAILED, result.outcome());
    }
}
